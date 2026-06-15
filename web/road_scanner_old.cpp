/**
 * road_scanner.cpp
 * ============================================================
 * Implement cho LiDAR VB22A 1 tia, huong co dinh xuong mat duong.
 * Khong con xu ly encoder hoac mang 2D.
 * Luong xu ly:
 * - reader_loop(): Doc UART (10 bytes), kiem tra checksum, day vao Queue.
 * - process_loop(): Lay tu Queue, loc EMA, tinh toan hinh hoc bu phuoc, 
 * phat hien o ga/vat can qua delta_m, luu vao timeline.
 * ============================================================
 */

#include "road_scanner.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <chrono>

/* ============================================================
 * UART / SERIAL INTERFACE
 * ============================================================ */
bool RoadScanner::uart_open() {
    /* Set baudrate qua stty (chuyen dung cho Jetson/Linux) truoc khi mo */
    char cmd[160];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo 2>/dev/null",
             dev_.c_str(), baud_);
    (void)system(cmd);

    fd_ = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
        printf("[Road] Cannot open %s: %s\n", dev_.c_str(), strerror(errno));
        return false;
    }
    printf("[Road] Mở %s @ %d bps THÀNH CÔNG\n", dev_.c_str(), baud_);
    return true;
}

void RoadScanner::uart_close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

/* ============================================================
 * SPSC QUEUE (1 Producer - 1 Consumer)
 * ============================================================ */
bool RoadScanner::qpush(const PktQ& p) {
    int h = qh_.load(std::memory_order_relaxed);
    int next = (h + 1) % Q;
    if (next == qt_.load(std::memory_order_acquire)) return false; /* Queue day */
    qbuf_[h] = p;
    qh_.store(next, std::memory_order_release);
    return true;
}

bool RoadScanner::qpop(PktQ& p) {
    int t = qt_.load(std::memory_order_relaxed);
    if (t == qh_.load(std::memory_order_acquire)) return false; /* Queue rong */
    p = qbuf_[t];
    qt_.store((t + 1) % Q, std::memory_order_release);
    return true;
}

/* ============================================================
 * PARSE PACKET (10 BYTES)
 * Cau truc gia dinh tu STM32:
 * [0]=0xAA [1,2]=dist_mm [3,4,5,6]=ts_ms [7,8]=... [8]=chk [9]=0x55
 * ============================================================ */
bool RoadScanner::parse(const uint8_t* b, uint16_t& dist_mm, uint32_t& ts) {
    if (b[0] != RD_HDR || b[RD_PKT - 1] != RD_FTR) return false;

    /* Checksum XOR tu byte 1 den byte RD_PKT-3 */
    uint8_t chk = 0;
    for (int i = 1; i < RD_PKT - 2; i++) {
        chk ^= b[i];
    }
    if (chk != b[RD_PKT - 2]) return false;

    /* Little Endian parse */
    dist_mm = (uint16_t)b[1] | ((uint16_t)b[2] << 8);
    ts = (uint32_t)b[3] | ((uint32_t)b[4] << 8) | 
         ((uint32_t)b[5] << 16) | ((uint32_t)b[6] << 24);
         
    return true;
}

/* ============================================================
 * THREAD 1: READER (Đọc byte và đẩy vào hàng đợi)
 * ============================================================ */
void RoadScanner::reader_loop() {
    printf("[Road Reader] Đã khởi chạy\n"); fflush(stdout);

    uint8_t pkt[RD_PKT], b;
    
    while (running_.load(std::memory_order_relaxed)) {
        /* Tim byte Header */
        if (::read(fd_, &b, 1) != 1) continue;
        if (b != RD_HDR) continue;

        pkt[0] = RD_HDR;
        int got = 1;
        while (got < RD_PKT) {
            int r = ::read(fd_, pkt + got, RD_PKT - got);
            if (r > 0) got += r;
        }

        uint16_t dist_mm;
        uint32_t ts;
        if (!parse(pkt, dist_mm, ts)) continue;

        PktQ item{ dist_mm, ts };
        if (!qpush(item)) {
            /* Neu queue day, pop diem cu nhat de ghi de */
            PktQ dummy;
            qpop(dummy);
            qpush(item);
        }
    }
    printf("[Road Reader] Đã dừng\n");
}

/* ============================================================
 * THREAD 2: PROCESS (Xử lý lọc nhiễu, hình học và sự kiện)
 * ============================================================ */
void RoadScanner::process_loop() {
    printf("[Road Proc] Đã khởi chạy\n"); fflush(stdout);

    ema_init_ = false;
    baseline_n_ = 0;
    baseline_sum_ = 0.0;
    baseline_dist_ = 0.0f;
    baseline_pitch_rad_ = PITCH_STATIC_RAD; /* fallback truoc khi calib xong */
    calibrated_.store(false, std::memory_order_relaxed);
    pothole_confirm_ = 0;
    obstacle_confirm_ = 0;

    using namespace std::chrono_literals;

    while (running_.load(std::memory_order_relaxed)) {
        PktQ p;
        if (!qpop(p)) {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        pts_.fetch_add(1, std::memory_order_relaxed);
        
        float dist_raw_m = p.dist_mm / 1000.0f;
        last_dist_.store(p.dist_mm, std::memory_order_relaxed);

        /* Kiem tra tin hieu nhieu/out-of-range */
        if (p.dist_mm == 0xFFFF || dist_raw_m < DIST_MIN_M || dist_raw_m > DIST_MAX_M) {
            pothole_confirm_ = 0;
            obstacle_confirm_ = 0;
            continue;
        }

        /* 1. KHOI TAO & LOC EMA */
        if (!ema_init_) {
            ema_dist_ = dist_raw_m;
            ema_init_ = true;
        }
        ema_dist_ = (EMA_ALPHA * dist_raw_m) + ((1.0f - EMA_ALPHA) * ema_dist_);

        /* 2. HOC BASELINE - Bo qua phat hien o nhung mau dau tien de on dinh
         * Dong thoi tu calib goc nghieng: trung binh ema_dist_ trong giai doan
         * nay duoc coi la "mat duong phang" thuc te (xe phai dung tren mat
         * phang khi khoi dong), tu do tinh nguoc goc nghieng thuc te thay cho
         * PITCH_STATIC_RAD (gia tri thiet ke, thuong khong khop lap dat). */
        if (baseline_n_ < N_BASELINE) {
            baseline_sum_ += ema_dist_;
            baseline_n_++;

            if (baseline_n_ == N_BASELINE) {
                baseline_dist_ = (float)(baseline_sum_ / N_BASELINE);

                float ratio = H_MOUNT / baseline_dist_;
                if (ratio > 1.0f) ratio = 1.0f;
                if (ratio < -1.0f) ratio = -1.0f;
                float pitch = asinf(ratio);
                if (pitch < PITCH_MIN_RAD) pitch = PITCH_MIN_RAD;
                if (pitch > PITCH_MAX_RAD) pitch = PITCH_MAX_RAD;
                baseline_pitch_rad_ = pitch;

                uint32_t bd_u32, bp_u32;
                std::memcpy(&bd_u32, &baseline_dist_, sizeof(baseline_dist_));
                std::memcpy(&bp_u32, &baseline_pitch_rad_, sizeof(baseline_pitch_rad_));
                baseline_dist_raw_.store(bd_u32, std::memory_order_relaxed);
                baseline_pitch_raw_.store(bp_u32, std::memory_order_relaxed);
                calibrated_.store(true, std::memory_order_relaxed);

                printf("[Road Proc] Tu calib xong: baseline_dist=%.3f m, "
                       "pitch=%.2f deg (thiet ke: SLANT_RANGE_REF=%.2f m, "
                       "pitch=%.2f deg)\n",
                       baseline_dist_, baseline_pitch_rad_ * (180.0f / 3.14159265f),
                       SLANT_RANGE_REF, PITCH_STATIC_RAD * (180.0f / 3.14159265f));
                fflush(stdout);

                if (fabsf(baseline_dist_ - SLANT_RANGE_REF) > 0.3f) {
                    printf("[Road Proc] CANH BAO: baseline_dist (%.3f m) lech "
                           "nhieu so voi SLANT_RANGE_REF thiet ke (%.2f m). "
                           "Kiem tra lai goc/do cao lap dat LiDAR, hoac dam "
                           "bao xe dang dung tren mat duong PHANG luc khoi dong.\n",
                           baseline_dist_, SLANT_RANGE_REF);
                    fflush(stdout);
                }
            }
            continue;
        }

        /* 3. TINH TOAN HINH HOC & BU PHUOC
         * expected_dist: khoang cach can co de tia cham mat duong PHANG
         * delta_m: expected_dist - ema_dist_
         * - Am: do xa hon du kien (tia xuyen xuong lo) -> O GA
         * - Duong: do gan hon du kien (tia bi chan som) -> VAT CAN
         *
         * baseline_pitch_rad_ duoc tu calib tu N_BASELINE mau dau (xem o tren),
         * thay cho PITCH_STATIC_RAD (gia tri thiet ke). Nho vay tai dung diem
         * baseline (mat duong phang luc khoi dong, phuoc khong nen), delta = 0
         * mot cach tu nhien, khong phu thuoc viec do dac H_MOUNT/SLANT_RANGE_REF
         * co chinh xac voi thuc te lap dat hay khong.
         */
        float pitch_tot = baseline_pitch_rad_ + get_pitch_dynamic();
        float dist_exp = expected_dist(pitch_tot);
        float delta = dist_exp - ema_dist_;

        /* Cap nhat thuoc tinh last_delta de web/API lay nhanh */
        uint32_t delta_u32;
        std::memcpy(&delta_u32, &delta, sizeof(delta));
        last_delta_raw_.store(delta_u32, std::memory_order_relaxed);

        RoadSample sample{};
        sample.ts_ms = p.ts;
        sample.dist_raw_m = dist_raw_m;
        sample.dist_ema_m = ema_dist_;
        sample.dist_expect_m = dist_exp;
        sample.delta_m = delta;
        sample.is_valid = true;
        sample.is_pothole = false;
        sample.is_obstacle = false;

        /* 4. LOGIC XAC NHAN SU KIEN (CONFIRMATION) */
        if (delta < -POTHOLE_THRESH_M) {
            pothole_confirm_++;
            obstacle_confirm_ = 0;
            if (pothole_confirm_ >= N_CONFIRM) {
                sample.is_pothole = true;
                npot_.fetch_add(1, std::memory_order_relaxed);
                
                if (cb_pot_) cb_pot_(sample);

                /* Luu su kien O ga */
                PotholeEvent pe;
                pe.ts_ms = p.ts;
                pe.depth_m = -delta; /* chuyen gia tri am thanh do sau duong */
                /* Tam chieu: cach dau xe bao xa */
                pe.x_ahead_m = (ema_dist_ * cosf(pitch_tot)) + LIDAR_OY;

                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    potholes_.push_back(pe);
                    if (potholes_.size() > 50) potholes_.erase(potholes_.begin());
                }
            }
        } else if (delta > OBSTACLE_THRESH_M) {
            obstacle_confirm_++;
            pothole_confirm_ = 0;
            if (obstacle_confirm_ >= N_CONFIRM) {
                sample.is_obstacle = true;
                nobs_.fetch_add(1, std::memory_order_relaxed);
                
                if (cb_obs_) cb_obs_(sample);
            }
        } else {
            /* Quay ve mat phang */
            pothole_confirm_ = 0;
            obstacle_confirm_ = 0;
        }

        /* 5. GHI VAO TIMELINE DE VE BIEU DO GUI LEN WEB/UI */
        {
            std::lock_guard<std::mutex> lk(mtx_);
            timeline_.push_back(sample);
            if (timeline_.size() > TIMELINE_LEN) {
                timeline_.pop_front();
            }
        }
    }
    printf("[Road Proc] Đã dừng. Tổng điểm: %u\n", pts_.load());
}

/* ============================================================
 * QUAN LY VONG DOI (START / STOP)
 * ============================================================ */
void RoadScanner::start() {
    if (!uart_open()) return;
    
    pts_.store(0);
    npot_.store(0);
    nobs_.store(0);
    qh_.store(0);
    qt_.store(0);
    running_.store(true);
    
    thr_r_ = std::thread(&RoadScanner::reader_loop, this);
    thr_p_ = std::thread(&RoadScanner::process_loop, this);
}

void RoadScanner::stop() {
    running_.store(false);
    uart_close();  /* Giai phong tap tin de ngat block::read */
    
    if (thr_r_.joinable()) thr_r_.join();
    if (thr_p_.joinable()) thr_p_.join();
}