/**
 * front_scanner.cpp
 * ============================================================
 * id 0-3 : chieu thang, tinh wx/wy binh thuong
 * id 4   : LiDAR nghieng, tinh delta phat hien o ga / vat can
 *          (logic tuong tu road_scanner.cpp nhung chay chung luong)
 * ============================================================
 */
#include "front_scanner.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <thread>

/* ============================================================
 * UART
 * ============================================================ */
bool FrontScanner::uart_open() {
    char cmd[160];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo 2>/dev/null",
             dev_.c_str(), baud_);
    (void)system(cmd);
    fd_ = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
        printf("[Front] Khong mo duoc %s: %s\n", dev_.c_str(), strerror(errno));
        return false;
    }
    printf("[Front] Mo %s @ %d THANH CONG\n", dev_.c_str(), baud_);
    return true;
}

void FrontScanner::uart_close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

/* ============================================================
 * PARSE PACKET 12 BYTES
 * ============================================================ */
bool FrontScanner::parse_packet(const uint8_t* b, uint16_t& dist_mm_out,
                                 float& ox_m, float& oy_m, uint8_t& id_out) {
    if (b[0] != FRONT_PKT_HDR || b[FRONT_PKT_LEN-1] != FRONT_PKT_FTR) return false;
    if (b[1] >= FRONT_N_LIDAR) return false;

    uint8_t chk = 0;
    for (int i = 1; i <= 9; i++) chk ^= b[i];
    if (chk != b[10]) return false;

    id_out        = b[1];
    dist_mm_out   = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
    /* angle_10 o b[4,5]: chi dung cho id 0-3, id=4 Jetson dung pitch rieng */
    ox_m          = (int16_t)((uint16_t)b[6] | ((uint16_t)b[7] << 8)) / 1000.0f;
    oy_m          = (int16_t)((uint16_t)b[8] | ((uint16_t)b[9] << 8)) / 1000.0f;
    return true;
}

/* ============================================================
 * XU LY ID 0-3: CHIEU THANG
 *   wx = ox_m (cos90=0 nen dist khong anh huong X)
 *   wy = oy_m + dist_m (sin90=1)
 * ============================================================ */
FrontPoint FrontScanner::process_straight(uint8_t id, float dist_m,
                                           float ox_m, float oy_m) {
    FrontPoint pt{};
    pt.id        = id;
    pt.dist_m    = dist_m;
    pt.angle_deg = 90.0f;
    pt.ox_m      = ox_m;
    pt.oy_m      = oy_m;
    pt.wx        = ox_m;              /* cos(90)=0 */
    pt.wy        = oy_m + dist_m;    /* sin(90)=1 */
    pt.delta_m      = 0.0f;
    pt.is_pothole   = false;
    pt.is_obstacle  = false;
    pt.ts_local  = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);
    return pt;
}

/* ============================================================
 * XU LY ID 4: LIDAR NGHIENG (logic tuong tu road_scanner.cpp)
 *
 * expected_dist = H_MOUNT / sin(PITCH_STATIC_RAD)
 * delta = expected_dist - ema_dist
 *   delta < -THRESH -> O GA  (tia di xa hon du kien, xuyen xuong lo)
 *   delta > +THRESH -> VAT CAN (tia bi chan som)
 *
 * Diem map:
 *   wx = 0   (tia chui chinh giua xe)
 *   wy = oy_m + ema_dist * cos(pitch)  (chieu xuong mat duong)
 * ============================================================ */
FrontPoint FrontScanner::process_tilt(float dist_raw_m, float ox_m, float oy_m) {
    FrontPoint pt{};
    pt.id      = FRONT_ID_TILT;
    pt.dist_m  = dist_raw_m;
    pt.ox_m    = ox_m;
    pt.oy_m    = oy_m;
    pt.angle_deg = 90.0f; /* placeholder, khong dung de map */
    pt.ts_local  = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);

    /* Kiem tra out-of-range */
    if (dist_raw_m < FRONT_DIST_MIN_M || dist_raw_m > FRONT_DIST_MAX_M) {
        pothole_confirm_  = 0;
        obstacle_confirm_ = 0;
        pt.is_pothole = pt.is_obstacle = false;
        pt.delta_m = 0.0f;
        return pt;
    }

    /* EMA loc rung */
    if (!ema_init_) { ema_dist_ = dist_raw_m; ema_init_ = true; }
    ema_dist_ = FRONT_EMA_ALPHA * dist_raw_m + (1.0f - FRONT_EMA_ALPHA) * ema_dist_;

    /* Hoc baseline: bo qua FRONT_N_BASELINE mau dau */
    if (baseline_n_ < FRONT_N_BASELINE) {
        baseline_n_++;
        pt.is_pothole = pt.is_obstacle = false;
        pt.delta_m = 0.0f;
        return pt;
    }

    /* Tinh delta */
    float dist_exp = expected_dist(FRONT_PITCH_STATIC_RAD);
    float delta    = dist_exp - ema_dist_;
    pt.delta_m = delta;

    /* Diem map tren mat duong */
    pt.wx = 0.0f;
    pt.wy = oy_m + ema_dist_ * cosf(FRONT_PITCH_STATIC_RAD);

    /* Confirm su kien */
    if (delta < -FRONT_POTHOLE_THRESH_M) {
        pothole_confirm_++;
        obstacle_confirm_ = 0;
        pt.is_pothole  = (pothole_confirm_ >= FRONT_N_CONFIRM);
        pt.is_obstacle = false;
    } else if (delta > FRONT_OBSTACLE_THRESH_M) {
        obstacle_confirm_++;
        pothole_confirm_  = 0;
        pt.is_obstacle = (obstacle_confirm_ >= FRONT_N_CONFIRM);
        pt.is_pothole  = false;
    } else {
        pothole_confirm_  = 0;
        obstacle_confirm_ = 0;
        pt.is_pothole = pt.is_obstacle = false;
    }

    return pt;
}

/* ============================================================
 * READER LOOP
 * ============================================================ */
void FrontScanner::reader_loop() {
    printf("[Front] Reader khoi chay port=%s\n", dev_.c_str());
    fflush(stdout);

    /* Reset trang thai id=4 */
    ema_init_         = false;
    baseline_n_       = 0;
    pothole_confirm_  = 0;
    obstacle_confirm_ = 0;

    uint8_t pkt[FRONT_PKT_LEN], b;

    while (running_.load(std::memory_order_relaxed)) {
        /* Tim header 0xCC */
        if (::read(fd_, &b, 1) != 1) continue;
        if (b != FRONT_PKT_HDR) continue;

        pkt[0] = b;
        int got = 1;
        while (got < FRONT_PKT_LEN) {
            int r = ::read(fd_, pkt + got, FRONT_PKT_LEN - got);
            if (r > 0) got += r;
            else if (r < 0 && errno != EINTR) goto next_pkt;
        }

        {
            uint16_t dist_mm; float ox_m, oy_m; uint8_t id;
            if (!parse_packet(pkt, dist_mm, ox_m, oy_m, id)) goto next_pkt;

            float dist_m = dist_mm / 1000.0f;

            pts_.fetch_add(1, std::memory_order_relaxed);
            pts_per_id_[id].fetch_add(1, std::memory_order_relaxed);
            uint32_t raw; std::memcpy(&raw, &dist_m, sizeof(raw));
            last_dist_raw_[id].store(raw, std::memory_order_relaxed);

            FrontPoint pt;
            if (id == FRONT_ID_TILT)
                pt = process_tilt(dist_m, ox_m, oy_m);
            else
                pt = process_straight(id, dist_m, ox_m, oy_m);

            if (cb_) cb_(pt);
        }
        next_pkt:;
    }
    printf("[Front] Reader dung. Tong: %u\n", pts_.load());
}

/* ============================================================
 * VONG DOI
 * ============================================================ */
void FrontScanner::start() {
    if (!uart_open()) return;
    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        pts_per_id_[i].store(0);
        last_dist_raw_[i].store(0);
    }
    pts_.store(0);
    running_.store(true);
    thr_ = std::thread(&FrontScanner::reader_loop, this);
}

void FrontScanner::stop() {
    running_.store(false);
    uart_close();
    if (thr_.joinable()) thr_.join();
}