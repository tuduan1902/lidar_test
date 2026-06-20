/**
 * rear_lidar_3.cpp
 * ============================================================
 * Xu ly du lieu 3 LiDAR VB22A gan sau duoi xe may (phia Jetson).
 * ============================================================
 */

#include "rear_lidar_3.hpp"
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
bool RearScanner::uart_open() {
    /* Cau hinh port truoc khi mo (stty, tuong tu road_scanner) */
    char cmd[160];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo 2>/dev/null",
             dev_.c_str(), baud_);
    (void)system(cmd);

    fd_ = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
        printf("[Rear] Khong mo duoc %s: %s\n", dev_.c_str(), strerror(errno));
        return false;
    }
    printf("[Rear] Mo %s @ %d bps THANH CONG\n", dev_.c_str(), baud_);
    return true;
}

void RearScanner::uart_close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

/* ============================================================
 * PARSE 1 PACKET 14 BYTES
 * ============================================================ */
bool RearScanner::parse(const uint8_t* b, RearPoint& out) {
    /* Kiem tra header / footer */
    if (b[0] != REAR_PKT_HDR || b[REAR_PKT_LEN - 1] != REAR_PKT_FTR) return false;

    /* Kiem tra id hop le */
    uint8_t id = b[1];
    if (id >= REAR_N_LIDAR) return false;

    /* Checksum XOR byte[1..11] */
    uint8_t chk = 0;
    for (int i = 1; i <= 11; i++) chk ^= b[i];
    if (chk != b[12]) return false;

    /* Parse cac truong */
    uint16_t dist_cm  = (uint16_t)b[2]  | ((uint16_t)b[3]  << 8);
    uint16_t strength = (uint16_t)b[4]  | ((uint16_t)b[5]  << 8);
    uint16_t temp_raw = (uint16_t)b[6]  | ((uint16_t)b[7]  << 8);
    int16_t  angle_10 = (int16_t)((uint16_t)b[8] | ((uint16_t)b[9] << 8));
    int16_t  ox_mm    = (int16_t)((uint16_t)b[10] | ((uint16_t)b[11] << 8));

    float dist_m    = dist_cm / 100.0f;
    float angle_deg = angle_10 / 10.0f;
    float ox_m      = ox_mm   / 1000.0f;

    /* Loc khoang cach */
    if (dist_m < REAR_DIST_MIN_M || dist_m > REAR_DIST_MAX_M) return false;

    /* Tinh toa do the gioi (he toa do xe: +Y truoc, +X phai) */
    float angle_rad = angle_deg * (float)(M_PI / 180.0);
    float wx = ox_m + dist_m * cosf(angle_rad);
    float wy = REAR_MOUNT_OY_M + dist_m * sinf(angle_rad);

    /* Dien vao output */
    out.id        = id;
    out.dist_m    = dist_m;
    out.angle_deg = angle_deg;
    out.wx        = wx;
    out.wy        = wy;
    out.ox_m      = ox_m;
    out.strength  = strength;
    /* Nhiet do: (temp_raw / 8) - 256 theo datasheet TF-Luna/VB22A */
    out.temp_c    = (temp_raw / 8.0f) - 256.0f;

    /* Timestamp cuc bo Jetson */
    out.ts_local  = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);

    return true;
}

/* ============================================================
 * READER THREAD
 * ============================================================ */
void RearScanner::reader_loop() {
    printf("[Rear] Reader khoi chay, port=%s\n", dev_.c_str());
    fflush(stdout);

    uint8_t pkt[REAR_PKT_LEN];
    uint8_t b;

    while (running_.load(std::memory_order_relaxed)) {
        /* Tim byte header 0xBB */
        if (::read(fd_, &b, 1) != 1) continue;
        if (b != REAR_PKT_HDR) continue;

        pkt[0] = b;
        int got = 1;
        while (got < REAR_PKT_LEN) {
            int r = ::read(fd_, pkt + got, REAR_PKT_LEN - got);
            if (r > 0) got += r;
            else if (r < 0 && errno != EINTR) goto next_pkt;
        }

        {
            RearPoint pt;
            if (parse(pkt, pt)) {
                pts_.fetch_add(1, std::memory_order_relaxed);
                pts_per_id_[pt.id].fetch_add(1, std::memory_order_relaxed);

                /* Luu khoang cach cuoi (atomic float via memcpy) */
                uint32_t raw; std::memcpy(&raw, &pt.dist_m, sizeof(raw));
                last_dist_raw_[pt.id].store(raw, std::memory_order_relaxed);

                if (cb_) cb_(pt);
            }
        }

        next_pkt:;
    }

    printf("[Rear] Reader dung. Tong diem: %u\n", pts_.load());
}

/* ============================================================
 * QUAN LY VONG DOI
 * ============================================================ */
void RearScanner::start() {
    if (!uart_open()) return;
    for (int i = 0; i < REAR_N_LIDAR; i++) {
        pts_per_id_[i].store(0);
        last_dist_raw_[i].store(0);
    }
    pts_.store(0);
    running_.store(true);
    thr_ = std::thread(&RearScanner::reader_loop, this);
}

void RearScanner::stop() {
    running_.store(false);
    uart_close();
    if (thr_.joinable()) thr_.join();
}