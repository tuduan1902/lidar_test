/**
 * front_scanner.cpp
 * ============================================================
 * Xu ly du lieu 4 LiDAR VB22A dau xe (phia Jetson).
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
    printf("[Front] Mo %s @ %d bps THANH CONG\n", dev_.c_str(), baud_);
    return true;
}

void FrontScanner::uart_close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool FrontScanner::parse(const uint8_t* b, FrontPoint& out) {
    /* Kiem tra header / footer */
    if (b[0] != FRONT_PKT_HDR || b[FRONT_PKT_LEN-1] != FRONT_PKT_FTR) return false;

    uint8_t id = b[1];
    if (id >= FRONT_N_LIDAR) return false;

    /* Checksum XOR byte[1..9] */
    uint8_t chk = 0;
    for (int i = 1; i <= 9; i++) chk ^= b[i];
    if (chk != b[10]) return false;

    uint16_t dist_mm  = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
    int16_t  angle_10 = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    int16_t  ox_mm    = (int16_t)((uint16_t)b[6] | ((uint16_t)b[7] << 8));
    int16_t  oy_mm    = (int16_t)((uint16_t)b[8] | ((uint16_t)b[9] << 8));

    float dist_m    = dist_mm  / 1000.0f;  /* mm -> m */
    float angle_deg = angle_10 / 10.0f;   /* luon 90.0 */
    float ox_m      = ox_mm    / 1000.0f;
    float oy_m      = oy_mm    / 1000.0f;

    if (dist_m < FRONT_DIST_MIN_M || dist_m > FRONT_DIST_MAX_M) return false;

    /* Tinh toa do (angle=90 do -> cos=0, sin=1) */
    float angle_rad = angle_deg * (float)(M_PI / 180.0);
    out.id        = id;
    out.dist_m    = dist_m;
    out.angle_deg = angle_deg;
    out.ox_m      = ox_m;
    out.oy_m      = oy_m;
    out.wx        = ox_m + dist_m * cosf(angle_rad); /* ~ ox_m */
    out.wy        = oy_m + dist_m * sinf(angle_rad); /* ~ oy_m + dist_m */
    out.ts_local  = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);

    return true;
}

void FrontScanner::reader_loop() {
    printf("[Front] Reader khoi chay, port=%s\n", dev_.c_str());
    fflush(stdout);

    uint8_t pkt[FRONT_PKT_LEN];
    uint8_t b;

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
            FrontPoint pt;
            if (parse(pkt, pt)) {
                pts_.fetch_add(1, std::memory_order_relaxed);
                pts_per_id_[pt.id].fetch_add(1, std::memory_order_relaxed);
                uint32_t raw; std::memcpy(&raw, &pt.dist_m, sizeof(raw));
                last_dist_raw_[pt.id].store(raw, std::memory_order_relaxed);
                if (cb_) cb_(pt);
            }
        }
        next_pkt:;
    }
    printf("[Front] Reader dung. Tong diem: %u\n", pts_.load());
}

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