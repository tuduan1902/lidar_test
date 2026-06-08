#include "grid_manager.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

// ============================================================
//  UART OPEN (dung stty cho Jetson THS UART)
// ============================================================
bool GridManager::uart_open(const std::string& dev, int baud, int& fd) {
    // Dung stty vi Jetson THS UART khong ho tro tcsetattr dung
    char cmd[160];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo 2>/dev/null",
             dev.c_str(), baud);
    system(cmd);

    fd = ::open(dev.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        printf("[WARN] Cannot open %s: %s\n", dev.c_str(), strerror(errno));
        return false;
    }
    printf("[UART] %-20s @ %d OK (fd=%d)\n", dev.c_str(), baud, fd);
    return true;
}

void GridManager::uart_close(int& fd) {
    if (fd >= 0) { ::close(fd); fd = -1; }
}

// ============================================================
//  PARSE LIDAR PACKET 14 BYTES
//  [0]=0xAA [1]=id [2..3]=dist_mm [4..5]=angle*10
//  [6..9]=ts [10..11]=enc_lo [12]=chk [13]=0x55
// ============================================================
bool GridManager::parse_lidar(const uint8_t* b, LidarPoint& o) {
    if (b[0] != LIDAR_HDR || b[LIDAR_PKT-1] != LIDAR_FTR) return false;
    uint8_t chk = 0;
    for (int i = 1; i <= LIDAR_PKT-3; i++) chk ^= b[i];
    if (chk != b[LIDAR_PKT-2]) return false;

    o.dist_mm  = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
    int16_t a  = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    o.angle_deg = a / 10.0f;
    o.ts_ms    = (uint32_t)b[6] | ((uint32_t)b[7]<<8) |
                 ((uint32_t)b[8]<<16) | ((uint32_t)b[9]<<24);
    return true;
}

// ============================================================
//  PARSE ULTRASONIC JSON LINE
//  Format: {"us_1": 45.2, "us_2": 120.0, "us_3": -1.0, "us_4": 88.5}
// ============================================================
bool GridManager::parse_us_json(const char* line, UsPoint& o) {
    // Reset
    for (int i = 0; i < 4; i++) o.dist_cm[i] = -1.0f;

    // sscanf parse don gian, khong phu thuoc JSON library
    float v1=-1, v2=-1, v3=-1, v4=-1;
    int n = sscanf(line,
        "{\"us_1\": %f, \"us_2\": %f, \"us_3\": %f, \"us_4\": %f}",
        &v1, &v2, &v3, &v4);
    if (n < 1) return false;

    o.dist_cm[0] = v1;
    o.dist_cm[1] = v2;
    o.dist_cm[2] = v3;
    o.dist_cm[3] = v4;
    return true;
}

// ============================================================
//  MARK GRID HELPERS
// ============================================================
void GridManager::mark_xy(float wx, float wy) {
    int gx = GRID_OX + (int)(wx / CELL_M);
    int gy = GRID_OY + (int)(wy / CELL_M);
    if (gx < 0 || gx >= GRID_N) return;
    if (gy < 0 || gy >= GRID_N) return;
    __atomic_store_n(&data[gy*GRID_N+gx], OBSTACLE, __ATOMIC_RELAXED);
}

void GridManager::mark_lidar(const LidarPoint& p) {
    if (p.dist_mm == 0xFFFF || p.dist_mm < 30) return;
    float rad    = (LIDAR_MOUNT_DEG + p.angle_deg) * (float)M_PI / 180.0f;
    float dist_m = p.dist_mm / 1000.0f;
    float wx = LIDAR_OX + dist_m * std::sin(rad);
    float wy = LIDAR_OY + dist_m * std::cos(rad);
    mark_xy(wx, wy);
}

void GridManager::mark_us(const UsPoint& p) {
    for (int i = 0; i < 4; i++) {
        float cm = p.dist_cm[i];
        // -1 = invalid, < min = blind zone, > max = ngoai tam
        if (cm < 20.0f || cm > 600.0f) continue;

        float rad    = US_MOUNTS[i].dir_deg * (float)M_PI / 180.0f;
        float dist_m = cm / 100.0f;
        float wx = US_MOUNTS[i].ox + dist_m * std::sin(rad);
        float wy = US_MOUNTS[i].oy + dist_m * std::cos(rad);
        mark_xy(wx, wy);
    }
}

// ============================================================
//  THREAD 1: LIDAR READER
// ============================================================
static bool readbyte(int fd, uint8_t& b) {
    while (true) {
        ssize_t n = ::read(fd, &b, 1);
        if (n == 1) return true;
        if (n == 0) return false;
        if (errno == EINTR) continue;
        return false;
    }
}

void GridManager::lidar_reader_loop() {
    printf("[LiDAR Reader] started  dev=%s\n", lidar_dev_.c_str());
    fflush(stdout);

    uint8_t  pkt[LIDAR_PKT], b;
    uint32_t raw=0, aa=0, pkts=0, bad=0;

    while (running_.load(std::memory_order_relaxed)) {
        if (!readbyte(lidar_fd_, b)) break;
        raw++;
        if (b != LIDAR_HDR) continue;

        aa++;
        pkt[0] = LIDAR_HDR;
        bool ok = true;
        for (int i = 1; i < LIDAR_PKT; i++) {
            if (!readbyte(lidar_fd_, pkt[i])) { ok=false; break; }
            raw++;
        }
        if (!ok) break;
        if (pkt[LIDAR_PKT-1] != LIDAR_FTR) { bad++; continue; }

        LidarPoint lp;
        if (parse_lidar(pkt, lp)) {
            pkts++;
            if (pkts <= 3) {
                printf("[LiDAR] dist=%umm angle=%.1fdeg\n",
                       lp.dist_mm, lp.angle_deg);
                fflush(stdout);
            }
            if (!lidar_q_.push(lp)) {
                LidarPoint d; lidar_q_.pop(d); lidar_q_.push(lp);
            }
        }
    }
    printf("[LiDAR Reader] stopped raw=%u aa=%u pkts=%u bad=%u\n",
           raw, aa, pkts, bad);
}

// ============================================================
//  THREAD 2: ULTRASONIC READER (doc theo dong JSON)
// ============================================================
void GridManager::us_reader_loop() {
    printf("[US Reader] started  dev=%s\n", us_dev_.c_str());
    fflush(stdout);

    char    line[256];
    int     lpos = 0;
    uint8_t b;
    uint32_t pkts=0, errs=0;

    while (running_.load(std::memory_order_relaxed)) {
        if (!readbyte(us_fd_, b)) break;

        if (b == '\n' || b == '\r') {
            if (lpos > 5) {
                line[lpos] = '\0';
                UsPoint up;
                if (parse_us_json(line, up)) {
                    pkts++;
                    if (pkts <= 3) {
                        printf("[US] S1=%.1f S2=%.1f S3=%.1f S4=%.1f cm\n",
                               up.dist_cm[0], up.dist_cm[1],
                               up.dist_cm[2], up.dist_cm[3]);
                        fflush(stdout);
                    }
                    if (!us_q_.push(up)) {
                        UsPoint d; us_q_.pop(d); us_q_.push(up);
                    }
                } else {
                    errs++;
                }
            }
            lpos = 0;
        } else {
            if (lpos < (int)sizeof(line)-1)
                line[lpos++] = (char)b;
        }
    }
    printf("[US Reader] stopped pkts=%u errs=%u\n", pkts, errs);
}

// ============================================================
//  THREAD 3: GRID UPDATER (xu ly ca 2 queue)
// ============================================================
void GridManager::updater_loop() {
    printf("[Updater] started\n"); fflush(stdout);
    using namespace std::chrono_literals;

    while (running_.load(std::memory_order_relaxed)) {
        bool did_work = false;

        // Pop tat ca LiDAR points co san
        LidarPoint lp;
        while (lidar_q_.pop(lp)) {
            mark_lidar(lp);
            lidar_count_.fetch_add(1, std::memory_order_relaxed);
            did_work = true;
        }

        // Pop tat ca US points co san
        UsPoint up;
        while (us_q_.pop(up)) {
            mark_us(up);
            us_count_.fetch_add(1, std::memory_order_relaxed);
            did_work = true;
        }

        if (!did_work)
            std::this_thread::sleep_for(200us);
    }
    printf("[Updater] stopped\n");
}

// ============================================================
//  THREAD 4: DECAY
// ============================================================
void GridManager::decay_loop() {
    printf("[Decay] started  step=%d interval=%dms\n",
           DECAY_STEP, DECAY_MS);
    fflush(stdout);

    while (running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(DECAY_MS));
        for (int i = 0; i < GRID_N * GRID_N; i++) {
            uint8_t v = __atomic_load_n(&data[i], __ATOMIC_RELAXED);
            if (v == 0) continue;
            uint8_t nv = (v > DECAY_STEP) ? (v - DECAY_STEP) : 0;
            __atomic_store_n(&data[i], nv, __ATOMIC_RELAXED);
        }
    }
    printf("[Decay] stopped\n");
}

// ============================================================
//  START / STOP
// ============================================================
void GridManager::start() {
    // Mo ca 2 UART (neu khong mo duoc thi thread van chay nhung
    // readbyte se fail ngay -> thread exit nhanh, khong crash)
    bool lidar_ok = uart_open(lidar_dev_, lidar_baud_, lidar_fd_);
    bool us_ok    = uart_open(us_dev_,    us_baud_,    us_fd_);

    if (!lidar_ok) printf("[WARN] LiDAR device unavailable\n");
    if (!us_ok)    printf("[WARN] Ultrasonic device unavailable\n");

    running_.store(true);
    thr_lidar_r_  = std::thread(&GridManager::lidar_reader_loop, this);
    thr_us_r_     = std::thread(&GridManager::us_reader_loop,    this);
    thr_updater_  = std::thread(&GridManager::updater_loop,      this);
    thr_decay_    = std::thread(&GridManager::decay_loop,        this);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void GridManager::stop() {
    running_.store(false);
    // Dong fd de unblock readbyte() dang block
    uart_close(lidar_fd_);
    uart_close(us_fd_);
    if (thr_lidar_r_.joinable()) thr_lidar_r_.join();
    if (thr_us_r_.joinable())    thr_us_r_.join();
    if (thr_updater_.joinable()) thr_updater_.join();
    if (thr_decay_.joinable())   thr_decay_.join();
}