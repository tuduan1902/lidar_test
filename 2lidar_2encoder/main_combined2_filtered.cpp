/**
 * main_combined2_filtered.cpp
 * Separate filtered variant for 2x VB22A LiDAR + 4x ultrasonic.
 * Does not overwrite existing grid_manager2.* files.
 *
 * Build:
 *   g++ -std=c++17 -O2 -lpthread main_combined2_filtered.cpp -o lidar_us2_filtered
 * Run:
 *   ./lidar_us2_filtered /dev/ttyTHS1 /dev/ttyUSB0
 */

#include "grid_manager2.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>
#include <array>
#include <fcntl.h>
#include <unistd.h>

static std::atomic<bool> g_quit{false};
static void on_sig(int) { g_quit.store(true); }

static constexpr float PI = 3.14159265358979323846f;
static constexpr uint16_t LIDAR_MIN_MM = 30;
static constexpr uint16_t LIDAR_MAX_MM = 5000;
static constexpr float  LIDAR_MAX_ANGLE = 110.0f;

struct FilteredMap {
    static constexpr uint8_t HIT_LIGHT  = 2;
    static constexpr uint8_t HIT_STRONG = 6;
    static constexpr uint8_t VAL_LIGHT  = THRESH_FAINT;
    static constexpr uint8_t VAL_STRONG = THRESH_STRONG;
    static constexpr uint8_t VAL_EMPTY  = FREE;

    std::mutex mutex;
    std::array<uint8_t, GRID_N * GRID_N> cells;
    std::array<uint8_t, GRID_N * GRID_N> hits;
    std::array<uint8_t, GRID_N * GRID_N> sources;

    FilteredMap() {
        cells.fill(VAL_EMPTY);
        hits.fill(0);
        sources.fill(0);
    }

    bool valid_cell(int gx, int gy) const {
        return gx >= 0 && gx < GRID_N && gy >= 0 && gy < GRID_N;
    }

    void update_cell(int gx, int gy, uint8_t add, uint8_t src) {
        if (!valid_cell(gx, gy)) return;
        int idx = gy * GRID_N + gx;
        std::lock_guard<std::mutex> lock(mutex);
        uint8_t new_hits = hits[idx] + add;
        if (new_hits < hits[idx]) new_hits = 255;
        hits[idx] = new_hits;
        if (src != 0) sources[idx] = src;
        if (hits[idx] >= HIT_STRONG) {
            cells[idx] = VAL_STRONG;
        } else if (hits[idx] >= HIT_LIGHT) {
            cells[idx] = VAL_LIGHT;
        } else {
            cells[idx] = VAL_EMPTY;
        }
    }

    void decay() {
        std::lock_guard<std::mutex> lock(mutex);
        for (int i = 0; i < GRID_N * GRID_N; i++) {
            if (hits[i] == 0) {
                cells[i] = VAL_EMPTY;
                sources[i] = 0;
                continue;
            }
            hits[i] -= 1;
            if (hits[i] >= HIT_STRONG) {
                cells[i] = VAL_STRONG;
            } else if (hits[i] >= HIT_LIGHT) {
                cells[i] = VAL_LIGHT;
            } else {
                cells[i] = VAL_EMPTY;
            }
        }
    }

    void snapshot(uint8_t* dst) {
        std::lock_guard<std::mutex> lock(mutex);
        std::memcpy(dst, cells.data(), cells.size());
    }

    void snapshot(uint8_t* dst, uint8_t* src_dst) {
        std::lock_guard<std::mutex> lock(mutex);
        std::memcpy(dst, cells.data(), cells.size());
        std::memcpy(src_dst, sources.data(), sources.size());
    }
};

class FilteredCombinedManager {
public:
    FilteredCombinedManager(const std::string& lidar_dev,
                            const std::string& us_dev,
                            int lidar_baud = 115200,
                            int us_baud = 115200)
        : lidar_dev_(lidar_dev)
        , us_dev_(us_dev)
        , lidar_baud_(lidar_baud)
        , us_baud_(us_baud) {
        for (int i = 0; i < 4; i++) last_us_cm_[i].store(-1.0f, std::memory_order_relaxed);
    }

    void start();
    void stop();
    void snapshot(uint8_t* dst) { filtered_.snapshot(dst); }
    void snapshot(uint8_t* dst, uint8_t* src_dst) { filtered_.snapshot(dst, src_dst); }
    uint32_t lidar_pts() const { return lidar_count_.load(); }
    uint32_t us_pts()    const { return us_count_.load(); }
    uint16_t last_lidar_dist_mm(int id) const { return last_lidar_dist_mm_[id].load(std::memory_order_relaxed); }
    int32_t last_lidar_angle_tenths(int id) const { return last_lidar_angle_tenths_[id].load(std::memory_order_relaxed); }
    float last_us_cm(int idx) const { return last_us_cm_[idx].load(std::memory_order_relaxed); }

private:
    std::string lidar_dev_, us_dev_;
    int         lidar_baud_, us_baud_;
    int         lidar_fd_ = -1;
    int         us_fd_    = -1;

    SpscQ<LidarPoint, 512> lidar_q_;
    SpscQ<UsPoint,    64>  us_q_;

    std::atomic<bool> running_{false};
    std::atomic<uint32_t> lidar_count_{0};
    std::atomic<uint32_t> us_count_{0};

    std::array<std::atomic<uint16_t>, 2> last_lidar_dist_mm_{{0xFFFF,0xFFFF}};
    std::array<std::atomic<int32_t>, 2> last_lidar_angle_tenths_{{0,0}};
    std::array<std::atomic<float>, 4> last_us_cm_;

    std::thread thr_lidar_r_;
    std::thread thr_us_r_;
    std::thread thr_updater_;
    std::thread thr_decay_;

    FilteredMap filtered_;

    bool uart_open(const std::string& dev, int baud, int& fd);
    void uart_close(int& fd);
    bool parse_lidar(const uint8_t* b, LidarPoint& o);
    bool parse_us_json(const char* line, UsPoint& o);
    void mark_xy(float wx, float wy, uint8_t weight, uint8_t src);
    void mark_lidar(const LidarPoint& p);
    void mark_us(const UsPoint& p);
    void lidar_reader_loop();
    void us_reader_loop();
    void updater_loop();
    void decay_loop();
};

static bool readbyte(int fd, uint8_t& b) {
    while (true) {
        ssize_t n = ::read(fd, &b, 1);
        if (n == 1) return true;
        if (n == 0) return false;
        if (errno == EINTR) continue;
        return false;
    }
}

bool FilteredCombinedManager::uart_open(const std::string& dev, int baud, int& fd) {
    char cmd[160];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo 2>/dev/null",
             dev.c_str(), baud);
    int rc = system(cmd);
    (void)rc;
    fd = ::open(dev.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        printf("[WARN] Cannot open %s: %s\n", dev.c_str(), strerror(errno));
        return false;
    }
    printf("[UART] %-20s @ %d OK (fd=%d)\n", dev.c_str(), baud, fd);
    return true;
}

void FilteredCombinedManager::uart_close(int& fd) {
    if (fd >= 0) { ::close(fd); fd = -1; }
}

bool FilteredCombinedManager::parse_lidar(const uint8_t* b, LidarPoint& o) {
    if (b[0] != LIDAR_HDR || b[LIDAR_PKT - 1] != LIDAR_FTR) return false;
    uint8_t chk = 0;
    for (int i = 1; i <= LIDAR_PKT - 3; i++) chk ^= b[i];
    if (chk != b[LIDAR_PKT - 2]) return false;

    o.id = b[1];
    if (o.id >= 2) return false;
    o.dist_mm = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
    int16_t a = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    o.angle_deg = a / 10.0f;
    o.ts_ms = (uint32_t)b[6] | ((uint32_t)b[7] << 8) |
              ((uint32_t)b[8] << 16) | ((uint32_t)b[9] << 24);
    return true;
}

bool FilteredCombinedManager::parse_us_json(const char* line, UsPoint& o) {
    for (int i = 0; i < 4; i++) o.dist_cm[i] = -1.0f;
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

void FilteredCombinedManager::mark_xy(float wx, float wy, uint8_t weight, uint8_t src) {
    int gx = GRID_OX + (int)std::lround(wx / CELL_M);
    int gy = GRID_OY + (int)std::lround(wy / CELL_M);
    filtered_.update_cell(gx, gy, weight, src);
}

void FilteredCombinedManager::mark_lidar(const LidarPoint& p) {
    if (p.dist_mm == 0xFFFF || p.dist_mm < LIDAR_MIN_MM || p.dist_mm > LIDAR_MAX_MM) return;
    if (std::fabs(p.angle_deg) > LIDAR_MAX_ANGLE) return;

    const LidarMount& m = LIDAR_MOUNTS[p.id];
    float rad = (m.mount_deg + p.angle_deg) * PI / 180.0f;
    float dist_m = p.dist_mm / 1000.0f;
    float wx = m.ox + dist_m * std::sin(rad);
    float wy = m.oy + dist_m * std::cos(rad);
    mark_xy(wx, wy, 1, (uint8_t)(1 + p.id));
}

void FilteredCombinedManager::mark_us(const UsPoint& p) {
    for (int i = 0; i < 4; i++) {
        float cm = p.dist_cm[i];
        if (cm < 20.0f || cm > 600.0f) continue;
        float rad = US_MOUNTS[i].dir_deg * PI / 180.0f;
        float dist_m = cm / 100.0f;
        float wx = US_MOUNTS[i].ox + dist_m * std::sin(rad);
        float wy = US_MOUNTS[i].oy + dist_m * std::cos(rad);
        mark_xy(wx, wy, 2, (uint8_t)(3 + i));
    }
}

void FilteredCombinedManager::lidar_reader_loop() {
    printf("[LiDAR Reader] started dev=%s\n", lidar_dev_.c_str()); fflush(stdout);
    uint8_t pkt[LIDAR_PKT], b;
    uint32_t raw=0, aa=0, pkts=0, bad=0;

    while (running_.load(std::memory_order_relaxed)) {
        if (!readbyte(lidar_fd_, b)) break;
        raw++;
        if (b != LIDAR_HDR) continue;
        aa++;
        pkt[0] = LIDAR_HDR;
        bool ok = true;
        for (int i = 1; i < LIDAR_PKT; i++) {
            if (!readbyte(lidar_fd_, pkt[i])) { ok = false; break; }
            raw++;
        }
        if (!ok) break;
        if (pkt[LIDAR_PKT - 1] != LIDAR_FTR) { bad++; continue; }
        LidarPoint lp;
        if (parse_lidar(pkt, lp)) {
            pkts++;
            if (lp.id < 2) {
                last_lidar_dist_mm_[lp.id].store(lp.dist_mm, std::memory_order_relaxed);
                last_lidar_angle_tenths_[lp.id].store((int32_t)std::lround(lp.angle_deg * 10.0f),
                                                      std::memory_order_relaxed);
            }
            if (pkts <= 4) {
                printf("[LiDAR id=%d] dist=%umm angle=%.1fdeg\n",
                       lp.id, lp.dist_mm, lp.angle_deg);
                fflush(stdout);
            }
            if (!lidar_q_.push(lp)) {
                LidarPoint drop; lidar_q_.pop(drop);
                lidar_q_.push(lp);
            }
        }
    }
    printf("[LiDAR Reader] stopped raw=%u aa=%u pkts=%u bad=%u\n",
           raw, aa, pkts, bad);
}

void FilteredCombinedManager::us_reader_loop() {
    printf("[US Reader] started dev=%s\n", us_dev_.c_str()); fflush(stdout);
    char line[256]; int lpos = 0;
    uint8_t b; uint32_t pkts=0, errs=0;

    while (running_.load(std::memory_order_relaxed)) {
        if (!readbyte(us_fd_, b)) break;
        if (b == '\n' || b == '\r') {
            if (lpos > 5) {
                line[lpos] = '\0';
                UsPoint up;
                if (parse_us_json(line, up)) {
                    pkts++;
                    for (int i = 0; i < 4; i++) {
                        last_us_cm_[i].store(up.dist_cm[i], std::memory_order_relaxed);
                    }
                    if (pkts <= 3) {
                        printf("[US] %.1f %.1f %.1f %.1f cm\n",
                               up.dist_cm[0], up.dist_cm[1],
                               up.dist_cm[2], up.dist_cm[3]);
                        fflush(stdout);
                    }
                    if (!us_q_.push(up)) {
                        UsPoint drop; us_q_.pop(drop);
                        us_q_.push(up);
                    }
                } else {
                    errs++;
                }
            }
            lpos = 0;
        } else {
            if (lpos < (int)sizeof(line) - 1) line[lpos++] = (char)b;
        }
    }
    printf("[US Reader] stopped pkts=%u errs=%u\n", pkts, errs);
}

void FilteredCombinedManager::updater_loop() {
    printf("[Updater] started\n"); fflush(stdout);
    using namespace std::chrono_literals;
    while (running_.load(std::memory_order_relaxed)) {
        bool work = false;
        LidarPoint lp;
        while (lidar_q_.pop(lp)) {
            mark_lidar(lp);
            lidar_count_.fetch_add(1, std::memory_order_relaxed);
            work = true;
        }
        UsPoint up;
        while (us_q_.pop(up)) {
            mark_us(up);
            us_count_.fetch_add(1, std::memory_order_relaxed);
            work = true;
        }
        if (!work) std::this_thread::sleep_for(200us);
    }
    printf("[Updater] stopped\n");
}

void FilteredCombinedManager::decay_loop() {
    printf("[Decay] started\n"); fflush(stdout);
    while (running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(DECAY_MS));
        filtered_.decay();
    }
    printf("[Decay] stopped\n");
}

void FilteredCombinedManager::start() {
    bool lidar_ok = uart_open(lidar_dev_, lidar_baud_, lidar_fd_);
    bool us_ok    = uart_open(us_dev_,    us_baud_,    us_fd_);
    if (!lidar_ok) printf("[WARN] LiDAR device unavailable\n");
    if (!us_ok)    printf("[WARN] Ultrasonic device unavailable\n");

    running_.store(true);
    thr_lidar_r_ = std::thread(&FilteredCombinedManager::lidar_reader_loop, this);
    thr_us_r_    = std::thread(&FilteredCombinedManager::us_reader_loop,    this);
    thr_updater_ = std::thread(&FilteredCombinedManager::updater_loop,    this);
    thr_decay_   = std::thread(&FilteredCombinedManager::decay_loop,      this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void FilteredCombinedManager::stop() {
    running_.store(false);
    uart_close(lidar_fd_);
    uart_close(us_fd_);
    if (thr_lidar_r_.joinable()) thr_lidar_r_.join();
    if (thr_us_r_.joinable())    thr_us_r_.join();
    if (thr_updater_.joinable()) thr_updater_.join();
    if (thr_decay_.joinable())   thr_decay_.join();
}

static void draw(const uint8_t* grid, const uint8_t* src,
                 uint32_t lidar_pts, uint32_t us_pts, float hz,
                 const std::array<uint16_t,2>& last_dist,
                 const std::array<int32_t,2>& last_angle_tenths,
                 const std::array<float,4>& last_us) {
    constexpr int V = 100;
    constexpr int OFF = (GRID_N - V) / 2;

    std::printf("\033[2J\033[H");
    std::printf("  2xVB22A + 4xUS Filtered Grid  |  lidar=%-6u  us=%-6u  %.0f pts/s\n",
                lidar_pts, us_pts, hz);
    std::printf("  LiDAR #0: %s\n",
                last_dist[0] == 0xFFFF ? "invalid" : "ok");
    if (last_dist[0] != 0xFFFF) {
        std::printf("    dist=%4dmm angle=%.1fdeg\n",
                    last_dist[0], last_angle_tenths[0] / 10.0f);
    }
    std::printf("  LiDAR #1: %s\n",
                last_dist[1] == 0xFFFF ? "invalid" : "ok");
    if (last_dist[1] != 0xFFFF) {
        std::printf("    dist=%4dmm angle=%.1fdeg\n",
                    last_dist[1], last_angle_tenths[1] / 10.0f);
    }
    std::printf("  US cm: [ %5.1f | %5.1f | %5.1f | %5.1f ]\n",
                last_us[0], last_us[1], last_us[2], last_us[3]);
    std::printf("  [X]=xe  [L]=LiDAR  [1..4]=US1..US4  [*]=vat_can manh  [.] = vat_can nhe\n");
    std::printf("  Vung hien thi: %.1fm x %.1fm (o=%.2fm)\n\n",
                V * CELL_M, V * CELL_M, CELL_M);

    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");

    for (int y = OFF + V - 1; y >= OFF; y--) {
        std::printf("  |");
        for (int x = OFF; x < OFF + V; x++) {
            if (x == GRID_OX && y == GRID_OY) {
                std::printf("X");
                continue;
            }
            int idx = y * GRID_N + x;
            uint8_t s = src[idx];
            if (s == 1) std::printf("L");
            else if (s == 2) std::printf("R");
            else if (s >= 3 && s <= 6) std::printf("%d", s - 2);
            else {
                uint8_t v = grid[idx];
                if (v >= THRESH_STRONG) std::printf("*");
                else if (v >= THRESH_FAINT) std::printf(".");
                else std::printf(" ");
            }
        }
        std::printf("|\n");
    }

    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");
    std::fflush(stdout);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_sig);
    std::signal(SIGTERM, on_sig);

    const char* lidar_dev = (argc > 1) ? argv[1] : "/dev/ttyTHS1";
    const char* us_dev    = (argc > 2) ? argv[2] : "/dev/ttyUSB0";

    std::printf("=== LiDAR + Ultrasonic Filtered Occupancy Grid ===\n");
    std::printf("LiDAR  : %s @ 115200\n", lidar_dev);
    std::printf("US     : %s @ 115200\n\n", us_dev);
    std::fflush(stdout);

    FilteredCombinedManager mgr(lidar_dev, us_dev);
    mgr.start();

    uint8_t snap[GRID_N * GRID_N];
    uint8_t src[GRID_N * GRID_N];
    uint32_t last_lidar = 0, last_us = 0;
    auto last_time = std::chrono::steady_clock::now();

    while (!g_quit.load(std::memory_order_relaxed)) {
        mgr.snapshot(snap, src);
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        uint32_t lp = mgr.lidar_pts();
        uint32_t up = mgr.us_pts();
        float hz = dt > 0.0 ? (float)((lp - last_lidar) + (up - last_us)) / dt : 0.0f;
        last_lidar = lp;
        last_us = up;
        last_time = now;

        std::array<uint16_t,2> last_dist = {mgr.last_lidar_dist_mm(0), mgr.last_lidar_dist_mm(1)};
        std::array<int32_t,2> last_angle_tenths = {mgr.last_lidar_angle_tenths(0), mgr.last_lidar_angle_tenths(1)};
        std::array<float,4> last_us_vals = { mgr.last_us_cm(0), mgr.last_us_cm(1), mgr.last_us_cm(2), mgr.last_us_cm(3) };

        draw(snap, src, lp, up, hz, last_dist, last_angle_tenths, last_us_vals);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::printf("\033[2J\033[HStopped...\n");
    mgr.stop();
    std::printf("Total: lidar=%u us=%u\n", mgr.lidar_pts(), mgr.us_pts());
    return 0;
}
