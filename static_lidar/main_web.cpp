/**
 * main_combined2_filtered.cpp
 * Separate filtered variant for 2x VB22A LiDAR + 4x ultrasonic + 5x rear VB22A.
 *
 * Build:
 * g++ -std=c++17 -O2 -lpthread \
 *     ./web/main_web.cpp ./web/road_scanner.cpp \
 *     ./web/rear_scanner.cpp ./web/front_scanner.cpp \
 *     -o lidar_us_lidar
 *
 * Run:
 * ./lidar_us_lidar /dev/ttyTHS1 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2 /dev/ttyUSB3
 *   arg1 ttyTHS1  : LiDAR ngang 2 tia (ne vat can)
 *   arg2 ttyUSB0  : LiDAR chui quet o ga
 *   arg3 ttyUSB1  : Ultrasonic 4 cam bien
 *   arg4 ttyUSB2  : 5 LiDAR VB22A duoi xe (STM32 rear)
 *   arg5 ttyUSB3  : 4 LiDAR VB22A dau xe  (STM32 front)
 */

#include "grid_manager2.hpp"
#include "road_scanner.hpp"
#include "rear_scanner.hpp"
#include "front_scanner.hpp"
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
#include <utility>
#include <array>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
                                   
static std::atomic<bool> g_quit{false};
static void on_sig(int) { g_quit.store(true); }

/* ============================================================
 * RoadScanner event state cho web (LiDAR chui)
 * ------------------------------------------------------------
 * Truoc day chi co 1 bien g_lidar_chui_y atomic<float>, chi duoc
 * on_obstacle ghi -> badge va cham mau tren canvas (do on_pothole
 * khong gioi han wy, on_obstacle co gioi han 0.5<wy<1.75) co the
 * lech pha nhau, va gia tri cu khong bao gio bi xoa nen badge hien
 * thi "ma" sau khi cham da decay khoi map.
 *
 * Sua: dung 1 struct co timestamp, ca pothole va obstacle deu ghi
 * vao day (cung dieu kien loc wy), web se hien "--" neu su kien da
 * cu hon ROAD_EVENT_STALE_MS.
 * ============================================================ */
static constexpr int ROAD_EVENT_STALE_MS = 500;

enum class RoadEventType { NONE = 0, POTHOLE = 1, OBSTACLE = 2 };

struct RoadEventState {
    std::mutex mtx;
    float      y    = 0.0f;
    RoadEventType type = RoadEventType::NONE;
    std::chrono::steady_clock::time_point ts = std::chrono::steady_clock::now();

    void update(float wy, RoadEventType t) {
        std::lock_guard<std::mutex> lk(mtx);
        y  = wy;
        type = t;
        ts = std::chrono::steady_clock::now();
    }

    /* Tra ve (y, type) hien tai, hoac (-1, NONE) neu da qua cu */
    std::pair<float, RoadEventType> snapshot() const {
        std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(mtx));
        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - ts).count();
        if (type == RoadEventType::NONE || age_ms > ROAD_EVENT_STALE_MS) {
            return {-1.0f, RoadEventType::NONE};
        }
        return {y, type};
    }
};
static RoadEventState g_road_event;

static constexpr float PI = 3.14159265358979323846f;
static constexpr uint16_t LIDAR_MIN_MM = 10;
static constexpr uint16_t LIDAR_MAX_MM = 20000;
static constexpr float  LIDAR_MAX_ANGLE = 110.0f;

struct FilteredMap {
    static constexpr uint8_t HIT_LIGHT  = 2;
    static constexpr uint8_t HIT_STRONG = 6;
    static constexpr uint8_t VAL_LIGHT  = THRESH_FAINT;
    static constexpr uint8_t VAL_STRONG = THRESH_STRONG;
    static constexpr uint8_t VAL_EMPTY  = FREE;

    // THÊM MỚI CHO ROAD SCANNER

    static constexpr uint8_t VAL_ROAD_FLAT = 128; // Mặt đường phẳng (Gray)
    static constexpr uint8_t VAL_POTHOLE   = 150; // Ổ gà sụt lún (Purple/Red)
    static constexpr uint8_t VAL_ROAD_OBS  = 250; // Vật cản nổi (từ RoadScanner)

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
        uint8_t new_hits = hits[idx] + (add*3);

        if (new_hits > 6 || new_hits < hits[idx]) new_hits = 6;

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
            if (hits[i] <= 3) hits[i] = 0;
            else hits[i] -= 3;

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
    void mark_xy(float wx, float wy, uint8_t weight, uint8_t src);
    void snapshot(uint8_t* dst) { filtered_.snapshot(dst); }
    void snapshot(uint8_t* dst, uint8_t* src_dst) { filtered_.snapshot(dst, src_dst); }
    uint32_t lidar_pts() const { return lidar_count_.load(); }
    uint32_t us_pts()    const { return us_count_.load(); }
    uint16_t last_lidar_dist_mm(int id) const { return last_lidar_dist_mm_[id].load(std::memory_order_relaxed); }
    int32_t last_lidar_angle_tenths(int id) const { return last_lidar_angle_tenths_[id].load(std::memory_order_relaxed); }
    float last_us_cm(int idx) const { return last_us_cm_[idx].load(std::memory_order_relaxed); }
    using LidarBypassCb = std::function<void(float angle, uint16_t dist, uint32_t ts)>;
    void set_lidar_bypass(int target_id, LidarBypassCb cb) {
        bypass_id_ = target_id;
        bypass_cb_ = cb;
    }
private:
    std::string lidar_dev_, us_dev_;
    int         lidar_baud_, us_baud_;
    int         lidar_fd_ = -1;
    int         us_fd_    = -1;
    int           bypass_id_ = -1;       // ID muốn chặn (ví dụ = 1)
    LidarBypassCb bypass_cb_ = nullptr;  // Hàm callback để bắn dữ liệu đi

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
            // Chan du lieu tu 1 lidar 
            if (lp.id == bypass_id_ && bypass_cb_ != nullptr) {
                bypass_cb_(lp.angle_deg, lp.dist_mm, lp.ts_ms);
                continue; // Chặn đứng tại đây, không cho nạp vào map né vật cản chính nữa!
            }

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

static constexpr int WEB_PORT = 8080;

/* HTML + JS page */
static const char HTML_PAGE[] = R"HTML(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>LiDAR Grid</title>
<style>
  body { margin:0; background:#111; color:#eee; font-family:monospace; }
  #wrap { display:flex; flex-direction:column; align-items:center; padding:12px; }
  #status { font-size:13px; margin-bottom:8px; color:#8cf; }
  canvas { image-rendering:pixelated; border:1px solid #333; }
  #info { margin-top:8px; font-size:12px; color:#aaa; display:flex; gap:24px; }
  .badge { background:#222; padding:4px 8px; border-radius:4px; }
  .ok { color:#4f4; } .warn { color:#fa4; } .pothole { color:#f4f; }
</style>
</head>
<body>
<div id="wrap">
  <div id="status">Connecting...</div>
  <canvas id="c"></canvas>
  <div id="info">
    <span class="badge" id="l0">L0: --</span>
    <span class="badge" id="l1">L1: --</span>
    <span class="badge" id="lidar-chui"><span id="lidar-chui-label">LiDAR chúi</span>: <span id="lidar-chui-dist">--</span></span>
    <span class="badge" id="u1">US1: --</span>
    <span class="badge" id="u2">US2: --</span>
    <span class="badge" id="u3">US3: --</span>
    <span class="badge" id="u4">US4: --</span>
    <span class="badge" id="r0">R0: --</span>
    <span class="badge" id="r1">R1: --</span>
    <span class="badge" id="r2">R2: --</span>
    <span class="badge" id="r3">R3: --</span>
    <span class="badge" id="r4">R4: --</span>
    <span class="badge" id="f0">F0: --</span>
    <span class="badge" id="f1">F1: --</span>
    <span class="badge" id="f2">F2: --</span>
    <span class="badge" id="f3">F3: --</span>
    <span class="badge" id="fps">-- pts/s</span>
  </div>
</div>
<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

let imgData = null;
let px = null;

function drawGrid(cells, sources, ox, oy, grid_n) {
  const CELL = Math.floor(600 / grid_n); 
  const currentWidth = grid_n * CELL;

  if (!imgData || canvas.width !== currentWidth) {
    canvas.width  = currentWidth;
    canvas.height = currentWidth;
    imgData = ctx.createImageData(canvas.width, canvas.height);
    px = imgData.data;
  }

  for (let gy = 0; gy < grid_n; gy++) {
    for (let gx = 0; gx < grid_n; gx++) {
      const idx = gy * grid_n + gx;
      const val = cells[idx];
      const src = sources[idx];

      const sy = (grid_n - 1 - gy);

      let r, g, b;
      if (gx === ox && gy === oy) { r=255; g=255; b=0; }
      else if (src === 10) { r=255; g=0; b=255; }               /* o ga: magenta */
      else if (src === 11) { r=255; g=50; b=50; }               /* vat can: do */
      else if (src >= 1 && src <= 2) { r=0; g=229; b=255; }    /* LiDAR ngang: cyan */
      else if (src >= 3 && src <= 6) { r=255; g=180; b=0; }    /* ultrasonic: cam */
      else if (src === 20) { r=0; g=255; b=120; }               /* Rear L0 trung tam: xanh la */
      else if (src === 21) { r=80; g=255; b=80; }               /* Rear L1 -22.5: xanh la nhat */
      else if (src === 22) { r=160; g=255; b=40; }              /* Rear L2 -45: xanh vang */
      else if (src === 23) { r=80; g=255; b=80; }               /* Rear L3 +22.5: xanh la nhat */
      else if (src === 24) { r=160; g=255; b=40; }              /* Rear L4 +45: xanh vang */
      else if (src === 30) { r=255; g=255; b=100; }             /* Front F0 ngoai trai: vang sang */
      else if (src === 31) { r=255; g=220; b=80; }              /* Front F1 trong trai: vang cam */
      else if (src === 32) { r=255; g=220; b=80; }              /* Front F2 trong phai: vang cam */
      else if (src === 33) { r=255; g=255; b=100; }             /* Front F3 ngoai phai: vang sang */
      else if (val >= 200) { r=0;  g=180; b=255; }              /* strong: xanh duong */
      else if (val >= 80)  { r=0;  g=60;  b=90; }               /* light: xanh duong dam */
      else                 { r=17; g=17;  b=17; }               /* nen */

      for (let dy = 0; dy < CELL; dy++) {
        const row = (sy * CELL + dy) * currentWidth * 4 + gx * CELL * 4;
        for (let dx = 0; dx < CELL; dx++) {
          const p = row + dx * 4;
          px[p]   = r;
          px[p+1] = g;
          px[p+2] = b;
          px[p+3] = 255; 
        }
      }
    }
  }
  ctx.putImageData(imgData, 0, 0);
}

async function fetchData() {
  try {
    const res = await fetch('/data');
    const d = await res.json();

    drawGrid(d.cells, d.sources, d.ox, d.oy, d.grid_n);

    document.getElementById('status').textContent =
      `lidar=${d.lidar_pts}  us=${d.us_pts}  ${d.hz.toFixed(0)} pts/s`;

    const fmt = (mm) => mm === 65535 ? '--' : (mm/1000).toFixed(2)+'m';
    const fmtA = (a10) => (a10/10).toFixed(1)+'°';
    document.getElementById('l0').textContent =
      `L0: ${fmt(d.l0_mm)} ${fmtA(d.l0_a10)}`;
    document.getElementById('l0').className =
      'badge ' + (d.l0_mm === 65535 ? 'warn' : 'ok');
    document.getElementById('l1').textContent =
      `L1: ${fmt(d.l1_mm)} ${fmtA(d.l1_a10)}`;
    document.getElementById('l1').className =
      'badge ' + (d.l1_mm === 65535 ? 'warn' : 'ok');

    const uf = (v) => v < 0 ? '--' : v.toFixed(1)+'cm';
    ['u1','u2','u3','u4'].forEach((id,i) => {
      document.getElementById(id).textContent = `US${i+1}: ${uf(d['us'+i])}`;
    });

    // Cập nhật khoảng cách LiDAR chúi (su kien o ga/vat can gan nhat, "--" neu da cu/khong co)
    const lcType  = d.lidar_chui_type || 0;
    const lcLabel = lcType === 1 ? 'Ổ gà' : (lcType === 2 ? 'Vật cản' : 'LiDAR chúi');
    const lcDist  = (d.lidar_chui_y >= 0) ? d.lidar_chui_y.toFixed(2) + " m" : "--";
    document.getElementById('lidar-chui-label').textContent = lcLabel;
    document.getElementById('lidar-chui-dist').innerText = lcDist;
    document.getElementById('lidar-chui').className =
      'badge ' + (lcType === 1 ? 'pothole' : (lcType === 2 ? 'warn' : ''));

    // Cap nhat 5 badge LiDAR duoi xe
    const REAR_LABELS = ['R0(0°)','R1(-22°)','R2(-45°)','R3(+22°)','R4(+45°)'];
    for (let i = 0; i < 5; i++) {
      const v = d['rear' + i];
      const el = document.getElementById('r' + i);
      if (v === undefined || v < 0) {
        el.textContent = REAR_LABELS[i] + ': --';
        el.className = 'badge';
      } else {
        el.textContent = REAR_LABELS[i] + ': ' + v.toFixed(2) + 'm';
        el.className = 'badge ' + (v < 1.0 ? 'warn' : 'ok');
      }
    }

    // Cap nhat 4 badge LiDAR dau xe
    const FRONT_LABELS = ['F0(nT)','F1(tT)','F2(tP)','F3(nP)'];
    for (let i = 0; i < 4; i++) {
      const v = d['front' + i];
      const el = document.getElementById('f' + i);
      if (v === undefined || v < 0) {
        el.textContent = FRONT_LABELS[i] + ': --';
        el.className = 'badge';
      } else {
        el.textContent = FRONT_LABELS[i] + ': ' + v.toFixed(2) + 'm';
        el.className = 'badge ' + (v < 1.5 ? 'warn' : 'ok');
      }
    }

    document.getElementById('fps').textContent = `${d.hz.toFixed(0)} pts/s`;

  } catch(e) {
    document.getElementById('status').textContent = 'Connection error: ' + e;
  }
}

let running = false;
async function loop() {
  if (!running) {
    running = true;
    await fetchData();
    running = false;
  }
  setTimeout(loop, 100);
}
loop();
</script>
</body>
</html>
)HTML";

/* Trang thai toan cuc de HTTP thread doc */
struct WebState {
    std::mutex          mtx;
    uint8_t             cells  [GRID_N * GRID_N];
    uint8_t             sources[GRID_N * GRID_N];
    uint32_t            lidar_pts = 0;
    uint32_t            us_pts    = 0;
    float               lidar_chui_y    = -1.0f; /* -1 = chua co su kien / da cu */
    int                 lidar_chui_type = 0;      /* 0=none, 1=pothole, 2=obstacle */
    float               hz        = 0;
    uint16_t            l_mm [2]  = {0xFFFF, 0xFFFF};
    int32_t             l_a10[2]  = {0, 0};
    float               us_cm[4]  = {-1,-1,-1,-1};

    /* 5 LiDAR sau duoi xe */
    uint32_t            rear_pts  = 0;
    float               rear_dist[REAR_N_LIDAR]  = {-1,-1,-1,-1,-1};
    uint32_t            front_pts = 0;
    float               front_dist[FRONT_N_LIDAR] = {-1,-1,-1,-1};
};
static WebState g_web;

/* Cap nhat WebState tu main loop */
static void web_update(FilteredCombinedManager& mgr, RoadScanner& rs,
                       RearScanner& rear, FrontScanner& front, float hz) {
    (void)rs;
    std::lock_guard<std::mutex> lk(g_web.mtx);
    mgr.snapshot(g_web.cells, g_web.sources);

    g_web.lidar_pts = mgr.lidar_pts();
    { auto [y,type] = g_road_event.snapshot(); g_web.lidar_chui_y=(y); g_web.lidar_chui_type=(int)type; }
    g_web.us_pts    = mgr.us_pts();
    g_web.hz        = hz;
    for (int i=0;i<2;i++) { g_web.l_mm[i]=mgr.last_lidar_dist_mm(i); g_web.l_a10[i]=mgr.last_lidar_angle_tenths(i); }
    for (int i=0;i<4;i++) g_web.us_cm[i] = mgr.last_us_cm(i);
    g_web.rear_pts = rear.pts_total();
    for (int i=0;i<REAR_N_LIDAR;i++)  g_web.rear_dist[i]  = rear.last_dist_m(i);
    g_web.front_pts = front.pts_total();
    for (int i=0;i<FRONT_N_LIDAR;i++) g_web.front_dist[i] = front.last_dist_m(i);
}

/* Xu ly 1 HTTP connection */
static void handle_client(int client_fd) {
    /* Doc request */
    char buf[512] = {};
    recv(client_fd, buf, sizeof(buf)-1, 0);

    bool is_data = (strstr(buf, "GET /data") != nullptr);

    if (is_data) {
        /* Lock va lay snapshot */
        uint8_t cells  [GRID_N * GRID_N];
        uint8_t sources[GRID_N * GRID_N];
        uint32_t lpts, upts;
        float hz;
        float lc_y;  // gia tri Y cua su kien o ga/vat can gan nhat (-1 = khong co/da cu)
        int   lc_type; // 0=none, 1=pothole, 2=obstacle
        uint16_t lmm[2]; int32_t la10[2]; float ucm[4];
        uint32_t rear_pts; float rear_dist[REAR_N_LIDAR];
        uint32_t front_pts; float front_dist[FRONT_N_LIDAR];
        {
            std::lock_guard<std::mutex> lk(g_web.mtx);
            memcpy(cells,g_web.cells,sizeof(cells)); memcpy(sources,g_web.sources,sizeof(sources));
            lc_y=g_web.lidar_chui_y; lc_type=g_web.lidar_chui_type;
            lpts=g_web.lidar_pts; upts=g_web.us_pts; hz=g_web.hz;
            for(int i=0;i<2;i++){lmm[i]=g_web.l_mm[i];la10[i]=g_web.l_a10[i];}
            for(int i=0;i<4;i++) ucm[i]=g_web.us_cm[i];
            rear_pts=g_web.rear_pts;
            for(int i=0;i<REAR_N_LIDAR;i++)  rear_dist[i]=g_web.rear_dist[i];
            front_pts=g_web.front_pts;
            for(int i=0;i<FRONT_N_LIDAR;i++) front_dist[i]=g_web.front_dist[i];
        }

        /* Su dung std::string de cap phat bo nho dong an toan */
        std::string body;
        body.reserve(GRID_N * GRID_N * 8 + 2048); 

        // 1. Thêm các thông số meta vào JSON
        body += "{";
        body += "\"grid_n\":" + std::to_string(GRID_N) + ",";
        body += "\"ox\":"     + std::to_string(GRID_OX) + ",";
        body += "\"oy\":"     + std::to_string(GRID_OY) + ",";
        body += "\"lidar_pts\":" + std::to_string(lpts) + ",";
        body += "\"us_pts\":"    + std::to_string(upts) + ",";
        body += "\"hz\":"        + std::to_string(hz) + ",";
        body += "\"l0_mm\":"     + std::to_string(lmm[0]) + ",";
        body += "\"l0_a10\":"    + std::to_string(la10[0]) + ",";
        body += "\"l1_mm\":"     + std::to_string(lmm[1]) + ",";
        body += "\"l1_a10\":"    + std::to_string(la10[1]) + ",";
        body += "\"us0\":" + std::to_string(ucm[0]) + ",";
        body += "\"us1\":" + std::to_string(ucm[1]) + ",";
        body += "\"us2\":" + std::to_string(ucm[2]) + ",";
        body += "\"us3\":" + std::to_string(ucm[3]) + ",";
        body += "\"lidar_chui_y\":" + std::to_string(lc_y) + ",";
        body += "\"lidar_chui_type\":" + std::to_string(lc_type) + ",";
        body += "\"rear_pts\":" + std::to_string(rear_pts) + ",";
        for (int i=0;i<REAR_N_LIDAR;i++)
            body += "\"rear"+std::to_string(i)+"\":"+std::to_string(rear_dist[i])+",";
        body += "\"front_pts\":" + std::to_string(front_pts) + ",";
        for (int i=0;i<FRONT_N_LIDAR;i++)
            body += "\"front"+std::to_string(i)+"\":"+std::to_string(front_dist[i])+",";

        // 2. Nối mảng cells
        body += "\"cells\":[";
        for (int i = 0; i < GRID_N * GRID_N; i++) {
            if (i > 0) body += ",";
            body += std::to_string((int)cells[i]);
        }
        
        // 3. Nối mảng sources
        body += "],\"sources\":[";
        for (int i = 0; i < GRID_N * GRID_N; i++) {
            if (i > 0) body += ",";
            body += std::to_string((int)sources[i]);
        }
        body += "]}";

        char header[256];
        int hlen = snprintf(header, sizeof(header),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %zu\r\n"
            "Connection: close\r\n\r\n", body.size());

        send(client_fd, header, hlen, 0);
        send(client_fd, body.data(), body.size(), 0);

    } else {
        /* Tra ve HTML page */
        int  blen  = (int)strlen(HTML_PAGE);
        char header[256];
        int  hlen  = snprintf(header, sizeof(header),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n\r\n", blen);
        send(client_fd, header, hlen, 0);
        send(client_fd, HTML_PAGE, blen, 0);
    }
    close(client_fd);
}

/* HTTP server thread */
static void http_server_thread() {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(WEB_PORT);
    bind(srv, (struct sockaddr*)&addr, sizeof(addr));
    listen(srv, 8);

    printf("[HTTP] Listening on http://0.0.0.0:%d\n", WEB_PORT);
    fflush(stdout);

    while (!g_quit.load()) {
        struct sockaddr_in cli{};
        socklen_t clen = sizeof(cli);
        int client = accept(srv, (struct sockaddr*)&cli, &clen);
        if (client < 0) continue;
        std::thread([client]{ handle_client(client); }).detach();
    }
    close(srv);
}

/* ============================================================
 * MAIN (Hỗ trợ Đánh chặn LiDAR Đuôi làm LiDAR Chúi quét ổ gà)
 * ============================================================ */
int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    const char* lidar_dev = (argc > 1) ? argv[1] : "/dev/ttyTHS1";
    const char* road_dev  = (argc > 2) ? argv[2] : "/dev/ttyUSB0";
    const char* us_dev    = (argc > 3) ? argv[3] : "/dev/ttyUSB1";
    const char* rear_dev  = (argc > 4) ? argv[4] : "/dev/ttyUSB2";
    const char* front_dev = (argc > 5) ? argv[5] : "/dev/ttyUSB3"; /* STM32 4 LiDAR dau xe */
    

    printf("============================================================\n");
    printf("=== TESTING ROAD SCANNER WITH EXISTING LIDAR L1 (HIJACK) ===\n");
    printf("============================================================\n");
    printf("Lidar Scanner Bus: %s @ 115200 (Chứa L0-L1 né vật cản)\n", road_dev);
    printf("LiDAR Main Bus : %s @ 115200 (Chứa L0-L1 né vật cản)\n", lidar_dev);
    printf("Ultrasonic Bus : %s @ 115200\n", us_dev);
    printf("Rear LiDAR Bus : %s @ 460800 (5 LiDAR VB22A duoi xe, STM32)\n", rear_dev);
    printf("Front LiDAR Bus: %s @ 460800 (4 LiDAR VB22A dau xe, STM32)\n", front_dev);
    printf("Open Web Interface: http://<JETSON_IP>:%d\n", WEB_PORT);
    printf("------------------------------------------------------------\n");
    fflush(stdout);

    // 1. Manager Lidar ngang & US
    FilteredCombinedManager mgr(lidar_dev, us_dev);
    RoadScanner  road_scanner(road_dev,  115200);
    RearScanner  rear_scanner(rear_dev,  460800);
    FrontScanner front_scanner(front_dev, 460800);

    rear_scanner.on_point([&mgr](const RearPoint& pt) {
        mgr.mark_xy(pt.wx, pt.wy, FilteredMap::HIT_STRONG, (uint8_t)(20 + pt.id));
    });

    /* Front LiDAR: src 30-33, mau xanh luc nhat (phia truoc xe)
     * wx = ox_m (vi tri ngang cua sensor, khong doi theo dist vi angle=90)
     * wy = oy_m + dist_m (thang phia truoc)
     * Diem nam dung phia truoc tung sensor, chinh xac. */
    front_scanner.on_point([&mgr](const FrontPoint& pt) {
        mgr.mark_xy(pt.wx, pt.wy, FilteredMap::HIT_STRONG, (uint8_t)(30 + pt.id));
    });

    //Callback xử lý ổ gà
    road_scanner.on_pothole([&mgr](const RoadSample& sample) {
        // Với 1 tia cố định, tọa độ X (ngang) luôn bằng 0 (nằm giữa đầu xe).
        float wx = 0.0f;

        // Tọa độ Y (phía trước) = khoảng cách tia * cos(goc_chui) + khoảng_cách_gắn_lidar.
        float wy = (sample.dist_ema_m * cosf(PITCH_STATIC_RAD)) + LIDAR_OY;

        // Cùng giới hạn vùng quan tâm phía trước xe như on_obstacle, để 2 loại
        // sự kiện đối xứng nhau trên bản đồ/badge.
        if (wy > 0.5f && wy < 1.75f) {
            g_road_event.update(wy, RoadEventType::POTHOLE);
            mgr.mark_xy(wx, wy, FilteredMap::HIT_STRONG, 10);
        }
    });

    // Callback xử lý gờ giảm tốc/vật cản (Khoảng cách đột ngột NGẮN lại)
    road_scanner.on_obstacle([&mgr](const RoadSample& sample) {
        // Biến static để lưu thời gian của lần kích hoạt gần nhất
        static auto last_trigger = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();

        // Tính toán tọa độ Y (X luôn bằng 0 vì tia nằm giữa)
        float wx = 0.0f;
        float wy = (sample.dist_ema_m * cosf(PITCH_STATIC_RAD)) + LIDAR_OY;

        // Đánh dấu lên map (hàm này chạy rất nhẹ, không lo tràn bộ nhớ)
        if (wy > 0.5f && wy < 1.75f) {
            g_road_event.update(wy, RoadEventType::OBSTACLE);
            mgr.mark_xy(wx, wy, FilteredMap::HIT_STRONG, 11);
        } else {
            return;
        }

        // RATE-LIMIT: Chỉ cho phép in log tối đa 5 lần/giây (mỗi 200ms)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_trigger).count() >= 200) {
            printf("[LiDAR Chúi] CẢNH BÁO: Vật cản tại Y = %.2f m\n", wy);
            last_trigger = now; // Cập nhật lại mốc thời gian
        }

    });

    mgr.start();
    road_scanner.start();
    rear_scanner.start();
    front_scanner.start();

    std::thread http_thr(http_server_thread);
    http_thr.detach();

    uint32_t last_lidar = 0, last_us = 0;
    auto     last_time  = std::chrono::steady_clock::now();

    while (!g_quit.load()) {
        auto     now = std::chrono::steady_clock::now();
        double   dt  = std::chrono::duration<double>(now - last_time).count();
        uint32_t lp  = mgr.lidar_pts();
        uint32_t up  = mgr.us_pts();
        
        float    hz  = dt > 0.0 ? (float)((lp - last_lidar) + (up - last_us)) / dt : 0.0f;
        last_lidar = lp; 
        last_us = up; 
        last_time = now;

        web_update(mgr, road_scanner, rear_scanner, front_scanner, hz);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    mgr.stop(); road_scanner.stop(); rear_scanner.stop(); front_scanner.stop();
    printf("Stopped. lidar=%u us=%u rear=%u front=%u\n",
           mgr.lidar_pts(), mgr.us_pts(), rear_scanner.pts_total(), front_scanner.pts_total());
    return 0;
}