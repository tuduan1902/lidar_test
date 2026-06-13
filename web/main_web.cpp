/* ============================================================
 *  HTTP SERVER — serve JSON data + HTML page tren localhost
 *  Khong dung thu vien ngoai, chi dung POSIX socket
 *  Port: 8080 (chinh o WEB_PORT neu can)
 * ============================================================ */
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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

static constexpr int WEB_PORT = 8080;

/* HTML + JS page — nhung thang vao string literal */
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
  .ok { color:#4f4; } .warn { color:#fa4; }
</style>
</head>
<body>
<div id="wrap">
  <div id="status">Connecting...</div>
  <canvas id="c"></canvas>
  <div id="info">
    <span class="badge" id="l0">L0: --</span>
    <span class="badge" id="l1">L1: --</span>
    <span class="badge" id="u1">US1: --</span>
    <span class="badge" id="u2">US2: --</span>
    <span class="badge" id="u3">US3: --</span>
    <span class="badge" id="u4">US4: --</span>
    <span class="badge" id="fps">-- pts/s</span>
  </div>
</div>
<script>
const GRID = 100;
const CELL = 6; // pixel per cell
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
canvas.width  = GRID * CELL;
canvas.height = GRID * CELL;

// Color map: value 0..255 -> rgba
// 0=black, 80..199=blue faint, 200..255=cyan strong
// source: 1=L left, 2=R right, 3..6=US 1..4
const SRC_COLORS = ['','#00e5ff','#00e5ff','#ffeb3b','#ffeb3b','#ff9800','#ff9800'];

function cellColor(val, src) {
  if (src >= 1 && src <= 6) return SRC_COLORS[src];
  if (val >= 200) return '#00e5ff';
  if (val >= 80)  return '#1a3a4a';
  return '#111';
}

// Pre-allocate ImageData for zero flicker
const imgData = ctx.createImageData(canvas.width, canvas.height);
const px = imgData.data;

function drawGrid(cells, sources, ox, oy) {
  for (let gy = 0; gy < GRID; gy++) {
    for (let gx = 0; gx < GRID; gx++) {
      const idx = gy * GRID + gx;
      const val = cells[idx];
      const src = sources[idx];

      // Screen Y: flip (grid Y up = screen Y down)
      const sy = (GRID - 1 - gy);

      let r, g, b;
      if (gx === ox && gy === oy) { r=255; g=255; b=0; }        // xe = yellow
      else if (src >= 1 && src <= 2) { r=0; g=229; b=255; }     // LiDAR = cyan
      else if (src >= 3 && src <= 6) { r=255; g=180; b=0; }     // US = orange
      else if (val >= 200) { r=0;  g=180; b=255; }              // strong = blue
      else if (val >= 80)  { r=0;  g=60;  b=90; }               // faint
      else                 { r=17; g=17;  b=17; }               // empty

      const px_base = (sy * GRID * CELL + gx) * CELL;
      for (let dy = 0; dy < CELL; dy++) {
        const row = (sy * CELL + dy) * GRID * CELL * 4 + gx * CELL * 4;
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

    drawGrid(d.cells, d.sources, d.ox, d.oy);

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
    document.getElementById('fps').textContent = `${d.hz.toFixed(0)} pts/s`;

  } catch(e) {
    document.getElementById('status').textContent = 'Connection error: ' + e;
  }
}

// Poll 10 FPS, staggered — dam bao khong chong cheo
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
    float               hz        = 0;
    uint16_t            l_mm [2]  = {0xFFFF, 0xFFFF};
    int32_t             l_a10[2]  = {0, 0};
    float               us_cm[4]  = {-1,-1,-1,-1};
};
static WebState g_web;

/* Cap nhat WebState tu main loop */
static void web_update(FilteredCombinedManager& mgr, float hz) {
    std::lock_guard<std::mutex> lk(g_web.mtx);
    mgr.snapshot(g_web.cells, g_web.sources);
    g_web.lidar_pts = mgr.lidar_pts();
    g_web.us_pts    = mgr.us_pts();
    g_web.hz        = hz;
    for (int i=0;i<2;i++){
        g_web.l_mm[i]  = mgr.last_lidar_dist_mm(i);
        g_web.l_a10[i] = mgr.last_lidar_angle_tenths(i);
    }
    for (int i=0;i<4;i++) g_web.us_cm[i] = mgr.last_us_cm(i);
}

/* Xu ly 1 HTTP connection */
static void handle_client(int client_fd) {
    /* Doc request (bo qua noi dung, chi phan biet path) */
    char buf[512] = {};
    recv(client_fd, buf, sizeof(buf)-1, 0);

    bool is_data = (strstr(buf, "GET /data") != nullptr);

    if (is_data) {
        /* Tao JSON response */
        char body[GRID_N * GRID_N * 5 + 512];
        int  blen = 0;

        /* Lock va lay snapshot */
        uint8_t cells  [GRID_N * GRID_N];
        uint8_t sources[GRID_N * GRID_N];
        uint32_t lpts, upts;
        float hz;
        uint16_t lmm[2]; int32_t la10[2]; float ucm[4];
        {
            std::lock_guard<std::mutex> lk(g_web.mtx);
            memcpy(cells,   g_web.cells,   sizeof(cells));
            memcpy(sources, g_web.sources, sizeof(sources));
            lpts = g_web.lidar_pts; upts = g_web.us_pts; hz = g_web.hz;
            for(int i=0;i<2;i++){lmm[i]=g_web.l_mm[i];la10[i]=g_web.l_a10[i];}
            for(int i=0;i<4;i++) ucm[i]=g_web.us_cm[i];
        }

        /* cells array */
        blen += snprintf(body+blen, sizeof(body)-blen,
            "{\"ox\":%d,\"oy\":%d,"
            "\"lidar_pts\":%u,\"us_pts\":%u,\"hz\":%.1f,"
            "\"l0_mm\":%u,\"l0_a10\":%d,"
            "\"l1_mm\":%u,\"l1_a10\":%d,"
            "\"us0\":%.1f,\"us1\":%.1f,\"us2\":%.1f,\"us3\":%.1f,"
            "\"cells\":[",
            GRID_OX, GRID_OY,
            lpts, upts, hz,
            lmm[0], la10[0], lmm[1], la10[1],
            ucm[0], ucm[1], ucm[2], ucm[3]);

        for (int i = 0; i < GRID_N * GRID_N; i++) {
            blen += snprintf(body+blen, sizeof(body)-blen,
                             i ? ",%d" : "%d", cells[i]);
        }
        blen += snprintf(body+blen, sizeof(body)-blen, "],\"sources\":[");
        for (int i = 0; i < GRID_N * GRID_N; i++) {
            blen += snprintf(body+blen, sizeof(body)-blen,
                             i ? ",%d" : "%d", sources[i]);
        }
        blen += snprintf(body+blen, sizeof(body)-blen, "]}");

        char header[256];
        int hlen = snprintf(header, sizeof(header),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n\r\n", blen);
        send(client_fd, header, hlen, 0);
        send(client_fd, body,   blen, 0);

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
        /* Xu ly trong thread rieng de khong block accept */
        std::thread([client]{ handle_client(client); }).detach();
    }
    close(srv);
}

/* ============================================================
 *  MAIN
 * ============================================================ */
int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    const char* lidar_dev = (argc > 1) ? argv[1] : "/dev/ttyTHS1";
    const char* us_dev    = (argc > 2) ? argv[2] : "/dev/ttyUSB0";

    printf("=== LiDAR + Ultrasonic Web Visualizer ===\n");
    printf("LiDAR : %s @ 115200\n", lidar_dev);
    printf("US    : %s @ 115200\n", us_dev);
    printf("Open  : http://<JETSON_IP>:%d\n\n", WEB_PORT);
    fflush(stdout);

    FilteredCombinedManager mgr(lidar_dev, us_dev);
    mgr.start();

    /* Khoi dong HTTP server thread */
    std::thread http_thr(http_server_thread);
    http_thr.detach();

    uint32_t last_lidar = 0, last_us = 0;
    auto     last_time  = std::chrono::steady_clock::now();

    while (!g_quit.load()) {
        auto     now = std::chrono::steady_clock::now();
        double   dt  = std::chrono::duration<double>(now - last_time).count();
        uint32_t lp  = mgr.lidar_pts();
        uint32_t up  = mgr.us_pts();
        float    hz  = dt > 0.0 ? (float)((lp-last_lidar)+(up-last_us))/dt : 0.0f;
        last_lidar = lp; last_us = up; last_time = now;

        web_update(mgr, hz);

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); /* 20Hz update */
    }

    printf("Stopping...\n");
    mgr.stop();
    printf("Total: lidar=%u us=%u\n", mgr.lidar_pts(), mgr.us_pts());
    return 0;
}