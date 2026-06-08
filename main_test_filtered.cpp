/**
 * main_test_filtered.cpp
 * Visualize the XY occupancy grid from encoder-enabled LiDAR data.
 * This version keeps the original source untouched and shows a larger
 * centered view with the same grid data from grid_xy.cpp.
 *
 * Build:
 *   g++ -std=c++17 -O2 -lpthread grid_xy.cpp main_test_filtered.cpp -o lidar_test_xy_filtered
 * Run:
 *   ./lidar_test_xy_filtered /dev/ttyTHS1 115200
 */

#include "grid_xy.hpp"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <array>

static std::atomic<bool> g_quit{false};
static void on_sig(int) { g_quit.store(true); }

static void draw(const uint8_t* g, uint32_t total, uint16_t last_mm,
                 int last_angle_tenths, float hz) {
    constexpr int Vx  = 80;
    constexpr int Vy  = 40;
    constexpr int offx = (GRID_N - Vx) / 2;
    constexpr int offy = (GRID_N - Vy) / 2;

    std::printf("\033[2J\033[H");
    if (last_mm == 0xFFFF)
        std::printf("  Khoang cach: -- invalid --   pts=%-6u  %.0f/s\n", total, hz);
    else
        std::printf("  Khoang cach: %4d mm (%.2fm)  goc=%5.1f deg  pts=%-6u  %.0f/s\n",
                    last_mm, last_mm / 1000.0f, last_angle_tenths / 10.0f,
                    total, hz);

    std::printf("  [X]=xe  [*]=vat_can manh  [.] = vat_can nhe  [ ]=trong\n");
    std::printf("  +");
    for (int x = 0; x < Vx; x++) std::printf("-");
    std::printf("+\n");

    for (int y = offy + Vy - 1; y >= offy; y--) {
        std::printf("  |");
        for (int x = offx; x < offx + Vx; x++) {
            if (x == GRID_OX && y == GRID_OY) {
                std::printf("X");
            } else {
                uint8_t v = g[y * GRID_N + x];
                if (v >= THRESH_STRONG) std::printf("*");
                else if (v >= THRESH_FAINT) std::printf(".");
                else std::printf(" ");
            }
        }
        std::printf("|\n");
    }

    std::printf("  +");
    for (int x = 0; x < Vx; x++) std::printf("-");
    std::printf("+\n");
    std::printf("\n  Ctrl+C de thoat\n");
    std::fflush(stdout);
}

struct FilteredMap {
    static constexpr uint8_t COUNT_LIGHT   = 2;
    static constexpr uint8_t COUNT_STRONG  = 6;
    static constexpr uint8_t VALUE_LIGHT   = THRESH_FAINT;
    static constexpr uint8_t VALUE_STRONG  = THRESH_STRONG;
    static constexpr uint8_t VALUE_EMPTY   = FREE;

    std::mutex mutex;
    std::array<uint8_t, GRID_N * GRID_N> cells{};
    std::array<uint8_t, GRID_N * GRID_N> hits{};
    int prev_gx = -1;
    int prev_gy = -1;
    int repeat_count = 0;

    FilteredMap() {
        cells.fill(VALUE_EMPTY);
        hits.fill(0);
    }

    bool valid_cell(int gx, int gy) const {
        return gx >= 0 && gx < GRID_N && gy >= 0 && gy < GRID_N;
    }

    void update_point(float wx, float wy, uint16_t dist_mm, float angle_deg) {
        if (dist_mm == 0xFFFF) return;
        if (dist_mm < 30 || dist_mm > 5000) return;  // lọc các giá trị không hợp lệ
        if (std::fabs(angle_deg) > 110.0f) return;   // loại điểm ngoại vi quá lớn

        int gx = GRID_OX + (int)std::lround(wx / CELL_M);
        int gy = GRID_OY + (int)std::lround(wy / CELL_M);
        if (!valid_cell(gx, gy)) return;

        const int idx = gy * GRID_N + gx;
        std::lock_guard<std::mutex> lock(mutex);

        if (gx == prev_gx && gy == prev_gy) {
            repeat_count++;
        } else {
            repeat_count = 1;
            prev_gx = gx;
            prev_gy = gy;
        }

        uint8_t add = (repeat_count >= 2 ? 2 : 1);
        uint8_t new_hits = hits[idx] + add;
        hits[idx] = (new_hits > 255 ? 255 : new_hits);

        if (hits[idx] >= COUNT_STRONG) {
            cells[idx] = VALUE_STRONG;
        } else if (hits[idx] >= COUNT_LIGHT) {
            cells[idx] = VALUE_LIGHT;
        } else {
            cells[idx] = VALUE_EMPTY;
        }
    }

    void decay() {
        std::lock_guard<std::mutex> lock(mutex);
        for (int i = 0; i < GRID_N * GRID_N; i++) {
            if (hits[i] == 0) {
                cells[i] = VALUE_EMPTY;
                continue;
            }
            hits[i] = (hits[i] > 0 ? hits[i] - 1 : 0);
            if (hits[i] >= COUNT_STRONG) {
                cells[i] = VALUE_STRONG;
            } else if (hits[i] >= COUNT_LIGHT) {
                cells[i] = VALUE_LIGHT;
            } else {
                cells[i] = VALUE_EMPTY;
            }
        }
    }

    void snapshot(uint8_t* dst) {
        std::lock_guard<std::mutex> lock(mutex);
        std::memcpy(dst, cells.data(), cells.size());
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    const char* dev  = argc > 1 ? argv[1] : "/dev/ttyTHS1";
    int         baud = argc > 2 ? std::atoi(argv[2]) : 115200;

    std::printf("=== LiDAR XY Filtered Test ===\n");
    std::printf("Device: %s @ %d baud\n\n", dev, baud);
    std::fflush(stdout);

    GridXY mgr(dev, baud);
    FilteredMap filtered;

    uint16_t              last_dist = 0xFFFF;
    std::atomic<uint16_t> last_dist_atomic{0xFFFF};
    std::atomic<int32_t>  last_angle_tenths{0};

    mgr.on_point([&](const ScanPoint& sp, float wx, float wy) {
        filtered.update_point(wx, wy, sp.dist_mm, sp.angle_deg);
        if (sp.dist_mm != 0xFFFF) {
            last_dist_atomic.store(sp.dist_mm, std::memory_order_relaxed);
            last_angle_tenths.store((int32_t)std::lround(sp.angle_deg * 10.0f),
                                    std::memory_order_relaxed);
        }
    });

    mgr.start();
    std::printf("Threads started, waiting for data...\n");
    std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    uint8_t  snap[GRID_N * GRID_N];
    uint32_t last_cnt  = 0;
    auto     last_time = std::chrono::steady_clock::now();

    while (!g_quit.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        filtered.decay();
        filtered.snapshot(snap);
        last_dist = last_dist_atomic.load(std::memory_order_relaxed);

        auto     now = std::chrono::steady_clock::now();
        double   dt  = std::chrono::duration<double>(now - last_time).count();
        uint32_t cnt = mgr.count();
        float    hz  = dt > 0.0 ? (float)(cnt - last_cnt) / dt : 0.0f;
        last_cnt  = cnt;
        last_time = now;

        draw(snap, cnt, last_dist,
             (int)last_angle_tenths.load(std::memory_order_relaxed),
             hz);
    }

    std::printf("\033[2J\033[2H");
    std::printf("Stopped...\n");
    mgr.stop();
    std::printf("Total points processed: %u\n", mgr.count());
    return 0;
}
