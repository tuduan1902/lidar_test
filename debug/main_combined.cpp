/**
 * main_combined.cpp
 *
 * Build:
 *   g++ -std=c++17 -O2 -lpthread grid_manager.cpp main_combined.cpp -o lidar_us
 *
 * Chay (mac dinh):
 *   ./lidar_us
 *
 * Hoac chi dinh device:
 *   ./lidar_us /dev/ttyTHS1 /dev/ttyUSB0
 */
#include "grid_manager.hpp"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>

static std::atomic<bool> g_quit{false};
static void on_sig(int) { g_quit.store(true); }

// ============================================================
//  HIEN THI GRID ASCII
// ============================================================
static void draw(const uint8_t* g,
                 uint32_t lidar_pts, uint32_t us_pts, float hz) {
    constexpr int V   = 50;   // hien thi 50x50 o giua
    constexpr int OFF = (GRID_N - V) / 2;

    std::printf("\033[2J\033[H");

    // Header
    std::printf(
        "  LiDAR+Ultrasonic Grid  |  lidar=%-6u  us=%-6u  %.0f pts/s\n",
        lidar_pts, us_pts, hz);
    std::printf(
        "  [X]=xe  [*]=obstacle(manh)  [.]=obstacle(mo)  [ ]=trong\n");
    std::printf(
        "  LiDAR:truc Y truoc xe   US: 4 huong xung quanh\n\n");

    // Vien tren
    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");

    // Phan tu: Y cao = phia truoc xe (tren man hinh)
    for (int y = OFF + V - 1; y >= OFF; y--) {
        std::printf("  |");
        for (int x = OFF; x < OFF + V; x++) {
            int     idx = y * GRID_N + x;
            uint8_t v   = g[idx];
            if (x == GRID_OX && y == GRID_OY)
                std::printf("X");
            else if (v >= THRESH_STRONG)
                std::printf("*");
            else if (v >= THRESH_FAINT)
                std::printf(".");
            else
                std::printf(" ");
        }
        std::printf("|\n");
    }

    // Vien duoi
    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");

    std::printf("  Vung: %.1fm x %.1fm (o=3cm)  ",
                V * CELL_M, V * CELL_M);

    // Chu thich huong US
    std::printf("US: [S1=truoc S2=sau S3=trai S4=phai]\n");
    std::printf("  Ctrl+C de thoat\n");
    fflush(stdout);
}

// ============================================================
//  MAIN
// ============================================================
int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    const char* lidar_dev = (argc > 1) ? argv[1] : "/dev/ttyTHS1";
    const char* us_dev    = (argc > 2) ? argv[2] : "/dev/ttyUSB0";

    std::printf("=== LiDAR + Ultrasonic Occupancy Grid ===\n");
    std::printf("LiDAR  : %s @ 115200 (binary packet 14 bytes)\n", lidar_dev);
    std::printf("Ultrasonic: %s @ 115200 (JSON text)\n\n", us_dev);
    fflush(stdout);

    GridManager mgr(lidar_dev, us_dev);
    mgr.start();

    std::printf("\033[?25l");   // an cursor

    uint8_t  snap[GRID_N * GRID_N];
    uint32_t last_lidar = 0, last_us = 0;
    auto     last_time  = std::chrono::steady_clock::now();

    while (!g_quit.load()) {
        mgr.snapshot(snap);

        auto     now   = std::chrono::steady_clock::now();
        double   dt    = std::chrono::duration<double>(now - last_time).count();
        uint32_t lp    = mgr.lidar_pts();
        uint32_t up    = mgr.us_pts();
        float    hz    = dt > 0.0
                         ? (float)((lp - last_lidar) + (up - last_us)) / dt
                         : 0;
        last_lidar = lp; last_us = up; last_time = now;

        draw(snap, lp, up, hz);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::printf("\033[?25h\033[2J\033[H");
    std::printf("Dung lai...\n");
    mgr.stop();
    std::printf("Tong: lidar=%u  us=%u\n", mgr.lidar_pts(), mgr.us_pts());
    return 0;
}