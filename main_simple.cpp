/**
 * main_simple.cpp
 *
 * Build:
 *   g++ -std=c++17 -O2 -lpthread grid_simple.cpp main_simple.cpp -o lidar_test
 *
 * Chay:
// Chay:
//   ./lidar_test /dev/ttyACM0 460800   <- USB CDC (khuyen nghi)
//   ./lidar_test /dev/ttyAMA0 460800   <- UART hardware
 *   ./lidar_test /dev/ttyAMA0        <- UART hardware
 */

#include "grid_simple.hpp"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

static std::atomic<bool> g_quit{false};
static void on_sig(int) { g_quit.store(true); }

/* Hien thi grid 50x50 o giua man hinh */
static void draw(const uint8_t* g, uint32_t total,
                 uint16_t last_mm, float hz)
{
    constexpr int V   = 50;
    constexpr int off = (GRID_N - V) / 2;

    std::printf("\033[H");   /* cursor ve dau */

    /* Dong trang thai */
    if (last_mm == 0xFFFF)
        std::printf("  Khoang cach: -- invalid --     pts=%-6u  %.0f/s   \n",
                    total, hz);
    else
        std::printf("  Khoang cach: %4d mm  (%5.2f m)   pts=%-6u  %.0f/s   \n",
                    last_mm, last_mm/1000.0f, total, hz);

    std::printf("  [X] = xe/cam bien   [*] = vat can   [ ] = trong\n\n");

    /* Vien tren */
    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");

    /* Y tu cao xuong thap: hang tren man hinh = phia truoc */
    for (int y = off + V - 1; y >= off; y--) {
        std::printf("  |");
        for (int x = off; x < off + V; x++) {
            if (x == GRID_OX && y == GRID_OY)
                std::printf("X");
            else if (g[y * GRID_N + x] == OBSTACLE)
                std::printf("*");
            else
                std::printf(" ");
        }
        std::printf("|\n");
    }

    /* Vien duoi */
    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");

    std::printf("  Vung hien thi: %.1fm x %.1fm  (o = 3cm)  Ctrl+C thoat\n",
                V * CELL_M, V * CELL_M);

    /* Thanh khoang cach truc quan */
    if (last_mm != 0xFFFF && last_mm < 1500) {
        int bar = (int)(last_mm / 30);   /* moi o = 3cm */
        if (bar > 48) bar = 48;
        std::printf("  |");
        for (int i = 0; i < bar; i++)  std::printf("=");
        for (int i = bar; i < 48; i++) std::printf(" ");
        std::printf("| %d mm\n", last_mm);
    }
}

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    const char* dev  = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int         baud = argc > 2 ? std::atoi(argv[2]) : 460800;

    std::printf("LiDAR simple test\n");
    std::printf("Device: %s @ %d\n\n", dev, baud);

    GridSimple mgr(dev, baud);

    /* In moi diem nhan duoc ra console (debug) */
    uint16_t last_dist = 0xFFFF;
    mgr.on_point([&](uint16_t mm, int gy) {
        last_dist = mm;
        /* In them 1 dong don gian moi 20 diem de debug terminal */
        static int dbg = 0;
        if (++dbg % 20 == 0)
            std::printf("\r  [DBG] dist=%5dmm  grid_y=%d      \n", mm, gy);
    });

    mgr.start();

    std::printf("\033[2J");    /* xoa man hinh */
    std::printf("\033[?25l");  /* an cursor */

    uint8_t  snap[GRID_N * GRID_N];
    uint32_t last_cnt  = 0;
    auto     last_time = std::chrono::steady_clock::now();

    while (!g_quit.load()) {
        mgr.snapshot(snap);

        auto   now = std::chrono::steady_clock::now();
        double dt  = std::chrono::duration<double>(now - last_time).count();
        uint32_t cnt = mgr.count();
        float hz  = dt > 0 ? (cnt - last_cnt) / dt : 0;
        last_cnt  = cnt; last_time = now;

        draw(snap, cnt, last_dist, hz);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::printf("\033[?25h\033[2J\033[H");
    std::printf("Dung lai...\n");
    mgr.stop();
    std::printf("Tong diem: %u\n", mgr.count());
    return 0;
}