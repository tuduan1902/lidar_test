/**
 * main_simple.cpp
 * Build: g++ -std=c++17 -O2 -lpthread grid_simple.cpp main_simple.cpp -o lidar_test && echo "BUILD OK"
 * Chay:  ./lidar_test /dev/ttyTHS1 115200
 * Check UART data:stty -F /dev/ttyTHS1 115200 raw -echo -echoe -echok
 hexdump -C /dev/ttyTHS1
 * 
 *
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

static void draw(const uint8_t* g, uint32_t total, uint16_t last_mm, float hz) {
    constexpr int V   = 40;
    constexpr int off = (GRID_N - V) / 2;

    /* Xoa man hinh hoan toan moi frame - tranh loi cuon tren Jetson */
    std::printf("\033[2J\033[H");

    if (last_mm == 0xFFFF)
        std::printf("  Khoang cach: -- invalid --   pts=%-6u  %.0f/s\n", total, hz);
    else
        std::printf("  Khoang cach: %4d mm (%.2fm)  pts=%-6u  %.0f/s\n",
                    last_mm, last_mm/1000.0f, total, hz);

    std::printf("  [X]=xe  [*]=vat_can  [ ]=trong\n\n");

    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");

    for (int y = off + V - 1; y >= off; y--) {
        std::printf("  |");
        for (int x = off; x < off + V; x++) {
            if (x == GRID_OX && y == GRID_OY)      std::printf("X");
            else if (g[y * GRID_N + x] == OBSTACLE) std::printf("*");
            else                                     std::printf(" ");
        }
        std::printf("|\n");
    }

    std::printf("  +");
    for (int x = 0; x < V; x++) std::printf("-");
    std::printf("+\n");

    /* Thanh do khoang cach */
    if (last_mm != 0xFFFF && last_mm <= 1500) {
        int bar = last_mm / 30;
        if (bar > 38) bar = 38;
        std::printf("  |");
        for (int i = 0; i < bar; i++)  std::printf("=");
        for (int i = bar; i < 38; i++) std::printf(" ");
        std::printf("| %dmm\n", last_mm);
    }

    std::printf("\n  Ctrl+C de thoat\n");
    fflush(stdout);
}

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sig);
    std::signal(SIGTERM, on_sig);

    const char* dev  = argc > 1 ? argv[1] : "/dev/ttyTHS1";
    int         baud = argc > 2 ? std::atoi(argv[2]) : 115200;

    std::printf("=== LiDAR Test ===\n");
    std::printf("Device: %s @ %d baud\n\n", dev, baud);
    fflush(stdout);

    GridSimple mgr(dev, baud);

    uint16_t              last_dist = 0xFFFF;
    std::atomic<uint16_t> last_dist_atomic{0xFFFF};

    mgr.on_point([&](uint16_t mm, int gy) {
        last_dist_atomic.store(mm);
    });

    /* Start threads - ham nay da co delay 200ms ben trong */
    mgr.start();

    std::printf("Threads started, cho data...\n");
    fflush(stdout);

    /* Cho them 1s de xem debug print tu reader_loop */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    uint8_t  snap[GRID_N * GRID_N];
    uint32_t last_cnt  = 0;
    auto     last_time = std::chrono::steady_clock::now();

    while (!g_quit.load()) {
        mgr.snapshot(snap);
        last_dist = last_dist_atomic.load();

        auto     now = std::chrono::steady_clock::now();
        double   dt  = std::chrono::duration<double>(now - last_time).count();
        uint32_t cnt = mgr.count();
        float    hz  = dt > 0.0 ? (float)(cnt - last_cnt) / dt : 0;
        last_cnt  = cnt;
        last_time = now;

        draw(snap, cnt, last_dist, hz);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::printf("\033[2J\033[H");
    std::printf("Dung lai...\n");
    mgr.stop();
    std::printf("Tong diem: %u\n", mgr.count());
    return 0;
}