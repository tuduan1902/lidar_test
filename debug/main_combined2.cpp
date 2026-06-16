/**
 * main_combined2.cpp
 * Build: g++ -std=c++17 -O2 -lpthread grid_manager2.cpp main_combined2.cpp -o lidar_us
 * Chay:  ./lidar_us /dev/ttyTHS1 /dev/ttyUSB0
 */
#include "grid_manager2.hpp"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

static std::atomic<bool> g_quit{false};
static void on_sig(int){ g_quit.store(true); }

static void draw(const GridManager& mgr, const uint8_t* g,
                 uint32_t lpts, uint32_t upts, float hz) {
    constexpr int V   = 50;
    constexpr int OFF = (GRID_N-V)/2;

    std::printf("\033[2J\033[H");

    /* Status bar */
    std::printf("  2xLiDAR + 4xUltrasonic Grid  |  L=%-5u  US=%-5u  %.0f/s\n",
                lpts, upts, hz);
    std::printf("  [*]=obstacle  [.]=fading  [X]=xe\n");

    /* LiDAR status */
    for(int i=0;i<2;i++){
        const char* name = (i==0)?"LEFT ":"RIGHT";
        if(mgr.last_dist[i]==0xFFFF)
            std::printf("  LiDAR %s: -- invalid --  angle=%.1f\n",
                        name, mgr.last_angle[i]);
        else
            std::printf("  LiDAR %s: %4dmm (%.2fm)  angle=%.1f\n",
                        name, mgr.last_dist[i],
                        mgr.last_dist[i]/1000.0f,
                        mgr.last_angle[i]);
    }

    /* Ultrasonic status */
    std::printf("  US: F=%.0fcm B=%.0fcm L=%.0fcm R=%.0fcm\n",
                mgr.last_us[0], mgr.last_us[1],
                mgr.last_us[2], mgr.last_us[3]);
    std::printf("\n");

    /* Grid */
    std::printf("  +");
    for(int x=0;x<V;x++) std::printf("-");
    std::printf("+  <- TRUOC XE\n");

    for(int y=OFF+V-1; y>=OFF; y--){
        std::printf("  |");
        for(int x=OFF;x<OFF+V;x++){
            uint8_t v=g[y*GRID_N+x];
            if(x==GRID_OX&&y==GRID_OY)      std::printf("X");
            else if(v>=THRESH_STRONG)        std::printf("*");
            else if(v>=THRESH_FAINT)         std::printf(".");
            else                             std::printf(" ");
        }
        std::printf("|\n");
    }

    std::printf("  +");
    for(int x=0;x<V;x++) std::printf("-");
    std::printf("+  <- SAU XE\n");

    std::printf("  Vung: %.1fm x %.1fm (o=3cm)  Ctrl+C thoat\n",
                V*CELL_M, V*CELL_M);
    fflush(stdout);
}

int main(int argc, char** argv){
    std::signal(SIGINT, on_sig);
    std::signal(SIGTERM,on_sig);

    const char* ldev = argc>1 ? argv[1] : "/dev/ttyTHS1";
    const char* udev = argc>2 ? argv[2] : "/dev/ttyUSB0";

    std::printf("=== 2x LiDAR + 4x Ultrasonic Occupancy Grid ===\n");
    std::printf("LiDAR (2x VB22A):     %s @ 115200\n", ldev);
    std::printf("Ultrasonic (4x US):   %s @ 115200\n\n", udev);
    fflush(stdout);

    GridManager mgr(ldev, udev);
    mgr.start();

    std::printf("\033[?25l");

    uint8_t  snap[GRID_N*GRID_N];
    uint32_t last_l=0, last_u=0;
    auto     t0 = std::chrono::steady_clock::now();

    while(!g_quit.load()){
        mgr.snapshot(snap);
        auto now=std::chrono::steady_clock::now();
        double dt=std::chrono::duration<double>(now-t0).count();
        uint32_t cl=mgr.lidar_pts(), cu=mgr.us_pts();
        float hz=dt>0?(float)((cl-last_l)+(cu-last_u))/dt:0;
        last_l=cl; last_u=cu; t0=now;
        draw(mgr, snap, cl, cu, hz);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::printf("\033[?25h\033[2J\033[H");
    std::printf("Dung lai...\n");
    mgr.stop();
    std::printf("Tong: lidar=%u  us=%u\n", mgr.lidar_pts(), mgr.us_pts());
    return 0;
}