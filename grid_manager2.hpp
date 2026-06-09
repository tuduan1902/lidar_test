/**
 * grid_manager.hpp — 2x LiDAR + 4x Ultrasonic -> 1 Grid
 *
 * LiDAR LEFT  (id=0): front-left,  mount offset (-0.2m, +0.5m)
 * LiDAR RIGHT (id=1): front-right, mount offset (+0.2m, +0.5m)
 * Ultrasonic  (id S1..S4): truoc/sau/trai/phai
 *
 * Build:
 *   g++ -std=c++17 -O2 -lpthread grid_manager2.cpp main_combined2.cpp -o lidar_us
 * Chay:
 *   ./lidar_us [lidar_dev] [us_dev]
 *   ./lidar_us /dev/ttyTHS1 /dev/ttyUSB0
 */
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <atomic>
#include <thread>
#include <functional>
#include <string>
#include <chrono>

// ============================================================
//  GRID
// ============================================================
constexpr int   GRID_N   = 100;
constexpr float CELL_M   = 0.03f;
constexpr int   GRID_OX  = GRID_N / 2;
constexpr int   GRID_OY  = GRID_N / 2;
constexpr uint8_t THRESH_STRONG = 200;
constexpr uint8_t THRESH_FAINT  = 80;
constexpr uint8_t OBSTACLE      = 255;
constexpr uint8_t FREE          = 0;
constexpr uint8_t DECAY_STEP    = 40;
constexpr int     DECAY_MS      = 150;

// ============================================================
//  VI TRI 2 LIDAR TREN XE (chinh theo xe thuc te)
// ============================================================
// He truc: X=sang phai, Y=phia truoc xe
// mount_deg: 0=huong thang truoc, +deg=xoay sang phai
struct LidarMount {
    float ox, oy;       /* offset tu tam xe (m) */
    float mount_deg;    /* goc gan co dinh      */
};

constexpr LidarMount LIDAR_MOUNTS[2] = {
    { -0.20f, +0.50f,  0.0f },  /* id=0 LEFT:  20cm trai, 50cm truoc */
    { +0.20f, +0.50f,  0.0f },  /* id=1 RIGHT: 20cm phai, 50cm truoc */
};

// ============================================================
//  VI TRI 4 ULTRASONIC
// ============================================================
struct UsMountPoint { float ox, oy, dir_deg; };
constexpr UsMountPoint US_MOUNTS[4] = {
    {  0.0f, +0.4f,   0.0f },  /* S1 truoc  */
    {  0.0f, -0.4f, 180.0f },  /* S2 sau    */
    { -0.3f,  0.0f, 270.0f },  /* S3 trai   */
    { +0.3f,  0.0f,  90.0f },  /* S4 phai   */
};

// ============================================================
//  PACKET LIDAR (14 bytes)
// ============================================================
constexpr uint8_t LIDAR_HDR = 0xAA;
constexpr uint8_t LIDAR_FTR = 0x55;
constexpr int     LIDAR_PKT = 14;

struct LidarPoint {
    uint8_t  id;         /* 0=LEFT, 1=RIGHT */
    uint16_t dist_mm;
    float    angle_deg;  /* goc tuong doi so voi HOME */
    uint32_t ts_ms;
};

struct UsPoint { float dist_cm[4]; };

// ============================================================
//  SPSC QUEUE
// ============================================================
template<typename T, size_t CAP>
class SpscQ {
public:
    bool push(const T& v) {
        size_t h=head_.load(std::memory_order_relaxed);
        size_t n=(h+1)%CAP;
        if(n==tail_.load(std::memory_order_acquire)) return false;
        buf_[h]=v; head_.store(n,std::memory_order_release); return true;
    }
    bool pop(T& v) {
        size_t t=tail_.load(std::memory_order_relaxed);
        if(t==head_.load(std::memory_order_acquire)) return false;
        v=buf_[t]; tail_.store((t+1)%CAP,std::memory_order_release); return true;
    }
private:
    T buf_[CAP];
    std::atomic<size_t> head_{0},tail_{0};
};

// ============================================================
//  GRID MANAGER
// ============================================================
class GridManager {
public:
    uint8_t data[GRID_N * GRID_N]{};

    GridManager(std::string lidar_dev, std::string us_dev,
                int lidar_baud=115200, int us_baud=115200)
        : lidar_dev_(std::move(lidar_dev)), us_dev_(std::move(us_dev))
        , lidar_baud_(lidar_baud), us_baud_(us_baud) {}

    void start();
    void stop();
    void snapshot(uint8_t* dst) const { std::memcpy(dst,data,sizeof(data)); }
    uint32_t lidar_pts() const { return lidar_count_.load(); }
    uint32_t us_pts()    const { return us_count_.load(); }

    /* Last values de hien thi */
    uint16_t last_dist[2]  = {0xFFFF, 0xFFFF};
    float    last_angle[2] = {0.0f, 0.0f};
    float    last_us[4]    = {-1,-1,-1,-1};

private:
    std::string lidar_dev_, us_dev_;
    int         lidar_baud_, us_baud_;
    int         lidar_fd_=-1, us_fd_=-1;

    SpscQ<LidarPoint,512> lidar_q_;
    SpscQ<UsPoint,   64>  us_q_;

    std::atomic<bool>     running_{false};
    std::atomic<uint32_t> lidar_count_{0}, us_count_{0};

    std::thread thr_lidar_r_, thr_us_r_, thr_updater_, thr_decay_;

    bool uart_open(const std::string& dev, int baud, int& fd);
    void uart_close(int& fd);
    bool parse_lidar(const uint8_t* b, LidarPoint& o);
    bool parse_us_json(const char* line, UsPoint& o);
    void mark_xy(float wx, float wy);
    void mark_lidar(const LidarPoint& p);
    void mark_us(const UsPoint& p);
    void lidar_reader_loop();
    void us_reader_loop();
    void updater_loop();
    void decay_loop();
};