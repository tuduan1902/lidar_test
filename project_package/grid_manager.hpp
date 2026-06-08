/**
 * grid_manager.hpp
 * Occupancy grid 100x100 (3m x 3m, o 3cm)
 * Nhan 2 nguon du lieu:
 *   1. LiDAR STM32   -> /dev/ttyTHS1 @ 115200  packet 14 bytes binary
 *   2. Ultrasonic STM32 -> /dev/ttyUSB0 @ 115200  JSON text
 *
 * CA HAI UPDATE CUNG 1 GRID, decay 40/150ms -> vat can tu xoa sau ~1s
 *
 * Build:
 *   g++ -std=c++17 -O2 -lpthread grid_manager.cpp main_combined.cpp -o lidar_us
 * Chay:
 *   ./lidar_us
 * Hoac chi dinh device:
 *   ./lidar_us /dev/ttyTHS1 /dev/ttyUSB0
 */
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <string>
#include <chrono>

// ============================================================
//  GRID CONFIG
// ============================================================
constexpr int   GRID_N   = 100;
constexpr float CELL_M   = 0.03f;      // 3cm moi o
constexpr int   GRID_OX  = GRID_N / 2; // tam xe = o (50,50)
constexpr int   GRID_OY  = GRID_N / 2;

constexpr uint8_t THRESH_STRONG = 200;  // hien thi *
constexpr uint8_t THRESH_FAINT  = 80;   // hien thi .
constexpr uint8_t OBSTACLE      = 255;
constexpr uint8_t FREE          = 0;

constexpr uint8_t DECAY_STEP    = 40;   // giam moi chu ky
constexpr int     DECAY_MS      = 150;  // chu ky decay (ms)

// ============================================================
//  VI TRI CAM BIEN TREN XE (chinh theo xe thuc te)
// ============================================================
// He truc: X=sang phai, Y=phia truoc xe, goc = tam xe
// LiDAR don diem: gan phia truoc
constexpr float LIDAR_OX  =  0.0f;
constexpr float LIDAR_OY  =  0.3f;   // 30cm phia truoc tam xe
constexpr float LIDAR_MOUNT_DEG = 0.0f;

// 4 Ultrasonic: US1/US2 ben trai, US3/US4 ben phai
// (chinh lai sau khi do xe thuc te)
struct UsMountPoint { float ox, oy, dir_deg; };
constexpr UsMountPoint US_MOUNTS[4] = {
    { -0.3f,  0.2f, 270.0f },  // S1: trai truoc
    { -0.3f, -0.2f, 270.0f },  // S2: trai sau
    {  0.3f,  0.2f,  90.0f },  // S3: phai truoc
    {  0.3f, -0.2f,  90.0f },  // S4: phai sau
};

// ============================================================
//  LIDAR PACKET (14 bytes)
// ============================================================
constexpr uint8_t LIDAR_HDR = 0xAA;
constexpr uint8_t LIDAR_FTR = 0x55;
constexpr int     LIDAR_PKT = 14;

struct LidarPoint {
    uint16_t dist_mm;
    float    angle_deg;
    uint32_t ts_ms;
};

// ============================================================
//  SPSC QUEUE
// ============================================================
template<typename T, size_t CAP>
class SpscQ {
public:
    bool push(const T& v) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t n = (h+1)%CAP;
        if (n == tail_.load(std::memory_order_acquire)) return false;
        buf_[h]=v; head_.store(n,std::memory_order_release); return true;
    }
    bool pop(T& v) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        v=buf_[t]; tail_.store((t+1)%CAP,std::memory_order_release); return true;
    }
private:
    T buf_[CAP];
    std::atomic<size_t> head_{0}, tail_{0};
};

struct UsPoint { float dist_cm[4]; };

// ============================================================
//  GRID MANAGER
// ============================================================
class GridManager {
public:
    uint8_t data[GRID_N * GRID_N]{};

    GridManager(std::string lidar_dev, std::string us_dev,
                int lidar_baud=115200, int us_baud=115200)
        : lidar_dev_(std::move(lidar_dev))
        , us_dev_(std::move(us_dev))
        , lidar_baud_(lidar_baud)
        , us_baud_(us_baud) {}

    void start();
    void stop();
    void snapshot(uint8_t* dst) const { std::memcpy(dst, data, sizeof(data)); }

    uint32_t lidar_pts()  const { return lidar_count_.load(); }
    uint32_t us_pts()     const { return us_count_.load(); }

private:
    std::string lidar_dev_, us_dev_;
    int         lidar_baud_, us_baud_;
    int         lidar_fd_ = -1;
    int         us_fd_    = -1;

    SpscQ<LidarPoint, 512>  lidar_q_;
    SpscQ<UsPoint,    64>   us_q_;

    std::atomic<bool>     running_{false};
    std::atomic<uint32_t> lidar_count_{0};
    std::atomic<uint32_t> us_count_{0};

    std::thread thr_lidar_r_;   // LiDAR UART reader
    std::thread thr_us_r_;      // Ultrasonic UART reader
    std::thread thr_updater_;   // Grid updater (xu ly ca 2 queue)
    std::thread thr_decay_;     // Decay thread

    // UART
    bool uart_open(const std::string& dev, int baud, int& fd);
    void uart_close(int& fd);

    // Parse
    bool parse_lidar(const uint8_t* b, LidarPoint& o);
    bool parse_us_json(const char* line, UsPoint& o);

    // Threads
    void lidar_reader_loop();
    void us_reader_loop();
    void updater_loop();
    void decay_loop();

    // Grid ops
    void mark_lidar(const LidarPoint& p);
    void mark_us(const UsPoint& p);
    void mark_xy(float wx, float wy);
};