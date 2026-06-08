/**
 * grid_simple.hpp + grid_simple.cpp + main_simple.cpp
 * Pi nhan distance tu STM32, ve diem thang tren truc Y cua grid.
 * Khong co angle: tat ca diem duoc ve tai (X=0, Y=dist).
 *
 * Build:
 *   g++ -std=c++17 -O2 -lpthread grid_simple.cpp main_simple.cpp -o lidar_test
 *
 * Chay:
 *   ./lidar_test /dev/ttyAMA0      (UART hardware)
 *   ./lidar_test /dev/ttyACM0      (USB CDC tu Blue Pill)
 */

#pragma once
#include <cstdint>
#include <cstring>
#include <atomic>
#include <thread>
#include <functional>
#include <string>
#include <chrono>

/* ---- Grid 100x100, o 3cm, vung 3m x 3m ---- */
constexpr int   GRID_N   = 100;
constexpr float CELL_M   = 0.03f;
constexpr int   GRID_OX  = GRID_N / 2;   /* tam = o (50,50) */
constexpr int   GRID_OY  = GRID_N / 2;
constexpr uint8_t FREE      = 0;
constexpr uint8_t OBSTACLE  = 255;

/* ---- Packet 10 bytes tu STM32 ---- */
constexpr uint8_t PKT_HDR = 0xAA;
constexpr uint8_t PKT_FTR = 0x55;
constexpr int     PKT_LEN = 10;

struct ScanPoint {
    uint8_t  id;
    uint16_t dist_mm;
    uint32_t ts_ms;
};

/* ---- SPSC queue lock-free ---- */
template<typename T, size_t CAP>
class SpscQ {
public:
    bool push(const T& v) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t n = (h + 1) % CAP;
        if (n == tail_.load(std::memory_order_acquire)) return false;
        buf_[h] = v; head_.store(n, std::memory_order_release);
        return true;
    }
    bool pop(T& v) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        v = buf_[t]; tail_.store((t+1)%CAP, std::memory_order_release);
        return true;
    }
private:
    T                   buf_[CAP];
    std::atomic<size_t> head_{0}, tail_{0};
};

/* ---- Grid Manager ---- */
class GridSimple {
public:
    uint8_t data[GRID_N * GRID_N]{};

    explicit GridSimple(std::string dev, int baud = 921600)
        : dev_(std::move(dev)), baud_(baud) {}

    void start();
    void stop();

    void clear() { std::memset(data, FREE, sizeof(data)); }

    void snapshot(uint8_t* dst) const {
        std::memcpy(dst, data, sizeof(data));
    }

    uint32_t count() const { return count_.load(); }

    /* callback tuy chon de debug: cb(dist_mm, grid_y) */
    using Cb = std::function<void(uint16_t, int)>;
    void on_point(Cb cb) { cb_ = std::move(cb); }

private:
    std::string  dev_;
    int          baud_;
    int          fd_ = -1;

    SpscQ<ScanPoint, 512> queue_;
    std::atomic<bool>     running_{false};
    std::atomic<uint32_t> count_{0};
    std::thread           thr_r_, thr_u_;
    Cb                    cb_;

    bool uart_open();
    void uart_close();
    bool parse(const uint8_t* b, ScanPoint& o);
    void reader_loop();
    void updater_loop();
    void mark(uint16_t dist_mm);
};