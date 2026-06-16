/**
 * front_scanner.hpp
 * ============================================================
 * Nhan va xu ly du lieu tu 4 LiDAR VB22A dau xe may
 * (STM32F407VET6 -> USB-UART -> /dev/ttyUSB3 tren Jetson).
 *
 * PACKET NHAN (12 bytes tu STM32):
 *   [0]    = 0xCC  header
 *   [1]    = id    (0-3)
 *   [2,3]  = dist_mm  uint16 LE  (don vi MM, thang tu VB22A)
 *   [4,5]  = angle_10 int16  LE  (goc x10, luon=900 => 90.0 do)
 *   [6,7]  = ox_mm    int16  LE
 *   [8,9]  = oy_mm    int16  LE
 *   [10]   = checksum XOR byte[1..9]
 *   [11]   = 0x55  footer
 *
 * TINH TOAN DIEM TREN MAP (angle=90 do cho tat ca):
 *   wx = ox_m + dist_m * cos(90°) = ox_m        (chi phu thuoc vi tri lap)
 *   wy = oy_m + dist_m * sin(90°) = oy_m + dist_m  (khoang cach cong them)
 *
 * Vi du F0 (ox=-0.30m, oy=+0.90m), vat can o 2.5m:
 *   wx = -0.30m  (ben trai xe)
 *   wy = +0.90 + 2.50 = +3.40m (phia truoc)
 * ============================================================
 */
#pragma once
#include <cstdint>
#include <cmath>
#include <string>
#include <atomic>
#include <thread>
#include <functional>
#include <cstring>

static constexpr uint8_t FRONT_PKT_HDR  = 0xCC;
static constexpr uint8_t FRONT_PKT_FTR  = 0x55;
static constexpr int     FRONT_PKT_LEN  = 12;
static constexpr int     FRONT_N_LIDAR  = 4;

/* Loc khoang cach phia Jetson */
static constexpr float FRONT_DIST_MIN_M = 0.05f; /* blind zone 5cm */
static constexpr float FRONT_DIST_MAX_M = 20.0f;

struct FrontPoint {
    uint8_t  id;
    float    dist_m;
    float    angle_deg; /* luon 90.0 */
    float    wx;        /* = ox_m (khong doi theo dist) */
    float    wy;        /* = oy_m + dist_m */
    float    ox_m;
    float    oy_m;
    uint32_t ts_local;
};

class FrontScanner {
public:
    explicit FrontScanner(std::string dev, int baud = 460800)
        : dev_(std::move(dev)), baud_(baud) {}

    void start();
    void stop();

    uint32_t pts_total()    const { return pts_.load(); }
    uint32_t pts(int id)    const { return (id>=0&&id<FRONT_N_LIDAR)?pts_per_id_[id].load():0; }

    /* -1.0f neu chua co du lieu */
    float last_dist_m(int id) const {
        if (id < 0 || id >= FRONT_N_LIDAR) return -1.0f;
        uint32_t v = last_dist_raw_[id].load(std::memory_order_relaxed);
        float f; std::memcpy(&f, &v, sizeof(f));
        return f;
    }

    using PointCb = std::function<void(const FrontPoint&)>;
    void on_point(PointCb cb) { cb_ = std::move(cb); }

private:
    std::string dev_;
    int         baud_;
    int         fd_ = -1;

    std::atomic<bool>     running_{false};
    std::atomic<uint32_t> pts_{0};
    std::atomic<uint32_t> pts_per_id_[FRONT_N_LIDAR]{};
    std::atomic<uint32_t> last_dist_raw_[FRONT_N_LIDAR]{};

    PointCb     cb_ = nullptr;
    std::thread thr_;

    bool uart_open();
    void uart_close();
    bool parse(const uint8_t* b, FrontPoint& out);
    void reader_loop();
};