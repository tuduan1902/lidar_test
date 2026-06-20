/**
 * rear_lidar_3.hpp
 * ============================================================
 * Nhan va xu ly du lieu tu 3 LiDAR VB22A gan sau duoi xe may
 * (STM32F407VET6 -> USB-UART -> /dev/ttyUSBx tren Jetson Orin Nano).
 * ============================================================
 */
#pragma once
#include <cstdint>
#include <cmath>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <array>
#include <cstring>

/* ============================================================
 * CAU HINH VI TRI CUM SENSOR (DIEU CHINH THEO THUC TE)
 * ============================================================ */
static constexpr float REAR_MOUNT_OY_M = -0.85f; /* 85cm phia sau tam xe */

/* ============================================================
 * GIAO THUC PACKET
 * ============================================================ */
static constexpr uint8_t REAR_PKT_HDR  = 0xBB;
static constexpr uint8_t REAR_PKT_FTR  = 0x55;
static constexpr int     REAR_PKT_LEN  = 14;

// ĐÃ SỬA: Khớp với STM32 (từ 5 xuống 3)
static constexpr int     REAR_N_LIDAR  = 3;

/* ============================================================
 * NGUONG LOC PHIA JETSON
 * ============================================================ */
static constexpr float REAR_DIST_MIN_M = 0.10f;
static constexpr float REAR_DIST_MAX_M = 20.00f;

/* ============================================================
 * STRUCTS
 * ============================================================ */
struct RearPoint {
    uint8_t  id;        /* 0-2 */
    float    dist_m;
    float    angle_deg;
    float    wx;        /* toa do the gioi, truc X xe */
    float    wy;        /* toa do the gioi, truc Y xe */
    float    ox_m;      /* offset X cua sensor */
    uint16_t strength;
    float    temp_c;    /* nhiet do cam bien (do C) */
    uint32_t ts_local;  /* timestamp Jetson (ms tu epoch) */
};

/* ============================================================
 * REAR SCANNER CLASS
 * ============================================================ */
class RearScanner {
public:
    explicit RearScanner(std::string dev, int baud = 460800)
        : dev_(std::move(dev)), baud_(baud) {}

    void start();
    void stop();

    uint32_t pts_total()      const { return pts_.load(); }
    uint32_t pts(int id)      const { return (id >= 0 && id < REAR_N_LIDAR) ? pts_per_id_[id].load() : 0; }

    float last_dist_m(int id) const {
        if (id < 0 || id >= REAR_N_LIDAR) return -1.0f;
        uint32_t v = last_dist_raw_[id].load(std::memory_order_relaxed);
        float f; std::memcpy(&f, &v, sizeof(f));
        return f;
    }

    using PointCb = std::function<void(const RearPoint&)>;
    void on_point(PointCb cb) { cb_ = std::move(cb); }

private:
    std::string dev_;
    int         baud_;
    int         fd_ = -1;

    std::atomic<bool>     running_{false};
    std::atomic<uint32_t> pts_{0};
    std::atomic<uint32_t> pts_per_id_[REAR_N_LIDAR]{};
    std::atomic<uint32_t> last_dist_raw_[REAR_N_LIDAR]{};

    PointCb cb_ = nullptr;
    std::thread thr_;

    bool uart_open();
    void uart_close();
    bool parse(const uint8_t* b, RearPoint& out);
    void reader_loop();
};