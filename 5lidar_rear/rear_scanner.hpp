/**
 * rear_scanner.hpp
 * ============================================================
 * Nhan va xu ly du lieu tu 5 LiDAR VB22A gan sau duoi xe may
 * (STM32F407VET6 -> USB-UART -> /dev/ttyUSBx tren Jetson Orin Nano).
 *
 * PACKET (14 bytes, xem rear_lidar_stm32.h):
 *   [0]    = 0xBB  header
 *   [1]    = id    (0-4)
 *   [2,3]  = dist_cm     uint16 LE  (don vi cm)
 *   [4,5]  = strength    uint16 LE
 *   [6,7]  = temp_raw    uint16 LE  (nhiet do x10 do C)
 *   [8,9]  = angle_10    int16 LE   (goc x10 do, vd 1800 = 180.0 do)
 *   [10,11]= ox_mm       int16 LE   (offset X mm)
 *   [12]   = checksum XOR byte[1..11]
 *   [13]   = 0x55  footer
 *
 * VI TRI SENSOR:
 *   ox_m  = ox_mm / 1000.0 (offset X, + = phai xe)
 *   oy_m  = REAR_OY_M (hang so, goc Y cua cum sensor, < 0 = phia sau xe)
 *          -> Vi tri Y KHONG gui trong packet de tiet kiem byte;
 *             duoc cau hinh o day bang REAR_MOUNT_OY_M.
 *
 * TINH TOAN DIEM TREN MAP:
 *   angle_rad = angle_deg * PI / 180
 *   wx = ox_m + dist_m * cos(angle_rad)   (truc X: phai duong)
 *   wy = oy_m + dist_m * sin(angle_rad)   (truc Y: truoc duong)
 *
 *   L0 (180 do): wx = ox + dist*cos(180) = ox - dist, wy = oy + 0
 *                -> diem nam thang phia sau xe, dung
 *   L2 (135 do): wx = ox + dist*cos(135), wy = oy + dist*sin(135)
 *                -> diem lech trai va phia sau
 *
 * TICH HOP:
 *   Tao RearScanner(dev, baud), goi start()/stop().
 *   Pass con tro FilteredMap de RearScanner tu dong ghi diem vao map.
 *   Hoac dung callback on_point(cb) neu muon xu ly tuy chinh.
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
 * Toa do Y cua vi tri dat cum LiDAR so voi tam xe (m).
 * < 0 = phia sau xe. Nen khop voi oy_m trong rear_lidar_stm32.h.
 * ============================================================ */
static constexpr float REAR_MOUNT_OY_M = -0.85f; /* 85cm phia sau tam xe */

/* ============================================================
 * GIAO THUC PACKET
 * ============================================================ */
static constexpr uint8_t REAR_PKT_HDR  = 0xBB;
static constexpr uint8_t REAR_PKT_FTR  = 0x55;
static constexpr int     REAR_PKT_LEN  = 14;
static constexpr int     REAR_N_LIDAR  = 5;

/* ============================================================
 * NGUONG LOC PHIA JETSON (co the khac STM32 neu can tinh chinh)
 * ============================================================ */
static constexpr float REAR_DIST_MIN_M = 0.20f;
static constexpr float REAR_DIST_MAX_M = 8.00f;

/* ============================================================
 * STRUCTS
 * ============================================================ */
struct RearPoint {
    uint8_t  id;        /* 0-4 */
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

    /* So diem da nhan thanh cong tu moi sensor */
    uint32_t pts_total()      const { return pts_.load(); }
    uint32_t pts(int id)      const { return (id >= 0 && id < REAR_N_LIDAR) ? pts_per_id_[id].load() : 0; }

    /* Khoang cach gan nhat cua tung sensor (m), -1 neu chua co du lieu */
    float last_dist_m(int id) const {
        if (id < 0 || id >= REAR_N_LIDAR) return -1.0f;
        uint32_t v = last_dist_raw_[id].load(std::memory_order_relaxed);
        float f; std::memcpy(&f, &v, sizeof(f));
        return f;
    }

    /* Callback khi co diem hop le (goi tu reader thread).
     * Dung de gui diem len FilteredMap hoac xu ly khac. */
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