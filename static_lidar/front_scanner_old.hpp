/**
 * front_scanner.hpp
 * ============================================================
 * Nhan va xu ly 5 LiDAR VB22A cum dau xe tu STM32 (/dev/ttyUSB3).
 *
 * id=0-3 : Chieu thang truoc
 *   wx = ox_m + dist_m * cos(90°) = ox_m
 *   wy = oy_m + dist_m * sin(90°) = oy_m + dist_m
 *
 * id=4   : LiDAR nghieng quet o ga (thay the road_scanner.cpp)
 *   KHONG dung cong thuc wx/wy nhu 4 con kia.
 *   Dung hinh hoc pitch 3D rieng:
 *     expected_dist = H_MOUNT / sin(pitch_total)
 *     delta = expected_dist - ema_dist
 *     delta < -POTHOLE_THRESH  -> O GA
 *     delta > +OBSTACLE_THRESH -> VAT CAN / GO GIAM TOC
 *   Diem o ga/vat can duoc map tai:
 *     wx = ox_m = 0 (tia chui chinh giua xe)
 *     wy = oy_m + ema_dist * cos(pitch)
 *
 * CAU HINH HINH HOC LIDAR NGHIENG (sua theo thuc te lap dat):
 *   FRONT_H_MOUNT        : do cao so voi mat duong (m)
 *   FRONT_SLANT_RANGE_REF: slant range khi mat phang (m) - CAN DO THUC TE
 *   FRONT_PITCH_STATIC_RAD = asin(H/SLANT)
 *
 * LOC NHIEU id=4:
 *   EMA alpha=0.15 (loc rung), confirm 3 mau lien tiep, N_BASELINE=50
 *
 * PACKET NHAN (12 bytes tu STM32):
 *   [0]=0xCC [1]=id [2,3]=dist_mm [4,5]=angle_10
 *   [6,7]=ox_mm [8,9]=oy_mm [10]=XOR[1..9] [11]=0x55
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
#include <cstring>

/* ============================================================
 * HINH HOC LIDAR NGHIENG (id=4) - CHINH THEO THUC TE
 * ============================================================ */
static constexpr float FRONT_H_MOUNT         = 1.0f;   /* m - do cao LiDAR nghieng */
static constexpr float FRONT_SLANT_RANGE_REF = 2.0f;   /* m - CAN DO THUC TE tren mat phang */
static const     float FRONT_PITCH_STATIC_RAD =
    asinf(FRONT_H_MOUNT / FRONT_SLANT_RANGE_REF);      /* rad - goc chui thiet ke */

/* Nguong phat hien */
static constexpr float FRONT_POTHOLE_THRESH_M  = 0.05f; /* o ga: 5cm */
static constexpr float FRONT_OBSTACLE_THRESH_M = 0.05f; /* vat can: 5cm */
static constexpr float FRONT_DIST_MIN_M        = 0.05f; /* blind zone VB22A */
static constexpr float FRONT_DIST_MAX_M        = 20.0f;

/* Bo loc id=4 */
static constexpr float FRONT_EMA_ALPHA  = 0.15f;
static constexpr int   FRONT_N_CONFIRM  = 3;
static constexpr int   FRONT_N_BASELINE = 50;

/* ============================================================
 * PACKET
 * ============================================================ */
static constexpr uint8_t FRONT_PKT_HDR = 0xCC;
static constexpr uint8_t FRONT_PKT_FTR = 0x55;
static constexpr int     FRONT_PKT_LEN = 12;
static constexpr int     FRONT_N_LIDAR = 5;
static constexpr int     FRONT_ID_TILT = 4;    /* id LiDAR nghieng */

/* ============================================================
 * STRUCTS
 * ============================================================ */
struct FrontPoint {
    uint8_t  id;
    float    dist_m;
    float    angle_deg;
    float    wx, wy;        /* toa do map (chi dung cho id 0-3) */
    float    ox_m, oy_m;
    uint32_t ts_local;

    /* Chi co y nghia khi id=4 */
    float    delta_m;       /* expected - ema: am=o ga, duong=vat can */
    bool     is_pothole;
    bool     is_obstacle;
};

/* ============================================================
 * FRONT SCANNER
 * ============================================================ */
class FrontScanner {
public:
    explicit FrontScanner(std::string dev, int baud = 460800)
        : dev_(std::move(dev)), baud_(baud) {}

    void start();
    void stop();

    uint32_t pts_total()      const { return pts_.load(); }
    uint32_t pts(int id)      const {
        return (id >= 0 && id < FRONT_N_LIDAR) ? pts_per_id_[id].load() : 0;
    }

    /* Khoang cach gan nhat cua tung sensor (m), -1 neu chua co */
    float last_dist_m(int id) const {
        if (id < 0 || id >= FRONT_N_LIDAR) return -1.0f;
        uint32_t v = last_dist_raw_[id].load(std::memory_order_relaxed);
        float f; std::memcpy(&f, &v, sizeof(f));
        return f;
    }

    /* Callback chung cho tat ca sensor (id 0-4) */
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

    /* Trang thai EMA va confirm cho id=4 (LiDAR nghieng) */
    float ema_dist_    = 0.0f;
    bool  ema_init_    = false;
    int   baseline_n_  = 0;
    int   pothole_confirm_  = 0;
    int   obstacle_confirm_ = 0;

    bool uart_open();
    void uart_close();
    bool parse(const uint8_t* b, FrontPoint& out);
    bool parse_packet(const uint8_t* b, uint16_t& dist_mm_out,
                      float& ox_m, float& oy_m, uint8_t& id_out);
    FrontPoint process_straight(uint8_t id, float dist_m,
                                float ox_m, float oy_m);
    FrontPoint process_tilt(float dist_m, float ox_m, float oy_m);
    void reader_loop();

    float expected_dist(float pitch) const {
        float sp = sinf(pitch);
        if (sp < 0.01f) sp = 0.01f;
        return FRONT_H_MOUNT / sp;
    }
}; 