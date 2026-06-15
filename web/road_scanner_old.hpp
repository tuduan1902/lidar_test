/**
 * road_scanner.hpp
 * ============================================================
 * LiDAR VB22A 1 tia, co dinh, chui xuong mat duong
 *
 * HINH HOC (LiDAR khong xoay):
 *
 *         LiDAR (h=1m)
 *             *
 *              \  <- 1 tia co dinh
 *               \  slant = 6m
 *                \  goc pitch = arcsin(1/6) ~ 9.6 do
 *                 \
 *   _______________*____________  mat duong
 *                  ^
 *                  diem chieu (5.9m phia truoc)
 *
 * PHAT HIEN O GA:
 *   - Tinh "dist_ground_expected" = khoang cach khi mat phang
 *     = H_MOUNT / sin(pitch_total)
 *   - O ga: dist_measured > expected + POTHOLE_THRESH
 *     (tia xuyen qua o, chieu xa hon)
 *   - Vat can: dist_measured < expected - OBSTACLE_THRESH
 *     (co vat chan tia lai)
 *
 * BU PHUOC (suspension compensation):
 *   - Khi xe thang: phuoc nen, mui xe nghieng xuong them ~2-5 do
 *   - pitch_total = PITCH_STATIC + pitch_from_suspension
 *   - expected_dist thay doi -> tranh bao dong gia
 *   - set_suspension(compression_mm) de cap nhat
 *
 * LOC NHIEU RUNG:
 *   - EMA alpha=0.15 cho dist (loc rung phuoc 5-15Hz)
 *   - Confirm N=3 mau lien tiep (15ms) truoc khi bao cao
 *   - Baseline hoc tu 50 mau dau (khong bao cao trong giai doan hoc)
 *
 * TU CALIB GOC NGHIENG (QUAN TRONG):
 *   - H_MOUNT/SLANT_RANGE_REF/PITCH_STATIC_RAD chi la gia tri THIET KE
 *     (vd: SLANT_RANGE_REF=2.0m -> PITCH_STATIC_RAD=30 deg). Goc lap
 *     thuc te tren xe gan nhu chac chan KHAC gia tri nay -> neu dung
 *     truc tiep, expected_dist se sai va moi mau deu bi bao "vat can".
 *   - Trong N_BASELINE mau dau (xe dung tren mat duong PHANG, khong
 *     phanh/khong vat can), code se trung binh ema_dist_ -> baseline_dist_
 *     roi tinh nguoc baseline_pitch_rad_ = asin(H_MOUNT/baseline_dist_).
 *   - Tu sau do, expected_dist = H_MOUNT / sin(baseline_pitch_rad_ +
 *     pitch_dynamic_tu_phuoc). Tai thoi diem calib, ema_dist_ ==
 *     baseline_dist_ == expected_dist -> delta = 0 -> KHONG bao dong.
 *   - PITCH_STATIC_RAD / SLANT_RANGE_REF van giu lai chi de tham khao /
 *     log canh bao neu baseline_pitch_rad_ lech qua nhieu so voi thiet ke.
 *
 * OUTPUT:
 *   - road_timeline: mang dist theo thoi gian (ve bieu do)
 *   - pothole_events: danh sach vi tri o ga phat hien
 *   - callback khi phat hien su kien
 * ============================================================
 */
#pragma once
#include <math.h>
#include <cstdint>
#include <cmath>
#include <array>
#include <deque>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <cstring>
#include <vector>

/* ============================================================
 * CAU HINH HINH HOC
 * ============================================================ */
// Góc nghieng xuống là góc giữa tia laser và phương nằm ngang
static constexpr float H_MOUNT          = 1.0f;    /* m - do cao LiDAR */
static constexpr float SLANT_RANGE_REF  = 2.0f;    /* m - slant range toi mat phang (THIET KE, xem tu calib) */
static const float PITCH_STATIC_RAD = asin(H_MOUNT / SLANT_RANGE_REF); /* rad - goc nghieng THIET KE, dung lam fallback/tham khao */
static constexpr float PITCH_PER_MM     = 0.000873f;/* rad/mm phuoc nen */

/* Gioi han an toan cho goc nghieng sau khi tu calib (tranh asin() loi/NaN
 * neu baseline_dist_ qua nho hoac qua lon do nhieu/lap dat bat thuong) */
static constexpr float PITCH_MIN_RAD = 0.0873f; /* ~5 deg  - goc qua nong (gan nam ngang) */
static constexpr float PITCH_MAX_RAD = 1.3963f; /* ~80 deg - goc qua dung (gan thang xuong) */

/* X,Y offset LiDAR so voi tam xe */
static constexpr float LIDAR_OX = 0.0f;
static constexpr float LIDAR_OY = 0.8f; /* 80cm phia truoc */

/* ============================================================
 * NGUONG PHAT HIEN
 * ============================================================ */
static constexpr float POTHOLE_THRESH_M  = 0.05f; /* o ga: 5cm sau */
static constexpr float OBSTACLE_THRESH_M = 0.05f; /* vat can: 5cm cao */
static constexpr float DIST_MAX_M        = 9.0f;
static constexpr float DIST_MIN_M        = 0.3f;

/* ============================================================
 * BO LOC
 * ============================================================ */
static constexpr float EMA_ALPHA    = 0.15f; /* loc rung phuoc */
static constexpr int   N_CONFIRM    = 3;     /* mau lien tiep de xac nhan */
static constexpr int   N_BASELINE   = 50;    /* so mau hoc baseline ban dau */
static constexpr int   TIMELINE_LEN = 300;   /* luu 300 diem gan nhat de ve */

/* ============================================================
 * PACKET (10 bytes tu STM32)
 * ============================================================ */
static constexpr uint8_t RD_HDR = 0xAA;
static constexpr uint8_t RD_FTR = 0x55;
static constexpr int     RD_PKT = 10;

/* ============================================================
 * STRUCTS
 * ============================================================ */
struct RoadSample {
    uint32_t ts_ms;
    float    dist_raw_m;     /* tu VB22A, chua loc */
    float    dist_ema_m;     /* sau EMA */
    float    dist_expect_m;  /* khoang cach du kien khi mat phang */
    float    delta_m;        /* dist_ema - dist_expect (am=o ga, duong=vat can) */
    bool     is_pothole;
    bool     is_obstacle;
    bool     is_valid;
};

struct PotholeEvent {
    uint32_t ts_ms;
    float    depth_m;   /* uoc tinh do sau o ga */
    float    x_ahead_m; /* khoang cach phia truoc xe toi o ga */
};

/* ============================================================
 * ROAD SCANNER
 * ============================================================ */
class RoadScanner {
public:
    explicit RoadScanner(std::string dev, int baud = 115200)
        : dev_(std::move(dev)), baud_(baud) {}

    void start();
    void stop();

    /* Cap nhat trang thai phuoc (mm nen) */
    void set_suspension(float compression_mm) {
        float pr = compression_mm * PITCH_PER_MM;
        pitch_dynamic_.store(*reinterpret_cast<uint32_t*>(&pr));
    }

    /* Lay timeline gan nhat de ve web */
    void get_timeline(std::vector<RoadSample>& out) const {
        std::lock_guard<std::mutex> lk(mtx_);
        out.assign(timeline_.begin(), timeline_.end());
    }

    /* Lay danh sach o ga */
    void get_potholes(std::vector<PotholeEvent>& out) const {
        std::lock_guard<std::mutex> lk(mtx_);
        out = potholes_;
    }

    uint32_t pts()       const { return pts_.load(); }
    uint32_t n_pothole() const { return npot_.load(); }
    uint32_t n_obstacle()const { return nobs_.load(); }

    float last_dist_m()  const { return last_dist_.load() / 1000.0f; }
    float last_delta_m() const {
        uint32_t v = last_delta_raw_.load();
        return *reinterpret_cast<const float*>(&v);
    }

    /* Trang thai tu-calib goc nghieng (xem comment dau file) */
    bool  is_calibrated()      const { return calibrated_.load(std::memory_order_relaxed); }
    float baseline_dist_m()    const {
        uint32_t v = baseline_dist_raw_.load(std::memory_order_relaxed);
        float f; std::memcpy(&f, &v, sizeof(f));
        return f;
    }
    float baseline_pitch_deg() const {
        uint32_t v = baseline_pitch_raw_.load(std::memory_order_relaxed);
        float f; std::memcpy(&f, &v, sizeof(f));
        return f * (180.0f / 3.14159265358979323846f);
    }
    /* Goc nghieng (rad) hien dang dung de tinh expected_dist. Truoc khi calib
     * xong (is_calibrated()==false) tra ve PITCH_STATIC_RAD (gia tri thiet ke). */
    float baseline_pitch_rad() const {
        if (!calibrated_.load(std::memory_order_relaxed)) return PITCH_STATIC_RAD;
        uint32_t v = baseline_pitch_raw_.load(std::memory_order_relaxed);
        float f; std::memcpy(&f, &v, sizeof(f));
        return f;
    }

    using Cb = std::function<void(const RoadSample&)>;
    void on_pothole (Cb c) { cb_pot_ = std::move(c); }
    void on_obstacle(Cb c) { cb_obs_ = std::move(c); }

private:
    std::string dev_; int baud_; int fd_ = -1;

    std::atomic<bool>     running_{false};
    std::atomic<uint32_t> pts_{0}, npot_{0}, nobs_{0};
    std::atomic<uint32_t> last_dist_{0xFFFF};
    std::atomic<uint32_t> pitch_dynamic_{0};
    std::atomic<uint32_t> last_delta_raw_{0};

    /* Tu-calib goc nghieng: ket qua sau N_BASELINE mau, doc tu thread khac (web) */
    std::atomic<bool>     calibrated_{false};
    std::atomic<uint32_t> baseline_dist_raw_{0};
    std::atomic<uint32_t> baseline_pitch_raw_{0};

    Cb cb_pot_ = nullptr;
    Cb cb_obs_ = nullptr;

    mutable std::mutex           mtx_;
    std::deque<RoadSample>       timeline_;
    std::vector<PotholeEvent>    potholes_;

    /* EMA state */
    float   ema_dist_  = 0.0f;
    bool    ema_init_  = false;

    /* Baseline hoc (tu calib goc nghieng - xem comment dau file) */
    int     baseline_n_         = 0;
    double  baseline_sum_        = 0.0;  /* tong ema_dist_ trong giai doan hoc */
    float   baseline_dist_       = 0.0f; /* dist trung binh khi mat phang (sau khi hoc xong) */
    float   baseline_pitch_rad_  = PITCH_STATIC_RAD; /* goc nghieng dung de tinh expected_dist, mac dinh = gia tri thiet ke truoc khi calib xong */

    /* Confirm counters */
    int pothole_confirm_  = 0;
    int obstacle_confirm_ = 0;

    /* SPSC queue */
    static constexpr int Q = 256;
    struct PktQ { uint16_t dist_mm; uint32_t ts; };
    PktQ qbuf_[Q];
    std::atomic<int> qh_{0}, qt_{0};
    bool qpush(const PktQ& p);
    bool qpop (PktQ& p);

    std::thread thr_r_, thr_p_;

    bool uart_open();
    void uart_close();
    bool parse(const uint8_t* b, uint16_t& dist_mm, uint32_t& ts);
    void reader_loop();
    void process_loop();

    float get_pitch_dynamic() const {
        uint32_t v = pitch_dynamic_.load();
        return *reinterpret_cast<const float*>(&v);
    }

    /* Tinh khoang cach du kien khi mat dat phang */
    float expected_dist(float pitch_total) const {
        /* dist = H_MOUNT / sin(pitch_total) */
        float sp = sinf(pitch_total);
        if (sp < 0.01f) sp = 0.01f;
        return H_MOUNT / sp;
    }
};