/**
 * road_scanner.hpp
 * ============================================================
 * LiDAR VB22A chui xuong quet mat duong tu dau xe
 *
 * Mo hinh hinh hoc:
 *   - LiDAR gan o do cao H_MOUNT = 1.0m so voi mat duong
 *   - Goc chui xuong co dinh (do co khi): PITCH_STATIC_DEG ~ 9.6 do
 *     (arcsin(1.0/6.0) = 9.594 do khi slant range = 6m)
 *   - Khi xe thang / vuot o ga: phuoc nen -> xe nghieng them PITCH_DYNAMIC
 *   - LiDAR quet qua lai theo goc SCAN, moi goc cho 1 gia tri dist
 *
 * Cong thuc tinh chieu cao mat duong tai goc quet (scan_deg):
 *   total_pitch = pitch_static + pitch_dynamic   (rad)
 *   total_angle = total_pitch + scan_deg_to_rad
 *   h_road = dist * sin(total_angle)
 *   x_road = dist * cos(total_angle) * sin(scan_deg)
 *   y_road = dist * cos(total_angle) * cos(scan_deg) + LIDAR_OFFSET_Y
 *
 * Phat hien o ga:
 *   - Tinh h_road tai moi diem
 *   - Mat duong phang: h_road xap xi H_MOUNT (sai so < POTHOLE_THRESHOLD)
 *   - O ga: h_road > H_MOUNT + POTHOLE_THRESHOLD (diem o sau hon)
 *   - Goc cao: h_road < H_MOUNT - OBSTACLE_THRESHOLD (vat can phia truoc)
 *
 * Loc nhieu rung phuoc:
 *   - Dung EMA (Exponential Moving Average) cho dist
 *   - Dung median filter 3-mau cho phat hien su kien
 *   - Chi bao cao o ga neu lien tuc N_CONFIRM mau lien tiep
 * ============================================================
 */
#pragma once
#include <cstdint>
#include <cmath>
#include <array>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <cstring>

/* ============================================================
 * CAU HINH HINH HOC (chinh theo xe thuc te)
 * ============================================================ */

/* Do cao LiDAR so voi mat duong khi xe dung yen (m) */
static constexpr float H_MOUNT = 1.0f;

/* Slant range khi quet thang (m) - dung de tinh goc mac dinh */
static constexpr float SLANT_RANGE_REF = 6.0f;

/* Goc chui xuong co dinh (rad)
 * = arcsin(H_MOUNT / SLANT_RANGE_REF) = arcsin(1/6) ~ 0.1674 rad ~ 9.59 do
 * Chinh lai bang cach do thuc te: dat thuoc ngang 6m truoc xe,
 * lam mat phang, doc dist tu LiDAR, tinh lai goc */
static constexpr float PITCH_STATIC_RAD =
    0.16745f; /* arcsin(1.0/6.0) */

/* Do nhay phuoc: bao nhieu rad moi mm phuoc nen
 * Phuoc xe may dien hinh: hanh trinh 100mm ~ 5 do nghieng mui xe
 * -> 5*pi/180 / 100 = 0.000873 rad/mm
 * Chinh lai sau khi do thuc te */
static constexpr float PITCH_PER_MM_SUSPENSION = 0.000873f;

/* Vi tri LiDAR tren xe (m, tu tam xe) */
static constexpr float LIDAR_OFFSET_Y = 0.8f;  /* 80cm phia truoc */
static constexpr float LIDAR_OFFSET_X = 0.0f;  /* giua xe */

/* ============================================================
 * NGUONG PHAT HIEN
 * ============================================================ */

/* O ga: h_road > H_MOUNT + nguong nay thi coi la o ga (m)
 * 0.05m = 5cm do sau -> dieu chinh theo do nhay mong muon */
static constexpr float POTHOLE_DEPTH_THRESH = 0.05f;

/* Vat can: h_road < H_MOUNT - nguong nay (m)
 * Vat cao hon mat duong > 3cm */
static constexpr float OBSTACLE_HEIGHT_THRESH = 0.03f;

/* Slant range toi da hop le (m) - qua day la nhieu hoac khong co gi */
static constexpr float DIST_MAX_VALID = 8.0f;

/* Slant range toi thieu hop le (m) */
static constexpr float DIST_MIN_VALID = 0.3f;

/* ============================================================
 * BO LOC NHIEU
 * ============================================================ */

/* He so EMA cho dist: alpha ~ 0.3 la trung binh
 * alpha cao (0.7-0.9): phan ung nhanh, nhieu nhieu
 * alpha thap (0.1-0.3): mat phan ung, loc nhieu tot
 * Phuoc xe may rung ~5-15Hz, LiDAR 200Hz -> alpha=0.15 la hop ly */
static constexpr float EMA_ALPHA = 0.15f;

/* So mau lien tiep can de xac nhan o ga / vat can
 * LiDAR 200Hz -> 3 mau = 15ms, du de loai bo nhieu xung */
static constexpr int N_CONFIRM = 3;

/* ============================================================
 * GRID CHO MAT DUONG (rieng voi obstacle grid)
 * Ghi nhan chieu cao tuong doi so voi H_MOUNT
 * Gia tri 128 = mat phang, > 128 = o ga, < 128 = vat can
 * ============================================================ */
static constexpr int ROAD_GRID_N = 100;
static constexpr float ROAD_CELL_M = 0.10f;  /* 10cm moi o (lon hon obstacle grid) */
static constexpr int ROAD_OX = ROAD_GRID_N / 2;
static constexpr int ROAD_OY = ROAD_GRID_N / 2;

/* ============================================================
 * PACKET LIDAR (14 bytes, giong cu)
 * ============================================================ */
constexpr uint8_t ROAD_PKT_HDR = 0xAA;
constexpr uint8_t ROAD_PKT_FTR = 0x55;
constexpr int     ROAD_PKT_LEN = 14;

/* ============================================================
 * CAC STRUCT
 * ============================================================ */

/* Mot diem do tu LiDAR */
struct RawScanPoint {
    float    angle_deg;   /* goc quet tu encoder (-90..+90) */
    uint16_t dist_mm;     /* khoang cach slant (mm) */
    uint32_t ts_ms;
};

/* Ket qua sau khi xu ly hinh hoc */
struct RoadPoint {
    float wx, wy;         /* toa do mat dat (m, he toa do xe) */
    float h_road;         /* chieu cao do lai so voi mat dat (m) */
    float h_delta;        /* = h_road - H_MOUNT (am=o ga, duong=vat can) */
    bool  is_pothole;     /* o ga? */
    bool  is_obstacle;    /* vat can? */
    uint32_t ts_ms;
};

/* Trang thai phuoc (doc tu cam bien hoac uoc tinh) */
struct SuspensionState {
    float compression_mm = 0.0f;  /* do nen phuoc (mm), 0=binh thuong */
    float pitch_add_rad  = 0.0f;  /* goc nghieng them do phuoc (rad) */
};

/* ============================================================
 * ROAD SCANNER CLASS
 * ============================================================ */
class RoadScanner {
public:
    /* Grid chieu cao mat duong: 0=chua biet, 128=phang, >128=o ga, <128=vat can */
    uint8_t road_grid[ROAD_GRID_N * ROAD_GRID_N];

    explicit RoadScanner(std::string dev, int baud = 115200)
        : dev_(std::move(dev)), baud_(baud) {}

    void start();
    void stop();

    /* Cap nhat trang thai phuoc tu nguon ben ngoai
     * (co the lay tu IMU, cam bien hanh trinh, hoac uoc tinh tu gia toc) */
    void set_suspension(float compression_mm) {
        susp_.compression_mm = compression_mm;
        susp_.pitch_add_rad  = compression_mm * PITCH_PER_MM_SUSPENSION;
    }

    /* Lay snapshot road_grid an toan */
    void snapshot_road(uint8_t* dst) const {
        std::memcpy(dst, road_grid, sizeof(road_grid));
    }

    uint32_t pothole_count() const { return pothole_cnt_.load(); }
    uint32_t obstacle_count() const { return obstacle_cnt_.load(); }
    uint32_t point_count() const { return point_cnt_.load(); }

    /* Callback khi phat hien o ga hoac vat can */
    using AlertCb = std::function<void(const RoadPoint&)>;
    void on_pothole(AlertCb cb)  { cb_pothole_  = std::move(cb); }
    void on_obstacle(AlertCb cb) { cb_obstacle_ = std::move(cb); }

private:
    std::string dev_;
    int         baud_;
    int         fd_ = -1;

    std::atomic<bool>     running_{false};
    std::atomic<uint32_t> pothole_cnt_{0};
    std::atomic<uint32_t> obstacle_cnt_{0};
    std::atomic<uint32_t> point_cnt_{0};

    SuspensionState susp_;

    AlertCb cb_pothole_;
    AlertCb cb_obstacle_;

    /* EMA filter state per scan angle bin (360 bins, -180..+180 do) */
    static constexpr int N_ANGLE_BINS = 360;
    float ema_dist_[N_ANGLE_BINS];
    bool  ema_init_[N_ANGLE_BINS];

    /* Confirm buffer: dem bao nhieu mau lien tiep detect o ga */
    int pothole_confirm_[N_ANGLE_BINS];
    int obstacle_confirm_[N_ANGLE_BINS];

    std::thread thr_reader_;
    std::thread thr_proc_;

    /* SPSC queue don gian */
    static constexpr int Q_CAP = 512;
    RawScanPoint q_buf_[Q_CAP];
    std::atomic<int> q_head_{0}, q_tail_{0};
    bool q_push(const RawScanPoint& p);
    bool q_pop(RawScanPoint& p);

    bool uart_open();
    void uart_close();
    bool parse_packet(const uint8_t* b, RawScanPoint& o);
    void reader_loop();
    void process_loop();

    /* XU LY CHINH: tinh hinh hoc + loc nhieu + cap nhat grid */
    RoadPoint compute_road_point(const RawScanPoint& raw);
    void      update_grid(const RoadPoint& rp);
    float     apply_ema(int angle_bin, float dist_m);
    int       angle_to_bin(float angle_deg);
};