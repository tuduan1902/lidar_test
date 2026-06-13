/**
 * road_scanner.cpp
 *
 * Luong xu ly:
 *   reader_loop():  doc UART -> parse packet -> push queue
 *   process_loop(): pop queue -> EMA filter -> tinh hinh hoc
 *                   -> confirm logic -> cap nhat road_grid -> callback
 */
#include "road_scanner.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>

/* ============================================================
 * UART
 * ============================================================ */
bool RoadScanner::uart_open() {
    /* Jetson THS UART: dung stty de set baud, sau do open O_RDONLY */
    char cmd[160];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo 2>/dev/null",
             dev_.c_str(), baud_);
    (void)system(cmd);

    fd_ = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
        printf("[Road] Cannot open %s: %s\n", dev_.c_str(), strerror(errno));
        return false;
    }
    printf("[Road] %s @ %d OK\n", dev_.c_str(), baud_);
    return true;
}

void RoadScanner::uart_close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

/* ============================================================
 * SPSC QUEUE (don gian, 1 producer 1 consumer)
 * ============================================================ */
bool RoadScanner::q_push(const RawScanPoint& p) {
    int h = q_head_.load(std::memory_order_relaxed);
    int n = (h + 1) % Q_CAP;
    if (n == q_tail_.load(std::memory_order_acquire)) return false; /* full */
    q_buf_[h] = p;
    q_head_.store(n, std::memory_order_release);
    return true;
}

bool RoadScanner::q_pop(RawScanPoint& p) {
    int t = q_tail_.load(std::memory_order_relaxed);
    if (t == q_head_.load(std::memory_order_acquire)) return false; /* empty */
    p = q_buf_[t];
    q_tail_.store((t + 1) % Q_CAP, std::memory_order_release);
    return true;
}

/* ============================================================
 * PARSE PACKET 14 BYTES
 * [0]=0xAA [1]=id [2..3]=dist_mm [4..5]=angle*10
 * [6..9]=ts [10..11]=enc_lo [12]=chk [13]=0x55
 * ============================================================ */
bool RoadScanner::parse_packet(const uint8_t* b, RawScanPoint& o) {
    if (b[0] != ROAD_PKT_HDR || b[ROAD_PKT_LEN-1] != ROAD_PKT_FTR)
        return false;

    /* Checksum XOR bytes 1..11 */
    uint8_t chk = 0;
    for (int i = 1; i <= ROAD_PKT_LEN-3; i++) chk ^= b[i];
    if (chk != b[ROAD_PKT_LEN-2]) return false;

    o.dist_mm   = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
    int16_t a10 = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    o.angle_deg = a10 / 10.0f;
    o.ts_ms     = (uint32_t)b[6] | ((uint32_t)b[7]<<8) |
                  ((uint32_t)b[8]<<16) | ((uint32_t)b[9]<<24);
    return true;
}

/* ============================================================
 * THREAD 1: READER
 * Doc byte tu UART, tim header 0xAA, push vao queue
 * ============================================================ */
void RoadScanner::reader_loop() {
    printf("[Road Reader] started\n"); fflush(stdout);

    uint8_t  pkt[ROAD_PKT_LEN], b;
    uint32_t pkts = 0;

    while (running_.load(std::memory_order_relaxed)) {
        /* Tim header */
        if (::read(fd_, &b, 1) != 1) continue;
        if (b != ROAD_PKT_HDR) continue;

        /* Doc 13 bytes con lai */
        pkt[0] = ROAD_PKT_HDR;
        int got = 1;
        while (got < ROAD_PKT_LEN) {
            int r = ::read(fd_, pkt + got, ROAD_PKT_LEN - got);
            if (r > 0) got += r;
        }

        /* Validate footer */
        if (pkt[ROAD_PKT_LEN-1] != ROAD_PKT_FTR) continue;

        RawScanPoint raw;
        if (!parse_packet(pkt, raw)) continue;

        pkts++;
        if (!q_push(raw)) {
            /* Queue day: bo diem cu nhat */
            RawScanPoint dummy;
            q_pop(dummy);
            q_push(raw);
        }
    }
    printf("[Road Reader] stopped pkts=%u\n", pkts);
}

/* ============================================================
 * BO LOC EMA
 * Tra ve dist (m) da duoc loc nhieu
 *
 * angle_bin: index 0..N_ANGLE_BINS-1 tuong ung -180..+180 do
 * dist_m:    gia tri moi nhat (m)
 *
 * EMA: y[n] = alpha*x[n] + (1-alpha)*y[n-1]
 * Voi alpha = EMA_ALPHA = 0.15
 * -> Rung phuoc 5-15Hz bi loc manh, mat duong thay doi cham van theo kip
 * ============================================================ */
int RoadScanner::angle_to_bin(float angle_deg) {
    /* Chuyen -180..+180 sang index 0..359 */
    int bin = (int)(angle_deg + 180.0f) % N_ANGLE_BINS;
    if (bin < 0) bin += N_ANGLE_BINS;
    return bin;
}

float RoadScanner::apply_ema(int bin, float dist_m) {
    if (!ema_init_[bin]) {
        /* Lan dau: khoi tao bang gia tri hien tai */
        ema_dist_[bin] = dist_m;
        ema_init_[bin] = true;
        return dist_m;
    }
    /* EMA update */
    ema_dist_[bin] = EMA_ALPHA * dist_m + (1.0f - EMA_ALPHA) * ema_dist_[bin];
    return ema_dist_[bin];
}

/* ============================================================
 * TINH HINH HOC: tu (angle, dist) -> (wx, wy, h_road)
 *
 * He toa do:
 *   - Goc toa do: tam xe, mat dat
 *   - X: sang phai, Y: phia truoc, Z: len tren
 *
 * LiDAR dat tai (LIDAR_OFFSET_X, LIDAR_OFFSET_Y, H_MOUNT)
 * Tia laser chui xuong voi:
 *   - pitch_total = PITCH_STATIC + pitch_from_suspension
 *   - goc quet ngang = angle_deg (tu encoder)
 *
 * Diem chiem tia laser:
 *   Trong he toa do LiDAR:
 *     lx = dist * cos(pitch) * sin(scan)
 *     ly = dist * cos(pitch) * cos(scan)
 *     lz = -dist * sin(pitch)       <- am vi chui xuong
 *
 *   Chuyen sang he toa do xe (tinh goc pitch):
 *     wx = lx + LIDAR_OFFSET_X
 *     wy = ly * cos(pitch) - lz * sin(pitch) + LIDAR_OFFSET_Y
 *     wz = ly * sin(pitch) + lz * cos(pitch) + H_MOUNT
 *
 *   h_road = wz (chieu cao diem chiem so voi mat dat)
 *   Mat duong phang: h_road ~ 0
 *   O ga: h_road < 0 (am, trong long o)
 *   Vat can: h_road > 0 va > OBSTACLE_HEIGHT_THRESH
 * ============================================================ */
RoadPoint RoadScanner::compute_road_point(const RawScanPoint& raw) {
    RoadPoint rp{};
    rp.ts_ms = raw.ts_ms;

    /* Dist hop le? */
    float dist_m = raw.dist_mm / 1000.0f;
    if (raw.dist_mm == 0xFFFF ||
        dist_m < DIST_MIN_VALID ||
        dist_m > DIST_MAX_VALID) {
        rp.h_road = H_MOUNT; /* khong co thong tin, coi la phang */
        return rp;
    }

    /* Tong goc pitch (goc chui xuong):
     * = goc co dinh + goc them do phuoc nen */
    float pitch = PITCH_STATIC_RAD + susp_.pitch_add_rad;

    /* Goc quet ngang (rad) */
    float scan_rad = raw.angle_deg * (float)M_PI / 180.0f;

    /* ----- Tinh toa do diem chiem trong he LiDAR -----
     * LiDAR huong xuong duoi goc pitch:
     *   - Truc Z LiDAR: chui xuong
     *   - Truc Y LiDAR: phia truoc xe (sau khi xoay pitch)
     *   - Truc X LiDAR: sang phai (khong doi voi pitch)
     */
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);
    float cos_scan  = cosf(scan_rad);
    float sin_scan  = sinf(scan_rad);

    /* Toa do trong he LiDAR (truoc khi chuyen sang he xe) */
    float lx =  dist_m * cos_pitch * sin_scan;
    float ly =  dist_m * cos_pitch * cos_scan;
    float lz = -dist_m * sin_pitch;  /* am vi tia chiu xuong */

    /* Chuyen sang he toa do xe (xoay nguoc pitch quanh truc X) */
    float wx_world =  lx + LIDAR_OFFSET_X;
    float wy_world =  ly * cos_pitch - lz * sin_pitch + LIDAR_OFFSET_Y;
    float wz_world =  ly * sin_pitch + lz * cos_pitch + H_MOUNT;

    rp.wx     = wx_world;
    rp.wy     = wy_world;
    rp.h_road = wz_world;
    rp.h_delta = wz_world; /* ~ 0 neu mat phang, am = o ga, duong = vat can */

    /* Phan loai
     * h_delta am: diem thap hon mat duong -> o ga
     * h_delta duong lon: diem cao hon mat duong -> vat can  */
    rp.is_pothole  = (rp.h_delta < -POTHOLE_DEPTH_THRESH);
    rp.is_obstacle = (rp.h_delta >  OBSTACLE_HEIGHT_THRESH);

    return rp;
}

/* ============================================================
 * CAP NHAT ROAD GRID
 *
 * Grid luu uint8_t:
 *   128       = mat phang (gia tri trung tinh)
 *   < 128     = o ga (cang thap = cang sau)
 *   > 128     = vat can (cang cao = cang ngo)
 *   0         = chua biet / out of range
 *
 * Map: val = 128 + clamp(h_delta * 100, -127, 127)
 *   -> 5cm o ga = 128 - 5 = 123
 *   -> 5cm vat can = 128 + 5 = 133
 *   -> 20cm o ga = 128 - 20 = 108
 * ============================================================ */
void RoadScanner::update_grid(const RoadPoint& rp) {
    /* Chuyen toa do the gioi sang grid index */
    int gx = ROAD_OX + (int)(rp.wx / ROAD_CELL_M);
    int gy = ROAD_OY + (int)(rp.wy / ROAD_CELL_M);

    if (gx < 0 || gx >= ROAD_GRID_N) return;
    if (gy < 0 || gy >= ROAD_GRID_N) return;

    /* Map h_delta (m) -> uint8 */
    int val = 128 + (int)(rp.h_delta * 100.0f);
    if (val < 1)   val = 1;   /* toi thieu 1 (da co thong tin) */
    if (val > 255) val = 255;

    /* Ghi atomic */
    __atomic_store_n(&road_grid[gy * ROAD_GRID_N + gx],
                     (uint8_t)val, __ATOMIC_RELAXED);
}

/* ============================================================
 * THREAD 2: PROCESS LOOP
 * Pop tu queue -> EMA -> hinh hoc -> confirm -> grid -> callback
 * ============================================================ */
void RoadScanner::process_loop() {
    printf("[Road Proc] started\n"); fflush(stdout);

    /* Khoi tao EMA va confirm buffers */
    for (int i = 0; i < N_ANGLE_BINS; i++) {
        ema_dist_[i]        = 0.0f;
        ema_init_[i]        = false;
        pothole_confirm_[i] = 0;
        obstacle_confirm_[i]= 0;
    }

    using namespace std::chrono_literals;

    while (running_.load(std::memory_order_relaxed)) {
        RawScanPoint raw;
        if (!q_pop(raw)) {
            std::this_thread::sleep_for(200us);
            continue;
        }

        /* --- Buoc 1: Bo loc EMA ---
         * Giam nhieu rung phuoc truoc khi xu ly hinh hoc */
        int   bin    = angle_to_bin(raw.angle_deg);
        float dist_m = raw.dist_mm == 0xFFFF
                       ? DIST_MAX_VALID          /* invalid: dung max */
                       : raw.dist_mm / 1000.0f;
        float dist_filtered = apply_ema(bin, dist_m);

        /* Tao raw point da loc */
        RawScanPoint raw_filt = raw;
        raw_filt.dist_mm = (dist_filtered >= DIST_MAX_VALID)
                           ? 0xFFFF
                           : (uint16_t)(dist_filtered * 1000.0f);

        /* --- Buoc 2: Tinh hinh hoc --- */
        RoadPoint rp = compute_road_point(raw_filt);
        point_cnt_.fetch_add(1, std::memory_order_relaxed);

        /* --- Buoc 3: Confirm logic ---
         * Chi bao cao su kien neu N_CONFIRM mau lien tiep cung phat hien
         * Muc dich: loai bo nhieu xung don le (rung manh 1 frame) */
        if (rp.is_pothole) {
            pothole_confirm_[bin]++;
            obstacle_confirm_[bin] = 0;
            if (pothole_confirm_[bin] == N_CONFIRM) {
                /* Xac nhan o ga */
                pothole_cnt_.fetch_add(1, std::memory_order_relaxed);
                if (cb_pothole_) cb_pothole_(rp);
            }
        } else if (rp.is_obstacle) {
            obstacle_confirm_[bin]++;
            pothole_confirm_[bin] = 0;
            if (obstacle_confirm_[bin] == N_CONFIRM) {
                obstacle_cnt_.fetch_add(1, std::memory_order_relaxed);
                if (cb_obstacle_) cb_obstacle_(rp);
            }
        } else {
            /* Mat phang: reset confirm counters */
            pothole_confirm_[bin]  = 0;
            obstacle_confirm_[bin] = 0;
        }

        /* --- Buoc 4: Cap nhat road grid --- */
        update_grid(rp);
    }
    printf("[Road Proc] stopped pts=%u pot=%u obs=%u\n",
           point_cnt_.load(), pothole_cnt_.load(), obstacle_cnt_.load());
}

/* ============================================================
 * START / STOP
 * ============================================================ */
void RoadScanner::start() {
    if (!uart_open()) return;
    memset(road_grid, 0, sizeof(road_grid));
    running_.store(true);
    thr_reader_ = std::thread(&RoadScanner::reader_loop,  this);
    thr_proc_   = std::thread(&RoadScanner::process_loop, this);
}

void RoadScanner::stop() {
    running_.store(false);
    uart_close();  /* unblock read() */
    if (thr_reader_.joinable()) thr_reader_.join();
    if (thr_proc_.joinable())   thr_proc_.join();
}