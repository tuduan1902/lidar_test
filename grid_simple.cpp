#include "grid_simple.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstdio>
#include <cmath>
#include <cstring>

static speed_t to_speed(int b) {
    switch(b) {
        case 9600:   return B9600;
        case 115200: return B115200;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

bool GridSimple::uart_open() {
    fd_ = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) { std::perror(("open " + dev_).c_str()); return false; }

    struct termios t{};
    std::memset(&t, 0, sizeof(t));
    cfsetispeed(&t, to_speed(baud_));
    cfsetospeed(&t, to_speed(baud_));
    t.c_cflag  = CS8 | CLOCAL | CREAD;
    t.c_iflag  = IGNPAR;
    t.c_cc[VMIN]  = 1;
    t.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSAFLUSH, &t);
    std::printf("[UART] %s @ %d  OK\n", dev_.c_str(), baud_);
    return true;
}

void GridSimple::uart_close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

/* Parse packet 10 bytes tu STM32
 * [0]=0xAA [1]=id [2][3]=dist_mm [4..7]=ts [8]=chk [9]=0x55
 */
bool GridSimple::parse(const uint8_t* b, ScanPoint& o) {
    if (b[0] != PKT_HDR)         return false;
    if (b[PKT_LEN - 1] != PKT_FTR) return false;
    o.id      = b[1];
    o.dist_mm = (uint16_t)(b[2] | (b[3] << 8));
    o.ts_ms   = (uint32_t)(b[4] | (b[5]<<8) | (b[6]<<16) | (b[7]<<24));
    return true;
}

void GridSimple::mark(uint16_t dist_mm) {
    if (dist_mm == 0xFFFF || dist_mm < 30) return;
    float dist_m = dist_mm / 1000.0f;
    int gx = GRID_OX;
    int gy = GRID_OY + (int)(dist_m / CELL_M);
    if (gx < 0 || gx >= GRID_N) return;
    if (gy < 0 || gy >= GRID_N) return;
    __atomic_store_n(&data[gy * GRID_N + gx], OBSTACLE, __ATOMIC_RELAXED);
}

/* Doc chinh xac n bytes */
static bool read_exact(int fd, uint8_t* buf, int n) {
    int got = 0;
    while (got < n) {
        ssize_t r = ::read(fd, buf + got, n - got);
        if (r > 0) { got += (int)r; continue; }
        if (r == 0) return false;
        if (errno == EINTR) continue;
        return false;
    }
    return true;
}

void GridSimple::reader_loop() {
    std::printf("[Reader] started\n"); fflush(stdout);

    uint8_t  b;
    uint8_t  buf[PKT_LEN];
    uint32_t raw_bytes  = 0;
    uint32_t found_pkts = 0;
    uint32_t bad_footer = 0;

    while (running_.load(std::memory_order_relaxed)) {

        /* Tim header 0xAA */
        while (running_.load(std::memory_order_relaxed)) {
            if (!read_exact(fd_, &b, 1)) goto done;
            raw_bytes++;
            if (b == PKT_HDR) break;
        }

        /* Doc 9 bytes con lai */
        buf[0] = PKT_HDR;
        if (!read_exact(fd_, buf + 1, PKT_LEN - 1)) goto done;
        raw_bytes += (PKT_LEN - 1);

        /* Kiem tra footer */
        if (buf[PKT_LEN - 1] != PKT_FTR) { bad_footer++; continue; }

        {
            ScanPoint sp;
            if (parse(buf, sp)) {
                found_pkts++;
                if (found_pkts <= 5) {
                    std::printf("[PKT#%u] dist=%u mm  ts=%u\n",
                                found_pkts, sp.dist_mm, sp.ts_ms);
                    fflush(stdout);
                }
                if (!queue_.push(sp)) {
                    ScanPoint dummy; queue_.pop(dummy); queue_.push(sp);
                }
            }
        }
    }

done:
    std::printf("[Reader] stopped  raw=%u pkts=%u bad_footer=%u\n",
                raw_bytes, found_pkts, bad_footer);
}

void GridSimple::updater_loop() {
    std::printf("[Updater] started\n"); fflush(stdout);
    using namespace std::chrono_literals;

    while (running_.load(std::memory_order_relaxed)) {
        ScanPoint sp;
        if (!queue_.pop(sp)) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }
        count_.fetch_add(1, std::memory_order_relaxed);
        mark(sp.dist_mm);
        int gy = GRID_OY + (int)((sp.dist_mm / 1000.0f) / CELL_M);
        if (cb_) cb_(sp.dist_mm, gy);
    }
    std::printf("[Updater] stopped\n");
}

void GridSimple::start() {
    if (!uart_open()) return;
    running_.store(true);
    thr_r_ = std::thread(&GridSimple::reader_loop,  this);
    thr_u_ = std::thread(&GridSimple::updater_loop, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void GridSimple::stop() {
    running_.store(false);
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    if (thr_r_.joinable()) thr_r_.join();
    if (thr_u_.joinable()) thr_u_.join();
}