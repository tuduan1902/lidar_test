/**
 * grid_simple.cpp
 */
#include "grid_simple.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstdio>
#include <cmath>

/* ---- UART ---- */
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
    fd_ = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY | O_SYNC);
    if (fd_ < 0) { std::perror(("open " + dev_).c_str()); return false; }

    struct termios t{};
    tcgetattr(fd_, &t);
    speed_t sp = to_speed(baud_);
    cfsetispeed(&t, sp); cfsetospeed(&t, sp);
    t.c_cflag  = CS8 | CLOCAL | CREAD;
    t.c_iflag  = 0; t.c_oflag = 0; t.c_lflag = 0;
    t.c_cc[VMIN] = 1; t.c_cc[VTIME] = 10;
    tcsetattr(fd_, TCSANOW, &t);
    tcflush(fd_, TCIFLUSH);
    std::printf("[UART] %s @ %d  OK\n", dev_.c_str(), baud_);
    return true;
}

void GridSimple::uart_close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

/* ---- Parse packet 10 bytes ---- */
bool GridSimple::parse(const uint8_t* b, ScanPoint& o) {
    if (b[0] != PKT_HDR || b[9] != PKT_FTR) return false;
    uint8_t chk = 0;
    for (int i = 1; i <= 7; i++) chk ^= b[i];
    if (chk != b[8]) return false;

    o.id      = b[1];
    o.dist_mm = (uint16_t)(b[2] | (b[3] << 8));
    o.ts_ms   = (uint32_t)(b[4] | (b[5]<<8) | (b[6]<<16) | (b[7]<<24));
    return true;
}

/* ---- Ve diem len grid ----
 * Khong co angle: LiDAR huong thang (doc theo xe).
 * Gia su LiDAR cam tay, huong thang ve phia truoc.
 * dist_mm -> Y cells tu tam; X = 0 (truong giua).
 *
 * Neu ban muon test nhieu huong: cam nghieng tay la X se lech,
 * nhung hien tai ta cu ve thang Y truoc cho don gian.
 */
void GridSimple::mark(uint16_t dist_mm) {
    if (dist_mm == 0xFFFF || dist_mm < 30) return;  /* invalid / nhieu */

    float dist_m = dist_mm / 1000.0f;

    /* Y tu tam ve phia truoc (gx=GRID_OX, gy tang khi dist tang) */
    int gx = GRID_OX;
    int gy = GRID_OY + (int)(dist_m / CELL_M);

    if (gx < 0 || gx >= GRID_N) return;
    if (gy < 0 || gy >= GRID_N) return;

    __atomic_store_n(&data[gy * GRID_N + gx], OBSTACLE, __ATOMIC_RELAXED);
}

/* ---- Thread 1: Doc UART ---- */
void GridSimple::reader_loop() {
    std::printf("[Reader] started\n");
    uint8_t win[PKT_LEN]{};
    int filled = 0;
    uint8_t b;

    while (running_.load(std::memory_order_relaxed)) {
        ssize_t n = ::read(fd_, &b, 1);
        if (n <= 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            std::perror("[Reader] read"); break;
        }
        if (filled < PKT_LEN) win[filled++] = b;
        else { std::memmove(win, win+1, PKT_LEN-1); win[PKT_LEN-1] = b; }

        if (filled < PKT_LEN) continue;
        if (win[0] != PKT_HDR)  continue;

        ScanPoint sp;
        if (!parse(win, sp)) continue;

        if (!queue_.push(sp)) {
            ScanPoint dummy; queue_.pop(dummy); queue_.push(sp);
        }
        filled = 0;
    }
    std::printf("[Reader] stopped\n");
}

/* ---- Thread 2: Cap nhat grid ---- */
void GridSimple::updater_loop() {
    std::printf("[Updater] started\n");
    using namespace std::chrono_literals;

    while (running_.load(std::memory_order_relaxed)) {
        ScanPoint sp;
        if (!queue_.pop(sp)) { std::this_thread::sleep_for(200us); continue; }

        mark(sp.dist_mm);
        count_.fetch_add(1, std::memory_order_relaxed);

        /* Tinh grid Y de hien thi */
        int gy = GRID_OY + (int)((sp.dist_mm / 1000.0f) / CELL_M);
        if (cb_) cb_(sp.dist_mm, gy);
    }
    std::printf("[Updater] stopped\n");
}

/* ---- Start / Stop ---- */
void GridSimple::start() {
    if (!uart_open()) return;
    running_.store(true);
    thr_r_ = std::thread(&GridSimple::reader_loop,  this);
    thr_u_ = std::thread(&GridSimple::updater_loop, this);
}

void GridSimple::stop() {
    running_.store(false);
    if (thr_r_.joinable()) thr_r_.join();
    if (thr_u_.joinable()) thr_u_.join();
    uart_close();
}