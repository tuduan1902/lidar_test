#include "grid_xy.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

/* ---- UART: dung stty (Jetson THS UART) ---- */
bool GridXY::uart_open() {
    char cmd[128];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo",
             dev_.c_str(), baud_);
    if (system(cmd) != 0) { printf("[UART] stty failed\n"); return false; }
    fd_ = ::open(dev_.c_str(), O_RDONLY|O_NOCTTY);
    if (fd_ < 0) { perror("open"); return false; }
    printf("[UART] %s @ %d OK\n", dev_.c_str(), baud_);
    return true;
}
void GridXY::uart_close(){ if(fd_>=0){::close(fd_);fd_=-1;} }

/* ---- Parse packet 14 bytes ---- */
bool GridXY::parse(const uint8_t* b, ScanPoint& o) {
    if (b[0]!=PKT_HDR || b[PKT_LEN-1]!=PKT_FTR) return false;
    uint8_t chk=0;
    for(int i=1;i<=PKT_LEN-3;i++) chk^=b[i];
    if(chk!=b[PKT_LEN-2]) return false;

    o.id      = b[1];
    o.dist_mm = (uint16_t)b[2] | ((uint16_t)b[3]<<8);
    int16_t a = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5]<<8));
    o.angle_deg = a / 10.0f;
    o.ts_ms   = (uint32_t)b[6]|((uint32_t)b[7]<<8)|
                ((uint32_t)b[8]<<16)|((uint32_t)b[9]<<24);
    return true;
}

/* ---- Tinh (wx, wy) the gioi ---- */
void GridXY::to_world(const ScanPoint& sp, float& wx, float& wy) {
    float rad   = (MOUNT_DEG + sp.angle_deg) * (float)M_PI / 180.0f;
    float dist_m = sp.dist_mm / 1000.0f;
    wx = LIDAR_OX + dist_m * std::sin(rad);
    wy = LIDAR_OY + dist_m * std::cos(rad);
}

/* ---- Ghi obstacle ---- */
void GridXY::mark(float wx, float wy) {
    int gx = GRID_OX + (int)(wx / CELL_M);
    int gy = GRID_OY + (int)(wy / CELL_M);
    if(gx<0||gx>=GRID_N||gy<0||gy>=GRID_N) return;
    __atomic_store_n(&data[gy*GRID_N+gx], OBSTACLE, __ATOMIC_RELAXED);
}

/* ---- Doc 1 byte ---- */
static bool readbyte(int fd, uint8_t& b){
    while(true){
        ssize_t n=::read(fd,&b,1);
        if(n==1) return true;
        if(n==0) return false;
        if(errno==EINTR) continue;
        return false;
    }
}

/* ---- Thread 1: Reader ---- */
void GridXY::reader_loop(){
    printf("[Reader] started\n"); fflush(stdout);
    uint8_t pkt[PKT_LEN], b;
    uint32_t raw=0,aa=0,pkts=0,bad=0;

    while(running_.load(std::memory_order_relaxed)){
        if(!readbyte(fd_,b)) break;
        raw++;
        if(b!=PKT_HDR) continue;
        aa++;
        pkt[0]=PKT_HDR;
        bool ok=true;
        for(int i=1;i<PKT_LEN;i++){
            if(!readbyte(fd_,pkt[i])){ok=false;break;}
            raw++;
        }
        if(!ok) break;
        if(pkt[PKT_LEN-1]!=PKT_FTR){bad++;continue;}
        ScanPoint sp;
        if(parse(pkt,sp)){
            pkts++;
            if(pkts<=3){
                printf("[PKT#%u] dist=%umm angle=%.1fdeg\n",
                       pkts,sp.dist_mm,sp.angle_deg);
                fflush(stdout);
            }
            if(!queue_.push(sp)){ScanPoint d;queue_.pop(d);queue_.push(sp);}
        }
    }
    printf("[Reader] stopped raw=%u aa=%u pkts=%u bad=%u\n",raw,aa,pkts,bad);
}

/* ---- Thread 2: Updater ---- */
void GridXY::updater_loop(){
    printf("[Updater] started\n"); fflush(stdout);
    using namespace std::chrono_literals;
    while(running_.load(std::memory_order_relaxed)){
        ScanPoint sp;
        if(!queue_.pop(sp)){
            std::this_thread::sleep_for(200us); continue;
        }
        count_.fetch_add(1,std::memory_order_relaxed);
        if(sp.dist_mm!=0xFFFF && sp.dist_mm>=30){
            float wx,wy;
            to_world(sp,wx,wy);
            mark(wx,wy);
            if(cb_) cb_(sp,wx,wy);
        }
    }
    printf("[Updater] stopped\n");
}

/* ---- Thread 3: Decay ---- */
void GridXY::decay_loop(){
    printf("[Decay] started\n"); fflush(stdout);
    while(running_.load(std::memory_order_relaxed)){
        std::this_thread::sleep_for(std::chrono::milliseconds(DECAY_MS));
        for(int i=0;i<GRID_N*GRID_N;i++){
            uint8_t v=__atomic_load_n(&data[i],__ATOMIC_RELAXED);
            if(v==0) continue;
            uint8_t nv=(v>DECAY_STEP)?(v-DECAY_STEP):0;
            __atomic_store_n(&data[i],nv,__ATOMIC_RELAXED);
        }
    }
    printf("[Decay] stopped\n");
}

void GridXY::start(){
    if(!uart_open()) return;
    running_.store(true);
    thr_r_=std::thread(&GridXY::reader_loop,  this);
    thr_u_=std::thread(&GridXY::updater_loop, this);
    thr_d_=std::thread(&GridXY::decay_loop,   this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
void GridXY::stop(){
    running_.store(false);
    if(fd_>=0){::close(fd_);fd_=-1;}
    if(thr_r_.joinable()) thr_r_.join();
    if(thr_u_.joinable()) thr_u_.join();
    if(thr_d_.joinable()) thr_d_.join();
}