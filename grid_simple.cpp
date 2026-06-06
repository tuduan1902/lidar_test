#include "grid_simple.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <cstdlib>

bool GridSimple::uart_open(){
    /* Jetson THS UART: dung stty de set baud rate */
    char cmd[128];
    snprintf(cmd, sizeof(cmd),
             "stty -F %s %d raw cs8 -parenb -cstopb -echo",
             dev_.c_str(), baud_);
    if(system(cmd) != 0){
        printf("[UART] stty failed!\n"); return false;
    }

    fd_ = ::open(dev_.c_str(), O_RDONLY|O_NOCTTY);
    if(fd_<0){perror("open"); return false;}

    printf("[UART] %s @ %d OK (fd=%d)\n", dev_.c_str(), baud_, fd_);
    return true;
}

void GridSimple::uart_close(){ if(fd_>=0){::close(fd_);fd_=-1;} }

bool GridSimple::parse(const uint8_t* b, ScanPoint& o){
    if(b[0]!=PKT_HDR||b[PKT_LEN-1]!=PKT_FTR) return false;
    o.id      = b[1];

    o.dist_mm = (uint16_t)b[2] | ((uint16_t)b[3]<<8);
    o.ts_ms   = (uint32_t)b[4]|((uint32_t)b[5]<<8)|
                ((uint32_t)b[6]<<16)|((uint32_t)b[7]<<24);
    return true;
}

void GridSimple::mark(uint16_t dist_mm){
    if(dist_mm==0xFFFF||dist_mm<30) return;
    float dist_m = dist_mm/1000.0f;
    int gx = GRID_OX;
    int gy = GRID_OY + (int)(dist_m/CELL_M);
    if(gx<0||gx>=GRID_N||gy<0||gy>=GRID_N) return;
    __atomic_store_n(&data[gy*GRID_N+gx], OBSTACLE, __ATOMIC_RELAXED);
}

static bool readbyte(int fd, uint8_t& b){
    while(true){
        ssize_t n = ::read(fd, &b, 1);
        if(n==1) return true;
        if(n==0) return false;
        if(errno==EINTR) continue;
        return false;
    }
}

void GridSimple::reader_loop(){
    printf("[Reader] started fd=%d\n", fd_); fflush(stdout);

    uint8_t pkt[PKT_LEN], b;
    uint32_t raw=0, aa=0, pkts=0, bad=0;

    while(running_.load(std::memory_order_relaxed)){
        if(!readbyte(fd_, b)){ break; }
        raw++;
        if(b != PKT_HDR) continue;

        aa++;
        pkt[0] = PKT_HDR;
        bool ok = true;
        for(int i=1; i<PKT_LEN; i++){
            if(!readbyte(fd_, pkt[i])){ok=false;break;}
            raw++;
        }
        if(!ok) break;

        if(aa<=3){
            printf("[AA#%u] ",aa);
            for(int i=0;i<PKT_LEN;i++) printf("%02x ",pkt[i]);
            printf("footer=%s\n",pkt[PKT_LEN-1]==PKT_FTR?"OK":"BAD");
            fflush(stdout);
        }

        if(pkt[PKT_LEN-1]!=PKT_FTR){bad++;continue;}

        ScanPoint sp;
        if(parse(pkt,sp)){
            pkts++;
            if(pkts<=5){
                printf("[PKT#%u] dist=%u mm\n",pkts,sp.dist_mm);
                fflush(stdout);
            }
            if(!queue_.push(sp)){ScanPoint d;queue_.pop(d);queue_.push(sp);}
        }
    }
    printf("[Reader] stopped raw=%u aa=%u pkts=%u bad=%u\n",
           raw,aa,pkts,bad);
}

void GridSimple::updater_loop(){
    printf("[Updater] started\n"); fflush(stdout);
    using namespace std::chrono_literals;
    while(running_.load(std::memory_order_relaxed)){
        ScanPoint sp;
        if(!queue_.pop(sp)){
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }
        count_.fetch_add(1,std::memory_order_relaxed);
        mark(sp.dist_mm);
        int gy=GRID_OY+(int)((sp.dist_mm/1000.0f)/CELL_M);
        if(cb_) cb_(sp.dist_mm,gy);
    }
    printf("[Updater] stopped\n");
}

void GridSimple::start(){
    if(!uart_open()) return;
    running_.store(true);
    thr_r_=std::thread(&GridSimple::reader_loop,this);
    thr_u_=std::thread(&GridSimple::updater_loop,this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void GridSimple::stop(){
    running_.store(false);
    if(fd_>=0){::close(fd_);fd_=-1;}
    if(thr_r_.joinable()) thr_r_.join();
    if(thr_u_.joinable()) thr_u_.join();
}
