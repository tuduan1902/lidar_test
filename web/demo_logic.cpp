/* demo_logic.cpp
 * Mo phong logic phan loai O GA / VAT CAN va dieu kien wy cua road_scanner
 * + main_web.cpp, voi cac vi du khoang cach do duoc cu the.
 * KHONG dung phan cung - chi tinh toan thuan tuy de minh hoa.
 */
#include "road_scanner.hpp"
#include <cstdio>
#include <cmath>

struct Case {
    const char* name;
    float baseline_dist; /* gia tri tu calib (m), gia dinh */
    float ema_dist;       /* dist_ema_m do duoc o sample hien tai (m) */
};

int main() {
    Case cases[] = {
        {"1) Baseline - mat phang",            1.20f, 1.20f},
        {"2) O ga nho  (lo sau ~10cm)",         1.20f, 1.30f},
        {"3) Vat can nho (go cao ~10cm)",       1.20f, 1.10f},
        {"4) O ga sau  (~80cm, ham/cong)",      1.20f, 2.00f},
        {"5) Vat can gan ngay duoi xe",         1.20f, 0.30f},
        {"6) Dao dong nho trong nguong (~2cm)", 1.20f, 1.22f},
    };

    printf("%-32s %8s %8s %9s %8s %9s %-10s %-6s\n",
           "Truong hop", "exp(m)", "ema(m)", "delta(m)", "wy(m)",
           "trong_range", "phan_loai", "ve_map");
    printf("------------------------------------------------------------------------------------\n");

    for (auto& c : cases) {
        /* Tinh baseline_pitch_rad_ giong nhu auto-calib trong road_scanner.cpp */
        float ratio = H_MOUNT / c.baseline_dist;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;
        float pitch = asinf(ratio);
        if (pitch < PITCH_MIN_RAD) pitch = PITCH_MIN_RAD;
        if (pitch > PITCH_MAX_RAD) pitch = PITCH_MAX_RAD;

        /* expected_dist (giong RoadScanner::expected_dist) */
        float sp = sinf(pitch);
        if (sp < 0.01f) sp = 0.01f;
        float dist_exp = H_MOUNT / sp;

        float delta = dist_exp - c.ema_dist;

        /* Phan loai (giong process_loop, bo qua N_CONFIRM vi day la 1 sample don) */
        const char* kind;
        if (delta < -POTHOLE_THRESH_M)       kind = "O GA";
        else if (delta > OBSTACLE_THRESH_M)  kind = "VAT CAN";
        else                                  kind = "khong su kien";

        /* wy nhu trong on_pothole/on_obstacle cua main_web.cpp */
        float wy = c.ema_dist * cosf(pitch) + LIDAR_OY;
        bool in_range = (wy > 0.5f && wy < 1.75f);

        const char* drawn = "KHONG";
        if (in_range && (kind[0] == 'O' && kind[1] == ' ')) drawn = "CO (hong)";
        else if (in_range && kind[0] == 'V')                drawn = "CO (do)";

        printf("%-32s %8.3f %8.3f %9.3f %8.3f %9s %-10s %-6s\n",
               c.name, dist_exp, c.ema_dist, delta, wy,
               in_range ? "co" : "KHONG", kind, drawn);
    }

    printf("\n--- Kiem tra can duoi cua dieu kien wy>0.5 ---\n");
    printf("Voi DIST_MIN_M=%.2f, LIDAR_OY=%.2f, pitch trong [%.1f,%.1f] deg:\n",
           DIST_MIN_M, LIDAR_OY, PITCH_MIN_RAD*180/M_PI, PITCH_MAX_RAD*180/M_PI);
    for (float pdeg : {5.0f, 30.0f, 56.4f, 80.0f}) {
        float p = pdeg * (float)M_PI / 180.0f;
        float wy_min = DIST_MIN_M * cosf(p) + LIDAR_OY;
        printf("  pitch=%5.1f deg -> wy_min = DIST_MIN_M*cos(pitch)+LIDAR_OY = %.3f m  (luon > 0.5)\n",
               pdeg, wy_min);
    }
    return 0;
}