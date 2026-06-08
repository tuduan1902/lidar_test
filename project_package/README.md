# LiDAR + Ultrasonic Sensor Fusion

## Mô tả

Ứng dụng này hiển thị bảng occupancy grid 3m x 3m từ dữ liệu LiDAR và 4 sensor siêu âm.

## Nội dung thư mục

- `main_combined_filtered_values.cpp`: chương trình chính đọc LiDAR + US và vẽ lưới.
- `grid_manager.hpp`: cấu hình lưới, tham số sensor, packet LiDAR, queue lock-free.
- `grid_manager.cpp`: triển khai logic bản đồ, decay và helper.
- `lidar_with_encoder.c`: firmware STM32 mẫu cho LiDAR + encoder.
- `lidar_with_encoder.h`: header firmware STM32.
- `ARCHITECTURE.md`: giải thích kiến trúc và luồng dữ liệu.

## Build trên Jetson/Linux

```bash
cd /home/tuduan/lidar_test/project_package
 g++ -std=c++17 -O2 -lpthread main_combined_filtered_values.cpp -o lidar_us_filtered_values
```

## Chạy

```bash
./lidar_us_filtered_values /dev/ttyTHS1 /dev/ttyUSB0
```

Thay `/dev/ttyTHS1` và `/dev/ttyUSB0` bằng thiết bị thực tế của bạn.

## Lưu ý

- `lidar_with_encoder.c` là mã STM32, không biên dịch trên Linux trực tiếp.
- Nếu bạn muốn chạy chương trình theo cách nhẹ hơn, chỉ cần dùng `main_combined_filtered_values.cpp` và `grid_manager.hpp`.

## Mở rộng

- Điều chỉnh `DECAY_MS` và `DECAY_STEP` trong `grid_manager.hpp` để thay đổi thời gian giữ vết chướng ngại.
- Thay đổi `US_MOUNTS` nếu vị trí sensor thực tế khác so với cấu hình hiện tại.
