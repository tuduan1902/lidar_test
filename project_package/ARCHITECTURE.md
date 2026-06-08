# LiDAR + Ultrasonic Sensor Fusion Project

## Mục đích

Dự án này thu thập dữ liệu từ:
- LiDAR VB22A qua UART
- 4 sensor siêu âm qua UART (dữ liệu JSON)

và hiển thị bản đồ chiếm chỗ (occupancy grid) 3m x 3m dưới dạng ASCII trong terminal. Mục tiêu là thấy vùng chướng ngại do LiDAR và siêu âm phát hiện.

## Thư mục chạy

`project_package/`
- `main_combined_filtered_values.cpp`
- `grid_manager.hpp`
- `grid_manager.cpp`
- `lidar_with_encoder.c`
- `lidar_with_encoder.h`
- `ARCHITECTURE.md`

Đây là bộ đầy đủ để build ứng dụng hiển thị LiDAR + US cùng với mã STM32 tạo packet LiDAR.

## Cấu trúc chính

### `grid_manager.hpp`

Chứa cấu hình và thông số bản đồ:
- `GRID_N = 100`: kích thước lưới 100x100
- `CELL_M = 0.03f`: mỗi ô tương đương 3cm
- `GRID_OX`, `GRID_OY`: tâm xe tại ô (50,50)
- `THRESH_FAINT`, `THRESH_STRONG`: ngưỡng hiển thị `.` và `*`
- `DECAY_STEP = 40`, `DECAY_MS = 150`: cơ chế suy giảm giá trị chướng ngại
- vị trí mount của LiDAR và 4 sensor US
- định nghĩa packet LiDAR và cấu trúc dữ liệu
- template `SpscQ<T, CAP>` dùng để truyền dữ liệu giữa các thread

### `grid_manager.cpp`

Triển khai các lớp và hàm sử dụng chung cho bản đồ và quản lý sensor. Nội dung chính:
- cơ chế `FilteredMap` / `Grid` để lưu giá trị cell, `hits` và decay
- các vòng lặp đọc và xử lý dữ liệu sensor nếu dùng mã gốc `main_combined.cpp`
- hàm vẽ lưới và logic in terminal
- ví dụ cách kết nối LiDAR và US trong một thread-based manager

### `main_combined_filtered_values.cpp`

Chứa toàn bộ ứng dụng:
- đọc LiDAR từ `lidar_dev`
- đọc ultrasonic JSON từ `us_dev`
- parse dữ liệu vào đối tượng `LidarPoint` và `UsPoint`
- biểu diễn mọi đo đạc trên `FilteredMap`
- cập nhật `hits` cho mỗi ô và gán nguồn: `L` cho LiDAR, `1..4` cho US
- decay dữ liệu mỗi 150ms để loại bỏ điểm cũ
- vẽ toàn bộ lưới 100x100 lên terminal

## Luồng dữ liệu

1. `lidar_reader_loop()`:
   - đọc packet LiDAR 14 byte
   - kiểm tra header, checksum, footer
   - parse khoảng cách và góc
   - lưu vào queue `lidar_q_`

2. `us_reader_loop()`:
   - đọc dòng JSON từ UART
   - parse 4 giá trị siêu âm
   - lưu vào queue `us_q_`

3. `updater_loop()`:
   - lấy các điểm từ queue
   - tính toạ độ `wx, wy` dựa trên góc và vị trí gắn cảm biến
   - gọi `mark_xy()` để cập nhật grid
   - mỗi điểm LiDAR dùng `src=1`, mỗi US dùng `src=2..5`

4. `decay_loop()`:
   - mỗi 150ms giảm `hits` mỗi ô
   - khi `hits` thấp hơn ngưỡng hiển thị, ô trở về trống

## Chiều không gian và góc

- LiDAR gắn cách tâm xe 30cm về phía trước
- phương đứng trước xe là trục Y dương
- `angle_deg` từ packet LiDAR là góc quét so với hướng trước
- `wx = LIDAR_OX + dist * sin(rad)`
- `wy = LIDAR_OY + dist * cos(rad)`

US1/US2 ở bên trái, US3/US4 ở bên phải, hướng đo 90° hoặc 270° để ra bên.

## Build và chạy

Từ thư mục `project_package` hoặc điều chỉnh đường dẫn file:

```bash
cd /home/tuduan/lidar_test/project_package
 g++ -std=c++17 -O2 -lpthread main_combined_filtered_values.cpp -o lidar_us_filtered_values
 ./lidar_us_filtered_values /dev/ttyTHS1 /dev/ttyUSB0
```

Nếu bạn chạy trong thư mục gốc, chỉ cần đường dẫn đúng tới `main_combined_filtered_values.cpp`.

## Khi nào cần thay đổi

- Nếu muốn phản ứng nhanh hơn với vật cản di chuyển, giảm `DECAY_MS` hoặc tăng `DECAY_STEP`.
- Nếu muốn chỉ giữ điểm mới nhất thay vì vùng tích luỹ, cần sửa `FilteredMap` hoặc bỏ decay state.
- Nếu cần hiển thị chỉ `L` mới nhất, có thể thay `sources` và `hits` để không lưu giữ nhiều điểm cũ.

## Gợi ý mở rộng

- thêm đồ hoạ đơn giản Qt / SDL để thấy bản đồ rõ hơn
- thêm log timestamp để đo tốc độ đọc cảm biến
- chuyển `US_MOUNTS` thành cấu hình dễ thay đổi nếu vị trí cảm biến thay đổi trên xe
- mở rộng `grid_manager.hpp` thành module dùng lại cho nhiều chế độ

## Mã STM32 LiDAR

### `lidar_with_encoder.c` và `lidar_with_encoder.h`

Đây là firmware mẫu cho STM32 dùng encoder và LiDAR:
- đọc encoder tuyệt đối để xác định góc quay của LiDAR
- tạo packet LiDAR nhị phân 14 byte cho máy chủ Jetson
- header `0xAA`, footer `0x55`, checksum XOR
- gửi khoảng cách và góc `angle_deg * 10` cùng timestamp

Firmware này không chạy trực tiếp trên desktop, nhưng là nguồn tạo dữ liệu LiDAR hợp lệ cho chương trình `main_combined_filtered_values.cpp`.

## Build STM32

Nếu muốn biên dịch firmware STM32, cần thêm toolchain ARM/CMSIS và cấu hình project tương ứng. Tài liệu hiện tại chỉ tập trung trên phần ứng dụng Jetson/Linux.
