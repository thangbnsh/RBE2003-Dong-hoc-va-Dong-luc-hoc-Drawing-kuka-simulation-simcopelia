# 🚗 Robot Tự Hành Điều Khiển Bằng Xử Lý Ảnh

## 🛠 Hướng dẫn sử dụng

### ✅ Bước 1: Tải phần mềm cần thiết

- MATLAB (có Toolboxes: Image Processing, Instrument Control)
- V-REP / CoppeliaSim
- Add-on V-REP Remote API cho MATLAB (đã được cấu hình sẵn nếu dùng code này)

---

### ✅ Bước 2: Thêm ảnh vào thư mục

- Copy ảnh `.jpg` bạn muốn xử lý và điều khiển robot vào thư mục chứa mã nguồn MATLAB.

---

### ✅ Bước 3: Thay đổi tên ảnh trong code MATLAB

- Mở file `.m` và tìm dòng lệnh đọc ảnh:

```matlab
RGB = imread('ten_anh.jpg');
```
---

### ✅ Bước 4: Khởi động mô phỏng và chạy mã

- Mở phần mềm V-REP (CoppeliaSim) và nhấn nút ▶ để chạy mô phỏng trước.

- Sau đó mới chạy mã trên MATLAB.
