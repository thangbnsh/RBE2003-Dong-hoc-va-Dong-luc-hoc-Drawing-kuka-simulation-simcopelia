# 📘 Hướng dẫn sử dụng project xử lý ảnh MATLAB & VREP

## 📌 Các bước thực hiện:

---

### ✅ Bước 1: Kiểm tra tên và định dạng file ảnh
- Đảm bảo ảnh có định dạng hợp lệ: `.jpg`, `.png`, `.bmp`, v.v.
- Tên ảnh **không chứa dấu cách** hoặc ký tự đặc biệt.  
  *(Ví dụ: `anh1.jpg`, `hinh_mau.png` là hợp lệ)*

---

### ✅ Bước 2: Di chuyển ảnh vào thư mục chứa code
- Copy ảnh vào **cùng thư mục** với file `.m` (code MATLAB).
- Ví dụ: nếu thư mục chứa code là `bao-cao-lam-quen-matlab-vrep`, hãy đặt ảnh vào thư mục này.

---

### ✅ Bước 3: Thay đổi tên ảnh trong code MATLAB
- Mở file `.m` và tìm dòng lệnh đọc ảnh:

```matlab
RGB = imread('ten_anh.jpg');


---

### ✅ Bước 4: Chạy V-REP và nạp code MATLAB
- Mở phần mềm V-REP (hoặc CoppeliaSim) trước và bấm ▶ Run Simulation.

- Sau đó mở MATLAB và chạy file .m.

- MATLAB sẽ kết nối tới V-REP và gửi lệnh điều khiển robot dựa trên kết quả xử lý ảnh.
