# Xây dựng một hệ thống ROS bao gồm các node thực hiện việc
# giả lập đo khoảng cách và cảnh báo khi vật thể quá gần.

## 1. Package ROS: `distance_warning`

## 2. Parameters (Parameter Server)

- Tạo tham số `threshold` lưu giá trị ngưỡng khoảng cách cảnh báo, mặc định 0.5 (đơn vị mét).
- Các node lấy và cập nhật giá trị tham số này qua Parameter Server.

## 3. Publisher / Subscriber

### Node `distance_publisher`:
- Giả lập đo khoảng cách bằng cách phát ngẫu nhiên giá trị float từ 0.1 đến 1.5 mét.
- Phát dữ liệu này lên topic `distance_topic` với message type `std_msgs/Float32`.
- Tần suất phát 1 Hz.

### Node `distance_listener`:
- Subcribe topic `distance_topic`.
- Lấy tham số `threshold` từ Parameter Server.
- Khi nhận khoảng cách nhỏ hơn `threshold`, in ra cảnh báo `"Warning: Object too close!"`.
- Hiển thị giá trị khoảng cách nhận được.

## 4. Service

- Tạo service `set_threshold` (ví dụ dùng `std_srvs/SetBool` hoặc tự định nghĩa).

### Node service server (`set_threshold_service`):
- Nhận yêu cầu thay đổi giá trị ngưỡng cảnh báo (`threshold`) trong quá trình chạy.
- Nếu request có `data: true` thì giảm ngưỡng 0.1 (tối thiểu 0.1).
- Nếu request có `data: false` thì tăng ngưỡng 0.1 (tối đa 1.5).
- Cập nhật giá trị mới lên Parameter Server.
- Trả về kết quả thành công và thông báo giá trị ngưỡng mới.

## 5. Action

- Định nghĩa một action `CheckDistance.action` với cấu trúc:

  # Goal
  float32 distance_to_check
  ---
  # Result
  bool is_safe
  ---
  # Feedback
  string feedback_msg

### Node (`distance_action_server`):
- Nhận goal chứa khoảng cách cần kiểm tra.
- Giả lập xử lý từng bước với feedback gửi về client (ví dụ 5 bước delay).
- Kiểm tra khoảng cách với threshold hiện tại lấy từ Parameter Server.
- Trả về kết quả is_safe = True nếu khoảng cách >= ngưỡng, ngược lại False.

### Node (`distance_action_client`):
- Gửi goal với khoảng cách muốn kiểm tra.
- Nhận feedback và kết quả từ server.
- In thông báo kết quả kiểm tra an toàn hay cảnh báo.
