# Swerve robot ROS 2 dashboard

Dashboard `aiohttp` + JavaScript thuần, nối trực tiếp mô phỏng ROS 2 Humble qua
một bridge `rclpy`. Không cần npm, React, chart library hay CDN.

## Chạy

Build và chạy factory trước:

```bash
cd /home/sonvv/Desktop/Project/SwerseRobot/SwerseRobot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch swerve_navigation factory.launch.py mode:=mapping
```

Trong terminal khác, dùng cùng `ROS_DOMAIN_ID` nếu bạn đã đổi domain:

```bash
cd /home/sonvv/Desktop/Project/SwerseRobot/web/swerve-dashboard-git/swerve-dashboard
source /opt/ros/humble/setup.bash
source /home/sonvv/Desktop/Project/SwerseRobot/SwerseRobot/install/setup.bash

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
HOST=127.0.0.1 PORT=8080 python server.py
```

Mở <http://127.0.0.1:8080>. Muốn truy cập trong LAN, đặt `HOST=0.0.0.0` và chỉ
mở cổng 8080 trên mạng tin cậy.

## Dữ liệu ROS

| Giao diện | Dùng cho dashboard |
|---|---|
| TF `map → base_footprint` | Pose robot trên occupancy map |
| `/odometry/filtered` | `vx`, `vy`, `omega` từ EKF |
| `/joint_states` | Góc lái đo được và tốc độ bốn bánh |
| `/map` | Occupancy grid; nén RLE và chỉ gửi khi revision đổi |
| `/factory/*/_action/status` | Khóa joystick khi scan/navigate đang chạy |
| `/factory/navigate` | Action goal tạo từ điểm được chọn trên bản đồ |
| `/swerve_controller/command` | Lệnh joystick `SwerveCommand` |

Thứ tự module được ánh xạ theo robot hiện tại: `1=FL`, `2=FR`, `3=RR`,
`4=RL`. Tốc độ joint bánh từ rad/s được đổi sang m/s bằng bán kính 0,0315 m.

Bridge không giữ publisher command khi chỉ xem telemetry. Khi bấm **ENABLE**,
publisher được tạo và phát 20 Hz. Khi STOP, mất WebSocket hoặc mất lệnh quá
0,5 giây, bridge phát zero rồi hủy publisher sau khoảng 0,6 giây. Nhờ vậy action
`/factory/navigate` có thể chạy sau khi nhả joystick mà không bị nhận diện là
nguồn lệnh cạnh tranh. Một thời điểm chỉ một browser được sở hữu manual control.

Hai mode joystick:

- `DYNAMIC_BODY`: `vx/vy` theo `base_footprint`, `omega` là yaw rate.
- `HOLD_BODY`: `vx/vy` theo `odom`, `omega` thay đổi hướng thân mục tiêu.

## Biểu đồ và chọn đích trên bản đồ

Ba biểu đồ lấy mẫu 20 Hz từ `/joint_states`: tốc độ bánh, gia tốc đã lọc và góc
lái. Mỗi canvas có ID `signal-speed`, `signal-accel`, `signal-steering`; các ID
này tách khỏi input `speedLimit` để tránh trình duyệt lấy nhầm phần tử. Có thể
đổi cửa sổ thời gian, ẩn từng module, pause, xóa và xuất CSV.

Trên occupancy map:

- Kéo chuột hoặc cảm ứng để pan; cuộn để zoom quanh vị trí con trỏ.
- Bấm **Theo robot** để đưa robot về tâm và tiếp tục tự bám pose.
- Bấm **Chọn đích**, chọn một ô trống, nhập hướng đích rồi bấm
  **ĐI ĐẾN ĐÍCH**.
- Browser và server cùng kiểm tra điểm nằm trong map, đã scan và không phải vật
  cản. Server sau đó tạo `swerve_msgs/action/NavigateToPose` trong frame `map`.
- Kết quả action thành công hoặc thất bại được hiển thị bằng modal. Nếu chưa có
  `/map`, dashboard báo robot chưa scan và không cho chọn tọa độ.

Mã nguồn được tách theo trách nhiệm:

- `server.py`: ROS subscriptions, TF, action client, command watchdog và
  WebSocket.
- `index.html`: cấu trúc các control và vùng hiển thị.
- `assets/dashboard.js`: telemetry, joystick, biểu đồ, map pan/zoom và action.
- `assets/dashboard.css`: layout, trạng thái điều khiển và modal.

`GET /health` trả trạng thái topic/TF/map, publisher manual và navigation hiện
tại. `WS /ws` phát telemetry 20 Hz; occupancy map chỉ kèm khi client chưa nhận
revision mới. Browser gửi `{type: "navigate", x, y, yaw}` để yêu cầu action.

Dashboard này dành cho mô phỏng. Trước khi dùng với robot thật cần xác thực,
TLS, command lease, E-stop phần cứng và bộ an toàn độc lập.
