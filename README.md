# Swerve robot dashboard (simulation)

Minimal Python aiohttp server + vanilla JS dashboard. No npm, React, chart libraries, CDN, or Internet required. WebSocket telemetry at 20 Hz; command transmission at 10 Hz.

## Install

```bash
sudo apt update
sudo apt install -y python3 python3-venv
cd swerve-dashboard
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python server.py
```

Visit http://localhost:8080 from server computer, or http://SERVER_LAN_IP:8080 from another computer on the same network. Find the IP with `hostname -I`. Allow TCP 8080 on the local firewall if needed.

## Architecture

- `GET /` static dashboard
- `GET /health` health check
- `WS /ws` JSON commands and telemetry
- Kinematics: vx/vy in robot frame, omega in rad/s; wheel position `(x,y)` in meters, `v_wheel=(vx-omega*y,vy+omega*x)`.
- `steering` is target wheel angle in degrees, not measured actuator feedback.
- `accel` is simulated speed derivative, not measured sensor acceleration.
- Map is local dead-reckoned XY, **not** a geographic map or GPS.
- Single shared command owner; multiple clients can view, but multiple simultaneous controllers will conflict.

## Safety

**Simulation only. Do not connect this code directly to motor drivers.** Before physical deployment add authentication/TLS, a single-controller lease, actuator limits, wheel steering feedback, a hardware emergency-stop circuit, a safety MCU, command expiry, and independent loss-of-link failsafe. The dashboard STOP button is not a hardware E-stop.

## v1.1
Heading hold: bật checkbox, chỉnh Target heading. Q/E quay ở chế độ yaw rate. Biểu đồ tách FL/FR/RL/RR. Khởi động lại server và Ctrl+Shift+R.
\n## v1.2 Telemetry Studio\n- Đồ thị 4 kênh chung trục, có legend bật/tắt, tooltip hover, trục X thời gian thực và Y có tick giá trị.\n- Lưu lịch sử trên RAM trình duyệt tối đa 60 phút, lựa chọn cửa sổ thời gian, pause hiển thị, xóa lịch sử và xuất CSV.\n- Không dùng thư viện chart ngoài, chạy offline trên LAN.\n- Khi thay file cần restart server và hard refresh Ctrl+Shift+R.\n

sonvv
