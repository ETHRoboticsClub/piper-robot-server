# VR Robot Teleoperation System

This system allows you to control both robot arms using VR controllers with relative positioning.

## Setup

1. **Ensure SSL certificates exist**:
   ```bash
   # If cert.pem and key.pem don't exist, generate them:
   openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -sha256 -days 365 -nodes -subj "/C=US/ST=Test/L=Test/O=Test/OU=Test/CN=localhost"
   ```

2. **Install dependencies** (if not already installed):
   ```bash
   pip install -r requirements.txt
   ```

## Running the System

### Option 1: Full VR Teleoperation (Recommended)

1. **Start the unified VR teleoperation system**:
   ```bash
   python vr_robot_teleop.py
   ```
   This single script includes:
   - WebSocket server for VR controller data
   - Robot arm control (both leader and follower)
   - IK calculations
   - PyBullet visualization

2. **Start the web server** (in another terminal):
   ```bash
   python serve_https.py
   ```

3. **Connect your VR headset**:
   - Open your VR browser
   - Navigate to `https://<your-laptop-ip>:8443`
   - Accept the self-signed certificate warning
   - Enter VR mode

### Option 2: Individual Scripts (Legacy)

If you prefer the modular approach:

1. **Start the controller server**:
   ```bash
   python controller_server.py
   ```

2. **Start the web server**:
   ```bash
   python serve_https.py
   ```

3. **Run keyboard teleoperation** (for testing):
   ```bash
   python lerobot_keyboard_ik.py
   ```

## How to Use

### VR Control Mapping
- **Left Controller Grip Button**: Controls the follower arm (`ttySO100follower`)
- **Right Controller Grip Button**: Controls the leader arm (`ttySO100leader`)

### Control Flow
1. **Grip Activation**: When you press and hold a grip button, the system captures that controller position as the relative origin (0,0,0)
2. **Relative Movement**: Any movement of the controller while the grip is held moves the corresponding robot arm by the same delta
3. **Grip Release**: When you release the grip button, the robot arm stops following that controller

### Coordinate System
- **VR Space**: X=right, Y=up, Z=back (toward user)
- **Robot Space**: X=forward, Y=left, Z=up
- The system automatically converts between coordinate systems

## Configuration

### Adjustable Parameters in `vr_robot_teleop.py`:

```python
VR_TO_ROBOT_SCALE = 1.0      # Scale factor for VR movements
SEND_INTERVAL = 0.05         # Robot command rate (20Hz)
POSITION_SMOOTHING = 0.1     # Movement smoothing (0=none, 1=full)
```

### Robot Ports:
- Leader arm: `/dev/ttySO100leader`
- Follower arm: `/dev/ttySO100follower`

## Troubleshooting

### Common Issues:

1. **WebSocket connection failed**:
   - Ensure `cert.pem` and `key.pem` exist
   - Check that port 8442 is not blocked by firewall
   - Verify the IP address in VR browser matches your laptop

2. **Robot not responding**:
   - Check USB connections to robot arms
   - Verify port names (`/dev/ttySO100leader`, `/dev/ttySO100follower`)
   - Ensure robot arms are powered on

3. **PyBullet visualization not working**:
   - Check URDF file exists: `URDF/SO_5DOF_ARM100_8j/urdf/so100.urdf`
   - Try running in headless mode if GUI fails

4. **VR controllers not detected**:
   - Ensure VR controllers are paired and tracking
   - Check browser console for WebSocket connection messages
   - Verify you're in VR mode, not just viewing on desktop

### Debug Mode:
The system logs detailed information to help with debugging. Check the console output for:
- VR client connections
- Grip activation/release events
- Robot command sending
- IK computation warnings

## Safety Notes

- **Always be ready to release grip buttons** to stop robot movement
- **Start with small movements** to test coordinate mapping
- **Ensure robot workspace is clear** before testing
- **The system automatically disables torque on shutdown** for safety

## File Structure

```
vr_so100_teleoperation/
├── vr_robot_teleop.py      # Main VR teleoperation script (NEW)
├── controller_server.py    # Legacy WebSocket server
├── lerobot_keyboard_ik.py  # Legacy keyboard control
├── serve_https.py          # HTTPS web server
├── app.js                  # VR web app (UPDATED)
├── index.html              # VR interface
├── cert.pem / key.pem      # SSL certificates
└── URDF/                   # Robot models
``` 