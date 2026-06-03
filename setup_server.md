# Control Server Setup — Tinker Board + Cloudflare Tunnel

## Architecture

```
Browser / Robot (SIM)
  │
  │  https://app.saintwings.xyz      (Mission Planner web UI)
  │  wss://robot.saintwings.xyz      (WebSocket control server)
  ▼
Cloudflare Edge  ── free tunnel, TLS terminated here, no port-forwarding needed
  │
  │  http://localhost:5000           (Flask → mission_planner_v2.html)
  │  ws://localhost:8765             (websockets → robot/web clients)
  ▼
Tinker Board  (user: linaro, project: ~/robot_server/)
  ├─ control_server.py   WebSocket :8765 + Flask :5000
  └─ cloudflared         tunnel daemon → Cloudflare edge
```

## System Info

| Item | Value |
|------|-------|
| Tinker Board user | `linaro` |
| Project path | `/home/linaro/robot_server/` |
| Python venv | `/home/linaro/robot_server/venv/` |
| Tunnel name | `robot-server` |
| Tunnel ID | `6521735f-aedc-4cdd-9eaa-4815937703c1` |
| cloudflared config | `/etc/cloudflared/config.yml` |
| Web UI URL | `https://app.saintwings.xyz` |
| WebSocket URL | `wss://robot.saintwings.xyz` |
| Admin panel | `https://app.saintwings.xyz/admin` |

---

## Part 1 — Cloudflare Domain Setup

### 1.1 Add domain to Cloudflare

1. Go to [dash.cloudflare.com](https://dash.cloudflare.com) → **Add a Site**
2. Enter `saintwings.xyz` → choose **Free** plan → Continue

### 1.2 Change nameservers on Namecheap

Cloudflare gives two nameservers (e.g. `aida.ns.cloudflare.com`).

1. Log in to [namecheap.com](https://namecheap.com) → **Domain List** → **Manage**
2. **Nameservers** → **Custom DNS** → enter both Cloudflare nameservers → ✓ Save

> Propagation takes 5–30 min. Cloudflare emails when active.

### 1.3 Enable WebSocket in Cloudflare

Cloudflare dashboard → **saintwings.xyz** → **Network** → **WebSockets** → toggle **On**

---

## Part 2 — Control Server on Tinker Board

### 2.1 Copy project files

From dev machine:
```bash
rsync -av --exclude='.pio' --exclude='__pycache__' --exclude='venv' \
  /home/saintwings/ackermann_esp32/ \
  linaro@<TINKER_BOARD_IP>:~/robot_server/
```

### 2.2 Create venv and install dependencies

```bash
ssh linaro@<TINKER_BOARD_IP>
cd ~/robot_server

python3 -m venv venv

venv/bin/pip install websockets flask flask-login numpy
```

> **Note:** `numpy` may take several minutes to build from source on ARM.
> If it fails, install the system package first:
> ```bash
> sudo apt install python3-numpy -y
> venv/bin/pip install websockets flask flask-login
> ```

### 2.3 Test the server manually

```bash
cd ~/robot_server
venv/bin/python3 control_server.py --host 0.0.0.0 --port 8765
```

Expected output:
```
INFO: WebSocket server listening on ws://0.0.0.0:8765
INFO: HTTP server listening on http://0.0.0.0:5000
```

Press **Ctrl+C** to stop.

> **Key point:** You do NOT need `source venv/bin/activate` in systemd.
> Calling `venv/bin/python3` directly is equivalent — the venv's packages
> are automatically available.

### 2.4 Create systemd service

```bash
sudo nano /etc/systemd/system/robot-server.service
```

```ini
[Unit]
Description=Robot Control Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=linaro
WorkingDirectory=/home/linaro/robot_server
ExecStart=/home/linaro/robot_server/venv/bin/python3 control_server.py --host 0.0.0.0 --port 8765
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-server
sudo systemctl start robot-server
sudo systemctl status robot-server   # should show: active (running)
```

---

## Part 3 — Cloudflare Tunnel

### 3.1 Download cloudflared (ARMv7 for Tinker Board)

```bash
wget https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm \
     -O /tmp/cloudflared
chmod +x /tmp/cloudflared
sudo mv /tmp/cloudflared /usr/local/bin/cloudflared
cloudflared --version   # should show 2026.x.x
```

### 3.2 Authenticate with Cloudflare

```bash
cloudflared tunnel login
```

Copy the printed URL → open in browser → select `saintwings.xyz` → Authorize.
Certificate saved to `~/.cloudflared/cert.pem`.

### 3.3 Create the tunnel

```bash
cloudflared tunnel create robot-server
```

Note the tunnel ID printed (e.g. `6521735f-aedc-4cdd-9eaa-4815937703c1`).

### 3.4 Create tunnel config at `/etc/cloudflared/config.yml`

> ⚠️ **Use `/etc/cloudflared/` — not `~/.cloudflared/`.**
> `sudo cloudflared service install` runs as root and cannot see `~linaro`.

```bash
sudo mkdir -p /etc/cloudflared
sudo nano /etc/cloudflared/config.yml
```

```yaml
tunnel: 6521735f-aedc-4cdd-9eaa-4815937703c1
credentials-file: /home/linaro/.cloudflared/6521735f-aedc-4cdd-9eaa-4815937703c1.json

ingress:
  # Mission Planner web UI (Flask HTTP server)
  - hostname: app.saintwings.xyz
    service: http://localhost:5000

  # Robot WebSocket control server
  - hostname: robot.saintwings.xyz
    service: ws://localhost:8765

  # Required catch-all
  - service: http_status:404
```

### 3.5 Create DNS records

```bash
cloudflared tunnel route dns robot-server app.saintwings.xyz
cloudflared tunnel route dns robot-server robot.saintwings.xyz
```

Both appear in Cloudflare DNS as CNAME records pointing to
`<tunnel-id>.cfargotunnel.com`. The grey cloud (DNS-only) icon is correct
for tunnel routes — do not enable the orange proxy cloud.

### 3.6 Fix UDP buffer size (improves QUIC performance)

```bash
sudo sysctl -w net.core.rmem_max=7500000
sudo sysctl -w net.core.wmem_max=7500000

echo "net.core.rmem_max=7500000" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max=7500000" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 3.7 Test tunnel manually (before installing as service)

```bash
cloudflared tunnel run robot-server
```

Healthy output:
```
SUMMARY: Environment is healthy. cloudflared will use 'quic' as primary protocol.
INF Registered tunnel connection connIndex=0 location=sin21 protocol=quic
INF Registered tunnel connection connIndex=1 location=sin15 protocol=quic
INF Registered tunnel connection connIndex=2 location=sin15 protocol=quic
INF Registered tunnel connection connIndex=3 location=sin20 protocol=quic
```

The `ICMP proxy disabled` warning is harmless — WebSocket is unaffected.

Press **Ctrl+C** when confirmed working.

### 3.8 Install cloudflared as a systemd service

```bash
sudo cloudflared service install
sudo systemctl enable cloudflared
sudo systemctl start cloudflared
sudo systemctl status cloudflared   # should show: active (running)
```

---

## Part 4 — Verify End-to-End

### Browser test

Open `https://app.saintwings.xyz` in a browser.

You should see the **login page** — not an error. ✅

Open `https://robot.saintwings.xyz` in a browser.

You should see:
```
Failed to open a WebSocket connection: invalid Connection header: keep-alive.
You cannot access a WebSocket server directly with a browser.
You need a WebSocket client.
```
This is correct — it means the full chain (browser → Cloudflare → tunnel →
`control_server.py`) is working.

### Python WebSocket test

```bash
python3 test_server.py
```

Expected:
```
Connecting to wss://robot.saintwings.xyz ...
Connected!
Sent registration
No reply from server within 5s (server may not send ack — connection still OK)
```

`Connected!` = WebSocket handshake works end-to-end. ✅
`No reply` = normal — server waits for the robot to send telemetry first.

---

## Part 5 — User Accounts & Login

### How authentication works

```
User visits https://app.saintwings.xyz
  → redirected to /login if not logged in
  → after login, session cookie is set
  → JS fetches /api/auth/ws-token (short-lived UUID token)
  → WebSocket connect sends token in register message
  → server validates token → identifies user → filters robots by assignment
```

### First-time setup

1. Go to `https://app.saintwings.xyz/signup`
2. Create your account — **the first registered user is automatically admin**
3. Log in → Mission Planner opens

### User database

Stored in `/home/linaro/robot_server/users.db` (SQLite).

To view or fix manually:
```bash
sqlite3 /home/linaro/robot_server/users.db

SELECT id, username, email, is_admin FROM users;

-- Make a user admin manually if needed
UPDATE users SET is_admin = 1 WHERE id = 1;

.quit
```

---

## Part 6 — Admin Panel & Robot Assignment

### Access admin panel

`https://app.saintwings.xyz/admin`

Only accessible by users with `is_admin = 1`.

The topbar in the Mission Planner shows an **⚙ Admin** link for admin users.

### Assign robots to users

1. Open admin panel
2. **Assign Robot to User** section at the bottom right
3. Select a user from the dropdown
4. Type the Robot ID (must match `ROBOT_ID` in `Config_2.h`, e.g. `esp32-02`)
5. Click **Assign**

### What assignment controls

| User type | Sees / controls |
|-----------|----------------|
| Admin | All robots (no restriction) |
| Regular user | Only robots explicitly assigned to them |
| Unauthenticated | Rejected immediately on WebSocket connect |

Robot telemetry, robot list, and commands are all filtered per user on the
server side. A user cannot send commands to a robot they are not assigned to.

---

## Part 7 — Daily Operations

### Sync code from dev machine to Tinker Board

```bash
rsync -av --exclude='.pio' --exclude='__pycache__' --exclude='venv' \
  /home/saintwings/ackermann_esp32/ \
  linaro@<TINKER_BOARD_IP>:~/robot_server/

ssh linaro@<TINKER_BOARD_IP> "sudo systemctl restart robot-server"
```

### View logs

```bash
sudo journalctl -u robot-server -f     # control server live log
sudo journalctl -u cloudflared -f      # tunnel live log
sudo journalctl -u robot-server -n 50  # last 50 lines
```

### Check service status

```bash
sudo systemctl status robot-server
sudo systemctl status cloudflared
```

### Restart services

```bash
sudo systemctl restart robot-server
sudo systemctl restart cloudflared
```

### Tunnel management

```bash
cloudflared tunnel list
cloudflared tunnel info robot-server
```

---

## Quick Reference

| URL | Purpose |
|-----|---------|
| `https://app.saintwings.xyz` | Mission Planner web UI |
| `https://app.saintwings.xyz/login` | Login page |
| `https://app.saintwings.xyz/signup` | Register new user |
| `https://app.saintwings.xyz/admin` | Admin panel (admin only) |
| `wss://robot.saintwings.xyz` | WebSocket (robots + web clients) |
| `ws://192.168.1.152:8765` | WebSocket when on local WiFi |
| `http://192.168.1.152:5000` | Web UI when on local network |

| Service | What it runs |
|---------|-------------|
| `robot-server` | `control_server.py` — WebSocket :8765 + Flask :5000 |
| `cloudflared` | Tunnel daemon → Cloudflare edge |

---

## Running on Any Computer (Local or Public)

The control server is a plain Python script — it runs on any machine with Python 3.9+.
No Docker, no special OS, no Tinker Board required.

### Prerequisites (any computer)

```bash
pip install websockets flask flask-login numpy
```

Files needed from the project folder:
```
control_server.py
coverage_path_service.py
coverage_path_planning.py
auth.py
templates/
mission_planner_v2.html
```

---

### Mode 1 — Local Network (any computer on same WiFi/LAN)

**1. Start the server**
```bash
python3 control_server.py --host 0.0.0.0 --port 8765
```

**2. Find this computer's local IP**
```bash
# Linux / Mac
ip addr show | grep "inet " | grep -v 127
# Windows
ipconfig
```

**3. Open web UI** from any browser on the same network:
```
http://<THIS_COMPUTER_IP>:5000
```

**4. Update firmware** — set the server IP in `include/Config_2.h`:
```cpp
#define SERVER_MODE 1
#define CONTROL_SERVER_HOST "192.168.1.xxx"   // ← this computer's local IP
```

> First run: go to `/signup` to create an account. The first account is automatically admin.

---

### Mode 2 — Public via Cloudflare Tunnel (any computer, anywhere)

Cloudflare Tunnel must run **on the same machine** as `control_server.py`
because `cloudflared` forwards to `localhost`.

**Option A — Tinker Board (already configured, runs as systemd service)**
```bash
sudo systemctl start robot-server
sudo systemctl start cloudflared
```
Web UI: `https://app.saintwings.xyz`

---

**Option B — Any other computer**

Step 1: Copy tunnel credentials from Tinker Board:
```bash
scp -r linaro@<TINKER_BOARD_IP>:~/.cloudflared ~/.cloudflared
```

Step 2: Install cloudflared on the new computer:
```bash
# Linux x64
wget https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 -O cloudflared
chmod +x cloudflared && sudo mv cloudflared /usr/local/bin/

# macOS
brew install cloudflared

# Windows — download from:
# https://github.com/cloudflare/cloudflared/releases/latest
```

Step 3: Run both in separate terminals:
```bash
# Terminal 1 — control server
python3 control_server.py --host 0.0.0.0 --port 8765

# Terminal 2 — Cloudflare tunnel
cloudflared tunnel run robot-server
```

Web UI: `https://app.saintwings.xyz`

**Firmware — no IP change needed**, just set mode:
```cpp
#define SERVER_MODE 2
// host/port/SSL configured automatically
```

---

### Mode Comparison

| | Mode 1 — Local | Mode 2 — Public |
|--|---|---|
| **Web UI** | `http://<IP>:5000` | `https://app.saintwings.xyz` |
| **WebSocket** | `ws://<IP>:8765` | `wss://robot.saintwings.xyz` |
| **Firmware** | `SERVER_MODE 1` + correct local IP | `SERVER_MODE 2` |
| **Commands to start** | `python3 control_server.py ...` | same + `cloudflared tunnel run robot-server` |
| **Internet required** | ❌ LAN only | ✅ |
| **SIM support** | NTRIP corrections only | Full control + NTRIP |
| **Run anywhere** | ✅ any computer | ✅ copy `~/.cloudflared/` credentials |

### Firmware SERVER_MODE cheat sheet

```cpp
// include/Config_2.h

#define SERVER_MODE 1   // ← local network test / development
#define SERVER_MODE 2   // ← field deployment / remote monitoring
```

One line change → reflash → done.

---

## Troubleshooting

**Login page not appearing at `app.saintwings.xyz`**
→ `robot-server` is down: `sudo systemctl status robot-server`
→ Check logs: `sudo journalctl -u robot-server -n 30`

**`No module named 'flask_login'`**
→ `venv/bin/pip install flask-login`
→ `sudo systemctl restart robot-server`

**`sudo cloudflared service install` says "no config file found"**
→ Config must be in `/etc/cloudflared/config.yml` (not `~/.cloudflared/`)
→ `sudo mkdir -p /etc/cloudflared && sudo cp ~/.cloudflared/config.yml /etc/cloudflared/`

**Cloudflare error 1033 or 502 in browser**
→ `robot-server` service is down or crashed
→ `sudo systemctl restart robot-server`

**WebSocket shows "Authentication required" on connect**
→ Session cookie expired — log out and log back in

**User cannot see their robot in Mission Planner**
→ Admin needs to assign the robot: admin panel → Assign Robot to User
→ Robot ID must exactly match `ROBOT_ID` in firmware `Config_2.h`

**`status=203/EXEC` in robot-server service**
→ venv Python binary not found at the path in `ExecStart`
→ Check: `ls /home/linaro/robot_server/venv/bin/python3`
→ If missing: `cd ~/robot_server && python3 -m venv venv && venv/bin/pip install websockets flask flask-login numpy`
