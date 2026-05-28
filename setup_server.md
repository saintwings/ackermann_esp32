# Control Server Setup — Tinker Board + Cloudflare Tunnel

Expose `control_server.py` to the internet via **Cloudflare Tunnel** so the
robot can connect over SIM/cellular using your domain `saintwings.xyz`.

**Architecture:**
```
Robot (SIM cellular)
  │  wss://robot.saintwings.xyz
  ▼
Cloudflare Edge  ◄─── free tunnel, TLS terminated here
  │  ws://localhost:8765  (inside the tunnel)
  ▼
Tinker Board (user: linaro)
  └─ control_server.py  (running locally, no port forwarding needed)
```

No port-forwarding, no static IP required.
Cloudflare's **free** plan supports WebSocket and the tunnel is free.

---

## Prerequisites

- Tinker Board running **TinkerOS / Armbian** (default user: `linaro`)
- Python 3.9+ installed
- Domain `saintwings.xyz` registered on Namecheap
- Cloudflare account (free at cloudflare.com)

---

## Part 1 — Add Your Domain to Cloudflare

### 1.1 Sign in to Cloudflare

Go to [dash.cloudflare.com](https://dash.cloudflare.com) → **Add a Site** →
enter `saintwings.xyz` → choose the **Free** plan.

Cloudflare scans your existing DNS records. Click **Continue**.

### 1.2 Change Nameservers on Namecheap

Cloudflare gives you two nameservers, for example:
```
aida.ns.cloudflare.com
brad.ns.cloudflare.com
```

1. Log in to [namecheap.com](https://namecheap.com)
2. Go to **Domain List** → click **Manage** next to `saintwings.xyz`
3. Under **Nameservers**, select **Custom DNS**
4. Enter the two nameservers Cloudflare gave you
5. Click the green ✓ to save

> ⏱️ DNS propagation takes **5–30 minutes**. Cloudflare will email you when active.

### 1.3 Enable WebSocket Support on Cloudflare

In Cloudflare dashboard for `saintwings.xyz`:

1. Go to **Network** (left sidebar)
2. Find **WebSockets** → toggle **On**

---

## Part 2 — Set Up the Control Server on Tinker Board

### 2.1 Copy project files to Tinker Board

From your development machine:
```bash
rsync -av --exclude='.pio' --exclude='__pycache__' \
  /home/saintwings/ackermann_esp32/ \
  linaro@<TINKER_BOARD_IP>:~/robot_server/
```

Or clone your repo directly on the Tinker Board.

### 2.2 Install Python dependencies

SSH into the Tinker Board:
```bash
ssh linaro@<TINKER_BOARD_IP>
cd ~/robot_server

# Create a virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install websockets flask
```

### 2.3 Test the server runs locally

```bash
cd ~/robot_server
source venv/bin/activate
python3 control_server.py --host 0.0.0.0 --port 8765
```

You should see:
```
INFO: WebSocket server started on 0.0.0.0:8765
INFO: HTTP server started on 0.0.0.0:5000
```

Press **Ctrl+C** to stop for now.

### 2.4 Create a systemd service (auto-start on boot)

```bash
sudo nano /etc/systemd/system/robot-server.service
```

Paste this:
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

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-server
sudo systemctl start robot-server

# Check it is running
sudo systemctl status robot-server
```

---

## Part 3 — Install Cloudflare Tunnel (`cloudflared`)

### 3.1 Download cloudflared for ARM (Tinker Board)

The Tinker Board uses an **ARMv7** CPU. Run on the Tinker Board:

```bash
wget https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm \
     -O /tmp/cloudflared

chmod +x /tmp/cloudflared
sudo mv /tmp/cloudflared /usr/local/bin/cloudflared

# Verify
cloudflared --version
```

Expected output: `cloudflared version 2026.x.x`

### 3.2 Authenticate cloudflared with your Cloudflare account

```bash
cloudflared tunnel login
```

This prints a URL. **Copy it and open it in your browser**.
Select `saintwings.xyz` from the list → Authorize.

A certificate file is saved at `~/.cloudflared/cert.pem` automatically.

### 3.3 Create a named tunnel

```bash
cloudflared tunnel create robot-server
```

Output example:
```
Created tunnel robot-server with id 6521735f-aedc-4cdd-9eaa-4815937703c1
```

> 📝 Note your tunnel ID — it appears in `~/.cloudflared/<tunnel-id>.json`

### 3.4 Create the tunnel configuration file

```bash
nano ~/.cloudflared/config.yml
```

Paste (replace `YOUR_TUNNEL_ID` with the ID from step 3.3):

```yaml
tunnel: YOUR_TUNNEL_ID
credentials-file: /home/linaro/.cloudflared/YOUR_TUNNEL_ID.json

ingress:
  # Robot WebSocket control server
  - hostname: robot.saintwings.xyz
    service: ws://localhost:8765

  # Catch-all (required by cloudflared)
  - service: http_status:404
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

### 3.5 Create the DNS record

```bash
cloudflared tunnel route dns robot-server robot.saintwings.xyz
```

Output:
```
Added CNAME robot.saintwings.xyz which will route to this tunnel
```

Verify in Cloudflare dashboard → **DNS** — you'll see a CNAME
`robot → <tunnel-id>.cfargotunnel.com` (DNS-only icon is correct for tunnels).

### 3.6 Test the tunnel manually

```bash
cloudflared tunnel run robot-server
```

**Healthy output looks like this (your actual output):**
```
SUMMARY: Environment is healthy. cloudflared will use 'quic' as primary protocol.
INF Registered tunnel connection connIndex=0 location=sin21 protocol=quic
INF Registered tunnel connection connIndex=1 location=sin15 protocol=quic
INF Registered tunnel connection connIndex=2 location=sin15 protocol=quic
INF Registered tunnel connection connIndex=3 location=sin20 protocol=quic
```

4 connections to Singapore PoPs = tunnel is running. ✅

**Expected warnings (harmless):**

| Warning | Cause | Impact |
|---------|-------|--------|
| `ICMP proxy disabled — GID not in ping_group_range` | Linux ping permission | None — WebSocket is unaffected |
| `failed to increase receive buffer` (got 416 kiB, wanted 7168 kiB) | OS kernel default | Minor throughput limit — fix below |

### 3.7 Fix the UDP buffer warning (optional, improves performance)

```bash
# Apply now
sudo sysctl -w net.core.rmem_max=7500000
sudo sysctl -w net.core.wmem_max=7500000

# Make permanent across reboots
echo "net.core.rmem_max=7500000" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max=7500000" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 3.8 Install cloudflared as a systemd service

Press **Ctrl+C** to stop the manual tunnel run, then:

```bash
sudo cloudflared service install

# if failed
# Copy config to the system-wide location
# sudo mkdir -p /etc/cloudflared
# sudo cp /home/linaro/.cloudflared/config.yml /etc/cloudflared/config.yml

sudo systemctl enable cloudflared
sudo systemctl start cloudflared

# Verify
sudo systemctl status cloudflared
```

Both services should now survive reboots:
```bash
sudo systemctl status robot-server   # control_server.py
sudo systemctl status cloudflared    # tunnel to Cloudflare
```

---

## Part 4 — Test the Tunnel From Outside

### From any device on the internet

Open a browser and visit `https://robot.saintwings.xyz`

You should see this message in the browser:
```
Failed to open a WebSocket connection: invalid Connection header: keep-alive.
You cannot access a WebSocket server directly with a browser.
You need a WebSocket client.
```

This means:
- ✅ Cloudflare received the request
- ✅ The tunnel forwarded it to `control_server.py`
- ✅ The Python `websockets` library rejected the plain browser request (correct — it expects a proper WebSocket client)

### From a terminal (WebSocket test)

```bash
python3 -c "
import asyncio, websockets

async def test():
    async with websockets.connect('wss://robot.saintwings.xyz') as ws:
        print('Connected!')
        msg = await asyncio.wait_for(ws.recv(), timeout=5)
        print('Server:', msg)

asyncio.run(test())
"
```

You should see `Connected!` and a JSON message from `control_server.py`.

---

## Part 5 — Update Firmware for SIM_CONTROL_ENABLE = 1

Edit [`include/Config_2.h`](include/Config_2.h):

```cpp
// ── SIM control server ────────────────────────────────────────────────────────
#define SIM_CONTROL_ENABLE       1

// Cloudflare Tunnel public address
#define CONTROL_SERVER_HOST_SIM  "robot.saintwings.xyz"
#define CONTROL_SERVER_PORT_SIM  443        // Cloudflare terminates TLS on 443
```

> ⚠️ **WSS (secure WebSocket) is required for Cloudflare on port 443.**
> The current `links2004/WebSockets` library can do WSS with `beginSSL()`.
> A WebSocket library swap to `gilmaimon/ArduinoWebsockets` is recommended
> for cleaner TinyGSM + WSS support — see next steps.

---

## Part 6 — Daily Operations

### Logs

```bash
# Watch control server logs live
sudo journalctl -u robot-server -f

# Watch tunnel logs live
sudo journalctl -u cloudflared -f

# Last 50 lines of each
sudo journalctl -u robot-server -n 50
sudo journalctl -u cloudflared -n 50
```

### After updating control_server.py

```bash
# Pull latest code then restart the service
cd ~/robot_server && git pull
sudo systemctl restart robot-server
```

### Tunnel management

```bash
cloudflared tunnel list                    # list all tunnels
cloudflared tunnel info robot-server       # show connections
cloudflared tunnel delete robot-server     # delete tunnel (if needed)
```

---

## Quick Reference

| URL | Used when |
|-----|-----------|
| `ws://192.168.1.152:8765` | Robot on local WiFi |
| `wss://robot.saintwings.xyz` | Robot on SIM over internet |

| Service | Controls |
|---------|---------|
| `robot-server` | `control_server.py` on port 8765 |
| `cloudflared` | Tunnel to Cloudflare edge |

---

## Troubleshooting

**Browser shows "Failed to open a WebSocket connection: invalid Connection header"**
→ ✅ This is correct. The tunnel and server are working. Use the Python test above instead.

**Browser shows Cloudflare error page (1033 / 502)**
→ `robot-server` service is down. Run `sudo systemctl status robot-server`.

**`cloudflared tunnel run` says credentials not found**
→ Check `~/.cloudflared/config.yml` has the correct tunnel ID and credentials path.

**CNAME record shows grey cloud (DNS-only) in Cloudflare dashboard**
→ ✅ This is expected for tunnel routes. Do not proxy it.

**`wscat` connects but immediately disconnects**
→ Check `control_server.py` logs: `sudo journalctl -u robot-server -f`
→ The server may need a valid registration packet from a known robot ID.
