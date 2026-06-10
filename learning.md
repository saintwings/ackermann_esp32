# Learning Guide: Networking, WebSockets, and Robot Communication

A complete explanation from zero background knowledge, using this project as the real-world example.

---

## Chapter 1: How Computers Talk to Each Other

### 1.1 The Physical World

When two devices want to communicate — a robot and a server, for example — they need a path between them. That path might be:

- A **WiFi radio signal** between the robot and a router
- A **4G/LTE cellular signal** between the robot's SIM modem and a cell tower
- **Fiber optic cables** between the cell tower and a data centre
- **Submarine cables** crossing the ocean between continents

The data travels as electrical signals, radio waves, or pulses of light. But all of these physical layers are transparent to the software — the software just sees "I can send and receive bytes."

### 1.2 IP Addresses

Every device on the internet has an **IP address** — a number that identifies its location on the network. Like a postal address, but for data.

Examples:
- `192.168.1.41` — a local IP (only works inside your home network)
- `104.21.34.0` — a public IP (reachable from anywhere on the internet)

When the robot sends data to the server, it needs to know the server's IP address (or a domain name that maps to one, like `robot.saintwings.xyz`).

### 1.3 Ports

A computer runs many programs at once. A **port** is a number (0–65535) that says which program should receive the incoming data.

Think of a building: the IP address is the building address, the port is the apartment number.

Common ports:
- Port **80** — HTTP (web pages, unencrypted)
- Port **443** — HTTPS (web pages, encrypted)
- Port **8765** — our local development server

### 1.4 TCP: The Reliable Pipe

**TCP (Transmission Control Protocol)** is the foundation of most internet communication. It provides:

- **Reliability**: If a packet gets lost, TCP automatically retransmits it
- **Ordering**: Data arrives in the same order it was sent
- **Flow control**: Doesn't flood the receiver with data faster than it can process

A TCP connection is a **bidirectional pipe** between two endpoints. Once open, both sides can send data to each other freely.

**But TCP has no concept of messages.** It is a stream of bytes. If you send `{"hello"}` and then `{"world"}`, the receiver might get:

```
Read 1: {"hell
Read 2: o"}{"wor
Read 3: ld"}
```

The bytes arrive in order, but in arbitrary chunks. Higher-level protocols (like HTTP or WebSocket) add structure on top.

---

## Chapter 2: HTTP — The Language of the Web

### 2.1 Request and Response

**HTTP (HyperText Transfer Protocol)** is a protocol built on top of TCP. It defines a simple conversation:

1. Client sends a **request**: "Give me this page"
2. Server sends a **response**: "Here is the page"
3. Connection is closed (traditionally)

A request looks like this:
```
GET /index.html HTTP/1.1
Host: example.com
User-Agent: MyBrowser/1.0

```

A response looks like this:
```
HTTP/1.1 200 OK
Content-Type: text/html
Content-Length: 1234

<html>...</html>
```

The structure is: **headers** (text lines ending with `\r\n`), then a **blank line** (`\r\n\r\n`), then the **body**.

### 2.2 Why HTTP Alone Isn't Enough for a Robot

HTTP is **request-response**: the client asks, the server answers. The server cannot spontaneously send data to the client.

For a robot, we need the server to be able to send a "turn left" command at any moment — without the robot constantly asking "do you have a command for me? do you have a command for me?" every millisecond. That constant asking is called **polling** and it wastes enormous bandwidth.

We need a **persistent, bidirectional connection** where either side can send at any time.

---

## Chapter 3: WebSocket — Upgrading the Connection

### 3.1 The Problem WebSocket Solves

WebSocket was invented to solve exactly the HTTP problem above. It lets you:

1. Start with a normal HTTP connection (so firewalls and proxies don't block it)
2. **Upgrade** that connection to a persistent, bidirectional channel
3. Send messages in either direction at any time, with very low overhead

WebSocket is defined in **RFC 6455** (published 2011). RFC stands for "Request for Comments" — it is the formal specification that every WebSocket implementation in the world must follow.

### 3.2 The Handshake — Knocking on the Door

Before WebSocket communication begins, the client must ask the server to upgrade. This uses a normal HTTP GET request with special headers.

**Step 1: Client sends the upgrade request**

```
GET / HTTP/1.1
Host: robot.saintwings.xyz
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
```

Key headers:
- `Upgrade: websocket` — "I want to switch protocols"
- `Connection: Upgrade` — "This is a protocol upgrade request"
- `Sec-WebSocket-Key` — A random 16-byte value encoded in base64. The server uses this to prove it understood the request.
- `Sec-WebSocket-Version: 13` — We want WebSocket version 13 (the current standard)

**Step 2: Server accepts with HTTP 101**

```
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
```

`101 Switching Protocols` means "OK, I agree to switch". After this response, both sides stop speaking HTTP and start speaking WebSocket.

The `Sec-WebSocket-Accept` value is computed from the client's key:
```
accept = base64(SHA1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"))
```
The strange string `"258EAFA5..."` is a magic constant defined in the RFC. The client checks this to make sure it's talking to a real WebSocket server, not an HTTP server that accidentally replied 101.

**In our project** (from `sim_ws_client.cpp`):

```cpp
// Build the GET request
String req = "GET ";
req += path;
req += " HTTP/1.1\r\n";
req += "Host: ";
req += host;
req += "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n";
req += "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n";
req += "Sec-WebSocket-Version: 13\r\n\r\n";
```

We use a fixed key (`dGhlIHNhbXBsZSBub25jZQ==`) — this is the example from the RFC itself. The key's purpose is just to verify the server is a real WebSocket server, so using a fixed value is fine for our use case.

### 3.3 The Cloudflare Complication

Our robot connects through **Cloudflare** — a service that protects and accelerates websites. Cloudflare sits between the robot and our Python server.

```
Robot → Cloudflare → Python server
```

When Cloudflare relays the `101 Switching Protocols` response, it **adds its own headers** at the end:

```
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=

CF-RAY: 9481a6b4b4f7b6a-BKK
alt-svc: h3=":443"; ma=86400
```

Notice there are now **two blank lines** (`\r\n\r\n`): one after the standard headers, and one after Cloudflare's headers. If we stop reading at the first blank line, we leave `CF-RAY: ...` and `alt-svc: ...` sitting in the TCP buffer. Those bytes will then be read as the first WebSocket frame — causing complete garbage.

**Our fix**: After finding the first blank line, we wait 500ms and read again to consume any remaining headers. We never save those extra bytes — we just throw them away. Then we clear `_rxLen = 0` to guarantee a clean start.

### 3.4 The Modem AT Command Complication

The SIM modem (SIMCOM A7670C) does not give us a raw TCP socket like WiFi does. We have to send **AT commands** (text commands) to the modem to make it do things:

```
ESP32 → modem: AT+CIPRXGET=2,1,512\r\n   (please give me data from channel 1)
modem → ESP32: +CIPRXGET: 2,1,127,0      (here is 127 bytes from channel 1)
               [127 bytes of actual data]
modem → ESP32: OK\r\n                     (AT command completed)
```

The `OK\r\n` at the end is the modem saying "the AT command finished" — it is part of the AT protocol, not part of the TCP data. But if our code isn't careful, those 4 bytes (`O`, `K`, `\r`, `\n`) end up mixed in with the TCP data.

`'O'` in ASCII is byte `0x4F`. When our WebSocket parser reads the first byte of a frame:
```
opcode = byte & 0x0F
0x4F & 0x0F = 0x0F    ← garbage opcode, not a real WebSocket opcode
```

The parser sees an unknown opcode and the connection crashes. This was the bug we fixed.

---

## Chapter 4: WebSocket Frames — The Message Format

### 4.1 Why a Special Format?

Remember that TCP is a byte stream with no message boundaries. WebSocket frames solve this by starting each message with a small header that says exactly how many bytes follow.

### 4.2 Frame Structure

```
 Byte 0         Byte 1         Bytes 2+        Last bytes
┌──────────────┬──────────────┬───────────────┬─────────────────┐
│FIN│RSV│OPCODE│MASK│ LENGTH  │ EXTENDED LEN  │ MASK KEY + DATA │
│ 1 │ 3 │  4   │ 1  │   7    │  (if needed)  │                 │
└──────────────┴──────────────┴───────────────┴─────────────────┘
```

**Byte 0:**
- `FIN` (1 bit): `1` = this is the complete message (we always use 1)
- `RSV` (3 bits): reserved, always `000`
- `OPCODE` (4 bits): what kind of frame this is

**Byte 1:**
- `MASK` (1 bit): `1` = payload is masked (client→server always)
- `LENGTH` (7 bits): payload length

**Length encoding** (clever space-saving trick):
- If length ≤ 125: those 7 bits hold the actual length
- If length = 126: the NEXT 2 bytes hold the actual length (16-bit, up to 65535)
- If length = 127: the NEXT 8 bytes hold the actual length (64-bit, huge)

Our telemetry JSON (~200 bytes) uses the 126 case:
```
Byte 0: 0x81  (FIN=1, opcode=0x01 text)
Byte 1: 0xFE  (MASK=1, length=126 → read next 2 bytes)
Byte 2: 0x00  (high byte of 16-bit length)
Byte 3: 0xC8  (low byte → 0x00C8 = 200 bytes)
Bytes 4-7: mask key (4 bytes)
Bytes 8-207: masked payload (200 bytes)
```

### 4.3 Opcodes

| Value | Name | Meaning |
|-------|------|---------|
| `0x01` | Text | UTF-8 text message (JSON in our case) |
| `0x02` | Binary | Binary data |
| `0x08` | Close | Gracefully closing the connection |
| `0x09` | Ping | "Are you still there?" |
| `0x0A` | Pong | "Yes, still here" (reply to ping) |

### 4.4 Masking — Why Client→Server Data Is Scrambled

The RFC requires all client-to-server frames to be **masked** (XOR-scrambled with a 4-byte key). Server-to-client frames are NOT masked.

This seems strange — why would you scramble your own data? The reason is historical: in 2010, researchers discovered that certain browser-controlled WebSocket connections could be used to trick intermediate HTTP proxies (caches) into storing malicious data. Masking prevents this specific attack. For a robot connecting directly (not through a browser), masking doesn't provide real security, but the RFC requires it anyway.

**How masking works:**
```
mask_key = [key0, key1, key2, key3]   (4 random bytes)

for i in range(len(payload)):
    masked_byte = payload[i] XOR mask_key[i % 4]
```

The receiver reverses it with the same operation (XOR is its own inverse):
```
for i in range(len(masked_payload)):
    original_byte = masked_payload[i] XOR mask_key[i % 4]
```

In our code (`sim_ws_client.cpp`):
```cpp
void SimWsClient::sendWsFrame(uint8_t opcode, const uint8_t* data, size_t len) {
    // ...
    const uint8_t kMask[4] = {0x37, 0x42, 0x13, 0x99};  // fixed mask key
    for (size_t i = 0; i < len; i++) {
        buf[hdrLen + i] = data[i] ^ kMask[i & 3];
    }
    // ...
}
```

### 4.5 Parsing a Frame

When the robot receives data from the server, it reads byte by byte:

```
Step 1: Read byte 0 → extract FIN, opcode
Step 2: Read byte 1 → extract MASK bit, length
Step 3: If length == 126, read 2 more bytes for actual length
        If length == 127, read 8 more bytes for actual length
Step 4: If MASK bit is set, read 4 mask key bytes
Step 5: Read [length] bytes of payload
Step 6: If masked, XOR-decode with mask key
Step 7: Pass decoded payload to application
```

The parsing code in `sim_ws_client.cpp`:
```cpp
uint8_t b0 = _rxBuf[0];
uint8_t b1 = _rxBuf[1];
uint8_t opcode = b0 & 0x0F;
bool masked = (b1 & 0x80) != 0;
uint64_t plen = b1 & 0x7F;

size_t hdrLen = 2;
if (plen == 126) {
    plen = ((uint64_t)_rxBuf[2] << 8) | _rxBuf[3];
    hdrLen = 4;
} else if (plen == 127) {
    // read 8 bytes ...
}
if (masked) hdrLen += 4;
```

---

## Chapter 5: Ping/Pong — Keeping the Connection Alive

### 5.1 The Idle Connection Problem

TCP connections can be closed by intermediate network equipment (routers, load balancers, NAT gateways) if they are idle for too long. Each piece of equipment has its own timeout:

- Home router NAT: typically 30–60 seconds
- **Cloudflare WebSocket**: **100 seconds**
- Mobile network NAT (4G): typically 30 seconds

If the robot stops sending telemetry for any reason, the connection will be silently killed. The robot won't know until it tries to send data and gets no response.

### 5.2 Ping/Pong to the Rescue

WebSocket includes a built-in keepalive mechanism:

- **Ping frame** (opcode `0x09`): "I'm still here, are you?"
- **Pong frame** (opcode `0x0A`): "Yes, I'm still here"

The RFC says: when you receive a ping, you MUST reply with a pong as soon as possible.

Our robot sends a ping every 15 seconds:

```cpp
void SimWsClient::sendPing() {
    sendWsFrame(0x09, nullptr, 0);  // Ping with empty payload
}
```

The Python `websockets` library on our server automatically replies with a pong. The robot's parser handles incoming pings from the server too:

```cpp
} else if (opcode == 0x09) {   // Ping from server
    sendWsFrame(0x0A, data, plen);  // Reply with Pong
}
```

Timeline with ping/pong:
```
t=0s    Robot → Server: telemetry
t=2s    Robot → Server: telemetry
...
t=14s   Robot → Server: telemetry
t=15s   Robot → Server: [PING]
t=15s   Server → Robot: [PONG]
t=16s   Robot → Server: telemetry
...
```

The ping resets the Cloudflare 100-second idle timer. As long as pings happen every 15 seconds, the connection stays alive forever.

### 5.3 Close Frame

When either side wants to end the connection cleanly:

1. Sender sends a **Close frame** (opcode `0x08`)
2. Receiver replies with its own Close frame
3. Both sides close the TCP connection

In our code:
```cpp
void SimWsClient::disconnect() {
    if (_connected) {
        uint8_t closeFrame[] = {0x88, 0x80, 0x00, 0x00, 0x00, 0x00};
        cipSend(closeFrame, sizeof(closeFrame));
    }
    _connected = false;
    cipClose();
}
```

`0x88` = `FIN=1, opcode=0x08 (close)`, `0x80` = `MASK=1, length=0`.

---

## Chapter 6: Our Project's Complete Message Protocol

Now that you understand TCP, HTTP, and WebSocket, let's look at exactly what messages flow in this system.

### 6.1 The Message Format

All messages in our system are **JSON** (JavaScript Object Notation) sent as WebSocket Text frames.

Every message has this structure:
```json
{
  "type": <number>,
  "robot_id": "ladybug_001",
  "payload": { ... }
}
```

**Message types:**

| Type | Name | Direction | Meaning |
|------|------|-----------|---------|
| `0` | REGISTER | Robot → Server | "I just connected, here is who I am" |
| `1` | TELEMETRY | Robot → Server | "Here is my current sensor data" |
| `2` | ACK | Both | "I received your message" |
| `10` | CMD_MOVE | Server → Robot | "Move at this speed and steering angle" |
| `11` | CMD_MISSION | Server → Robot | "Run this waypoint mission" |
| `12` | CMD_STOP | Server → Robot | "Stop immediately" |
| `99` | HEARTBEAT | Server → Robot | Server's keepalive tick |

### 6.2 The Full Lifecycle

#### Phase 1: Network Setup (~15–60 seconds)

**For WiFi (esp32-02, NET_MODE=3):**
```
Boot
→ esp_wifi_start()
→ Connect to "raina_robofarm_2.4G"
→ Got IP: 192.168.1.41
→ WebSocketsClient connects to robot.saintwings.xyz:443 (TLS)
→ TLS handshake (encrypted tunnel established)
→ WebSocket handshake (HTTP 101)
→ Connected
```

**For SIM (ladybug_001, NET_MODE=2):**
```
Boot
→ UART2 opens at 115200 baud to modem
→ AT → OK          (modem alive)
→ AT+CGREG?        (check 4G registration)
→ AT+CGDCONT=1,"IP","internet"  (set APN)
→ AT+CIICR         (activate GPRS/LTE)
→ AT+CIFSR         (get IP, e.g. 100.64.23.41)
→ AT+CIPOPEN=1,"TCP","robot.saintwings.xyz",80
→ +CIPOPEN: 1,0    (connected!)
→ Send GET request (HTTP)
→ Read until "101" found (WebSocket handshake)
→ _rxLen = 0       (clean buffer)
→ Connected
```

#### Phase 2: Registration (once, immediately after connect)

```json
Robot → Server:
{
  "type": 0,
  "robot_id": "ladybug_001",
  "payload": {
    "name": "ladybug",
    "type": "ackermann"
  }
}
```

```json
Server → Robot:
{
  "type": 2,
  "payload": { "status": "registered" }
}
```

The server now knows this robot is online. The web UI (mission_planner_v2.html) is notified and shows the robot.

#### Phase 3: Normal Operation Loop

**Robot sends telemetry every 2000ms (SIM) or 500ms (WiFi):**

```json
{
  "type": 1,
  "robot_id": "ladybug_001",
  "payload": {
    "gps": {
      "lat": 13.7563,
      "lng": 100.5018,
      "fix": 4,
      "sats": 22,
      "hacc": 0.012,
      "vacc": 0.018
    },
    "status": {
      "heading": 45.3,
      "speed": 0.0,
      "estop": false,
      "mission_running": false
    },
    "mission": {
      "state": "idle",
      "current_wp": 0,
      "total_wp": 0
    }
  }
}
```

The server forwards this to all connected web browsers. The browser updates the robot's position on the map.

**User clicks "Move" in the browser:**

```json
Server → Robot:
{
  "type": 10,
  "robot_id": "ladybug_001",
  "payload": {
    "speed": 0.3,
    "steering": -15.0
  }
}
```

The robot applies this to the motors and replies:

```json
Robot → Server:
{
  "type": 2,
  "robot_id": "ladybug_001",
  "payload": { "cmd_ack": true }
}
```

**Every 15 seconds, keepalive:**
```
Robot → Server: [WebSocket Ping frame, 6 bytes total]
Server → Robot: [WebSocket Pong frame, 2 bytes total]
```

**Every 10 seconds, server sends heartbeat:**
```json
Server → Robot:
{
  "type": 99,
  "payload": {}
}
```
(Robot ignores this — its purpose is to help the server detect dead connections)

#### Phase 4: Reconnection

If the connection drops (network glitch, modem reset, Cloudflare restart):

```
Detected: _simWs->connected() == false
Wait 12 seconds (modem may need time to recover)
Call beginSimWs() → full reconnect from Phase 1
```

The 12-second wait prevents hammering the modem with rapid reconnect attempts, which can lock it up.

### 6.3 The Server's Role

The Python `control_server.py` is a **message router**. It:

1. Accepts WebSocket connections from robots
2. Accepts WebSocket connections from web browsers (the mission planner UI)
3. Forwards telemetry from robots → browsers
4. Forwards commands from browsers → correct robot (matched by `robot_id`)
5. Logs everything to per-robot daily files

```
Browser A ──┐                    ┌── Robot ladybug_001
            │                    │
Browser B ──┤── control_server ──┤── Robot esp32-02
            │                    │
Browser C ──┘                    └── Robot ...
```

---

## Chapter 7: The SIM Path in Detail

### 7.1 Why SIM Is Different

With WiFi, the ESP32's TCP/IP stack handles everything transparently. The `WebSocketsClient` library opens a socket and you just call `send()` and read from a callback.

With SIM, there is no direct TCP socket. The cellular modem is a separate chip connected via **UART** (a serial cable, essentially). You talk to it using **AT commands** — text commands invented in 1981 for dial-up modems and still used today.

### 7.2 AT Command Basics

AT commands all start with `AT` (for "attention"). The modem replies with `OK` on success or `ERROR` on failure.

```
You send:      AT\r\n
Modem replies: OK\r\n

You send:      AT+CGMI\r\n       (get manufacturer)
Modem replies: SIMCOM_Ltd\r\nOK\r\n
```

For sending/receiving TCP data:

```
You send:      AT+CIPSEND=1,127\r\n    (I want to send 127 bytes on channel 1)
Modem replies: >                        (prompt: send your data now)
You send:      [127 bytes of data]
Modem replies: +CIPSEND: 1,127,127\r\nOK\r\n   (sent 127 of 127 bytes, OK)

You send:      AT+CIPRXGET=2,1,512\r\n  (give me up to 512 bytes from channel 1)
Modem replies: +CIPRXGET: 2,1,84,0\r\n  (here are 84 bytes, 0 remaining)
               [84 bytes of TCP data]
               OK\r\n
```

### 7.3 The CIPRXGET=1 Mode

Our code configures the modem with `AT+CIPRXGET=1` at startup. This tells the modem: "Do NOT automatically push data to me. I will ask for it when I'm ready."

Without this, the modem would push `+RECEIVE,1,84:` headers plus data at unpredictable times, which is hard to parse when you're also sending AT commands.

With `CIPRXGET=1`, our `cipRecv()` function polls explicitly:

```cpp
int SimWsClient::cipRecv(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
    String cmd = "AT+CIPRXGET=2,";
    cmd += _channel;
    cmd += ",";
    cmd += maxLen;
    sendAt(cmd.c_str());
    // parse "+CIPRXGET: 2,1,N,M" header
    // read N bytes
    // return N
}
```

### 7.4 Why Send and Receive Alternate

The SIM comms task loop looks like this:

```
loop:
    if time to send telemetry:
        cipSend(telemetry_json)      ← blocks ~300ms
    
    cipRecv(buf, 512)                ← ask for any incoming data (~200ms)
    if received data:
        parseWsFrame(buf)            ← decode WebSocket frame
        handleMessage(frame)         ← process command or ACK
    
    if time to send ping:
        sendPing()                   ← blocks ~200ms
    
    delay(50)                        ← small yield
```

Total cycle time: ~750ms minimum. That's why `SIM_TELEMETRY_INTERVAL_MS 2000` — you can't send faster than your modem can process.

---

## Chapter 8: Cloudflare Tunnel — Making the Robot Reachable Anywhere

### 8.1 The Problem

The Python server runs on a home computer at `192.168.1.166`. That address only works inside the home network. The robot (on 4G) and a user at a café cannot reach it.

Options:
1. **Port forwarding** on the home router — exposes your home IP publicly, requires a static public IP, security risk
2. **VPN** — complex setup
3. **Cloudflare Tunnel** — simple, free, secure

### 8.2 How Cloudflare Tunnel Works

```
Home PC running cloudflared
    └── Opens an OUTBOUND tunnel to Cloudflare servers
    └── Cloudflare assigns: robot.saintwings.xyz → this tunnel

Robot (anywhere) connects to robot.saintwings.xyz
    └── Cloudflare receives connection
    └── Cloudflare forwards it through the tunnel to Home PC
    └── Home PC's control_server.py handles it
```

The home PC initiates the connection to Cloudflare (outbound). No port forwarding needed. The public IP of your home doesn't matter.

### 8.3 SSL and Port 80 for SIM

Cloudflare normally redirects all HTTP (port 80) to HTTPS (port 443). But our SIM modem **cannot do SSL** — the firmware version (V11.0.01) doesn't have `AT+CSSLCFG`.

To allow the modem to connect on port 80 without being redirected to 443, we must disable "Always Use HTTPS" in the Cloudflare dashboard for `robot.saintwings.xyz`.

WiFi robots use the standard `WebSocketsClient` library with `beginSSL()`, which handles TLS properly and connects on port 443.

SIM robots use our custom `SimWsClient` which sends plain HTTP on port 80:

```cpp
// Config.h for ladybug_001 (SIM robot)
#define SIM_CONTROL_SERVER_PORT 80   // plain HTTP, no SSL
#define CONTROL_SERVER_PORT    443   // WiFi path uses SSL
```

---

## Chapter 9: The Two Tasks — FreeRTOS

### 9.1 What Is FreeRTOS?

The ESP32 runs **FreeRTOS** — a real-time operating system. Instead of one program running top-to-bottom, you create multiple **tasks** that run "simultaneously" (actually taking turns very rapidly).

Tasks are like separate programs sharing the same CPU. Each task has:
- Its own stack (memory for local variables)
- A priority (higher priority runs first)
- A CPU core affinity (ESP32 has two cores)

### 9.2 Our Two Tasks

**controlTask (Core 1, priority 5):**
- Runs every 10ms
- Reads GPS, IMU
- Computes Ackermann steering geometry
- Sends commands to CAN bus (motors)
- Runs mission planner state machine
- Handles PS2 controller input

**commsTask (Core 0, priority 3):**
- Handles WiFi or SIM connection
- Sends telemetry to server
- Receives commands from server
- Maintains WebSocket ping/pong

They communicate through **shared memory** protected by a **mutex** (mutual exclusion lock):

```cpp
// controlTask writes:
xSemaphoreTake(telemetryMutex, portMAX_DELAY);
telemetry.gps_lat = gps.lat;
telemetry.gps_lng = gps.lng;
telemetry.heading = imu.yaw;
xSemaphoreGive(telemetryMutex);

// commsTask reads:
xSemaphoreTake(telemetryMutex, portMAX_DELAY);
float lat = telemetry.gps_lat;
float lng = telemetry.gps_lng;
xSemaphoreGive(telemetryMutex);
```

The mutex prevents both tasks from reading/writing at the same time, which would cause corrupted data.

### 9.3 Why Separate Tasks for Control and Comms?

The control loop must run at exactly 10ms intervals to maintain stable motor control. Network operations are slow and unpredictable (a WiFi send might take 5ms, an AT command might take 400ms). If control and comms shared one task, a slow network call would delay the motor update loop, causing jerky or unsafe movement.

By separating them onto different cores:
- Core 1 is dedicated to real-time control — always responsive
- Core 0 handles the slow, bursty network work — doesn't affect control

---

## Chapter 10: GPS and RTK — Why Centimetre Precision

### 10.1 Standard GPS

A standard GPS receiver (like in your phone) gives ±3–5 metre accuracy. That's fine for navigation but terrible for agricultural robots — you could easily be 5 metres off the intended row.

### 10.2 RTK (Real-Time Kinematic)

**RTK** improves GPS accuracy to ±1–2 centimetres. It works by comparing measurements from a known fixed reference station with the robot's GPS:

```
Reference station (fixed, known position)
    └── Receives GPS signals
    └── Computes "error correction" data (RTCM messages)
    └── Sends RTCM corrections to robot via internet

Robot
    └── Receives GPS signals
    └── Receives RTCM corrections from reference station via NTRIP
    └── Applies corrections
    └── Gets 1-2cm accuracy
```

**NTRIP** is the protocol for streaming RTCM corrections over the internet. Our server: `110.78.0.54:2116`, mountpoint `VRS_RTCM32`.

**Fix quality levels** (shown in telemetry and on LED2):
- `1` = Standard GPS (~3m accuracy)
- `4` = RTK Fixed (~1cm accuracy) — green LED
- `5` = RTK Float (~10cm, still computing) — blue LED

### 10.3 ZED-F9P

Our GPS receiver is the **u-blox ZED-F9P** — a high-precision dual-frequency RTK receiver. It connects via UART1 at 38400 baud.

---

## Chapter 11: Putting It All Together

Here is the complete picture of data flow when you click "Move" in the web browser:

```
1. You click in mission_planner_v2.html
   
2. Browser creates JSON:
   {"type":10, "robot_id":"ladybug_001", "payload":{"speed":0.3,"steering":-15.0}}
   
3. Browser sends via WebSocket (WSS, port 443) to robot.saintwings.xyz
   
4. Cloudflare receives → forwards through tunnel to control_server.py
   
5. control_server.py:
   - Looks up "ladybug_001" in active robot connections
   - Forwards JSON to that robot's WebSocket
   - Logs: CMD_SENT to logs/ladybug_001/comm_20260610.log
   
6. For WiFi robot: WebSocketsClient library receives frame directly
   For SIM robot:  commsTask polls cipRecv() → gets 6 AT bytes + frame bytes
   
7. parseWsFrame() decodes the frame:
   - Reads opcode=0x01 (text)
   - Reads length
   - Decodes payload (server→client frames are unmasked)
   
8. handleMessage() parses JSON:
   - type=10 → store speed=0.3, steering=-15.0 in shared cmdData struct
   
9. controlTask (10ms loop) reads cmdData:
   - cmdData.speed = 0.3 m/s
   - cmdData.steering = -15.0°
   
10. Ackermann geometry computation:
    - Inner wheel angle: arctan(wheelbase / (radius - track/2))
    - Outer wheel angle: arctan(wheelbase / (radius + track/2))
    - RPM = speed / (π × wheel_diameter)
    
11. CAN bus commands sent to motors:
    - ZLAC drive controller: speed RPM to front/rear motors
    - GIM8108 or ODrive: angle to left/right steering motors
    
12. Robot moves

13. On next telemetry cycle:
    Robot → Server: {"type":1, ... "status":{"speed":0.3, ...}}
    
14. Server → Browser: telemetry forwarded
    
15. Map shows robot moving
```

Total latency (SIM path, worst case):
- Command transmission: ~300ms (AT+CIPSEND)
- Robot polling: up to 2000ms (next cipRecv cycle)
- Motor response: ~10ms (next control loop)
- Telemetry feedback: up to 2000ms (next send cycle)
- **Total: up to ~4 seconds** for the UI to show the robot responding

For WiFi, this drops to ~100ms total.

---

## Quick Reference

### WebSocket Frame Sizes in This Project

| Message | Direction | Approx Size | Length Field |
|---------|-----------|-------------|--------------|
| Ping | Robot→Server | 6 bytes | Short (0) |
| Pong | Server→Robot | 2 bytes | Short (0) |
| Register | Robot→Server | ~100 bytes | Short |
| Telemetry | Robot→Server | ~200 bytes | Extended 16-bit |
| Move command | Server→Robot | ~80 bytes | Short |
| ACK | Robot→Server | ~60 bytes | Short |

### Key Timing Constants

| Constant | Value | Where | Why |
|----------|-------|-------|-----|
| `SIM_TELEMETRY_INTERVAL_MS` | 2000ms | Config.h | Modem AT overhead |
| `ROBOT_TELEMETRY_INTERVAL_MS` | 500ms | Config.h | WiFi is fast |
| Ping interval | 15s | sim_ws_client.cpp | Under Cloudflare 100s limit |
| Reconnect delay | 12s | robot_client.cpp | Modem recovery time |
| Handshake timeout | 10s | sim_ws_client.cpp | Give modem time to connect |
| Extra drain delay | 500ms | sim_ws_client.cpp | Consume Cloudflare extra headers |

### Files and Their Purpose

| File | Purpose |
|------|---------|
| `src/sim_ws_client.cpp` | SIM modem WebSocket client (AT commands) |
| `src/robot_client.cpp` | High-level server communication (sends telemetry, receives commands) |
| `src/main.cpp` | Main firmware: FreeRTOS tasks, sensors, motor control |
| `control_server.py` | Python WebSocket server: routes messages between robots and browsers |
| `robot_logger.py` | Per-robot daily log files (telemetry + communication events) |
| `mission_planner_v2.html` | Web UI: map, telemetry display, mission planning |
| `include/Config.h` | Configuration for ladybug_001 (SIM robot) |
| `include/Config_2.h` | Configuration for esp32-02 (WiFi robot) |
| `include/RobotConfig.h` | Selects which config is active (`#include "Config.h"`) |
