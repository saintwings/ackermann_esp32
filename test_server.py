import asyncio, websockets, json

async def test():
    url = 'wss://robot.saintwings.xyz'
    print(f'Connecting to {url} ...')

    async with websockets.connect(url) as ws:
        print('Connected!')

        # Send robot registration — same packet the ESP32 sends on connect
        reg = json.dumps({
            "type": "register",
            "robot_id": "test-client",
            "robot_name": "TestBot",
            "robot_type": "ackermann"
        })
        await ws.send(reg)
        print('Sent registration')

        # Wait for server acknowledgement
        try:
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            print('Server reply:', msg)
        except asyncio.TimeoutError:
            print('No reply from server within 5s (server may not send ack — connection still OK)')

asyncio.run(test())
