# AWR6843 UDP Visualization Flow

## Raspberry Pi

Run:

```bash
python main_rpi.py
```

Flow:

```text
AWR6843 Radar
-> Raspberry Pi serial receive
-> TLV Parsing
-> DBSCAN Clustering
-> EKF Tracking
-> Lane Classification
-> TTC / Risk Decision
-> SPI turn-signal exchange
-> UDP JSON Packet Send
```

`network_sender.py` sends only processed object-center results. It does not send
the raw point cloud.

Payload:

```json
{
  "frame": 1,
  "timestamp": 12345.67,
  "turn_signal": 0,
  "selected_lane": 0,
  "objects": [
    {
      "track_id": 1,
      "lane": 3,
      "x": 1.2,
      "y": 8.4,
      "v": -2.1,
      "ttc": 4.0,
      "risk": 1
    }
  ]
}
```

## Laptop

Run:

```bash
python laptop_visualizer.py
```

Flow:

```text
Laptop UDP Receive
-> JSON Decode
-> stale frame discard
-> object dict conversion
-> advanced_visualizer.py real-time display
```

UDP settings are in `config.py`:

```python
UDP_ENABLED = True
LAPTOP_IP = "192.168.32.64"
UDP_PORT = 5005
UDP_RECV_BUFFER = 8192
```

The receiver keeps running on missing packets, invalid JSON, stale frames, and
empty object lists.
