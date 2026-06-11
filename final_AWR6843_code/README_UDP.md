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
  "turn_signal": 1,
  "selected_lane": 0,
  "vehicle_state": {
    "ego_speed_kmh": 18,
    "ego_speed_mps": 5.0,
    "steering_angle_deg": 12.5,
    "turn_signal": 1,
    "miso_valid": true,
    "spi_sequence": 42,
    "spi_valid_count": 100,
    "spi_invalid_count": 2
  },
  "advice": {
    "target_lane": "right",
    "lane_change_possible": true,
    "reason": "clear"
  },
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
-> vehicle state / lane-change advice display
-> advanced_visualizer.py real-time display
```

UDP settings are in `config.py`:

```python
UDP_ENABLED = True
LAPTOP_IP = "172.21.31.240"
UDP_PORT = 5005
UDP_RECV_BUFFER = 8192
```

The receiver keeps running on missing packets, invalid JSON, stale frames, and
empty object lists.
