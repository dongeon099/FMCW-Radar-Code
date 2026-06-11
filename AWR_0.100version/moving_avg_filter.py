from collections import deque #deque: 자동으로 오래된 값을 버리는 리스트
import numpy as np

class MovingAvergeFilter:
    def __init__(self, window_size = 5):
        self.x_buffer = deque(maxlen = window_size)
        self.y_buffer = deque(maxlen = window_size)
        self.v_buffer = deque(maxlen = window_size)
        self.distance_buffer = deque(maxlen = window_size)
    
    def update(self,obj):
        if obj is None:
            return None
        
        self.x_buffer.append(obj["x"])
        self.y_buffer.append(obj["y"])
        self.v_buffer.append(obj["v"])
        self.distance_buffer.append(obj["distance"])

        return {
            "id": obj["id"],
            "x": np.mean(self.x_buffer),
            "y": np.mean(self.y_buffer),
            "z": obj["z"],
            "v": np.mean(self.v_buffer),
            "distance": np.mean(self.distance_buffer),
        }