import numpy as np
from simple_pid import PID
pid = PID(1, 0.1, 0.05, setpoint=1)

class camera_pid:
    def __init__(self):
        self.midpoint = 0
        self.index = 0
        self.ltool_history = np.zeros(300)
        self.rtool_history = np.zeros(300) # 30 frames/ sec for 10 seconds.
    
    def update(self, ltool, rtool):
        self.index += 1
        self.ltool_history[self.index] = ltool
        self.ltool_history[self.index] = rtool
        
        left_tool_movement = np.sum(np.diff(self.ltool_history))
        right_tool_movement = np.sum(np.diff(self.ltool_history))

        relative_left = left_tool_movement/ (left_tool_movement + right_tool_movement);
        relative_right = 1.0 - relative_left
        self.midpoint = (ltool* relative_left + rtool * relative_right)/2
        return self.midpoint

