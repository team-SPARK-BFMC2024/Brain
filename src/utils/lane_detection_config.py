from dataclasses import dataclass
import numpy as np

@dataclass
class LaneDetectionConfig:
    white_threshold: int = 200
    yellow_h_low: int = 90
    yellow_h_high: int = 110
    yellow_s_low: int = 100
    yellow_v_low: int = 100
    kernel_size: int = 3
    canny_low: int = 50
    canny_high: int = 150
    roi_height_ratio: float = 0.4
    roi_bottom_width_ratio: float = 0.85
    roi_top_width_ratio: float = 0.07
    hough_rho: int = 2
    hough_theta: float = np.pi / 180
    hough_threshold: int = 15
    hough_min_line_length: int = 10
    hough_max_line_gap: int = 20
    slope_threshold: float = 0.5
    pid_kp: float = 0.2
    pid_ki: float = 0.0
    pid_kd: float = 0.1
    pid_min_output: float = -20
    pid_max_output: float = 20
