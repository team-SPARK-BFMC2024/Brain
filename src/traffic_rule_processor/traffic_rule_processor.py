import time

class TrafficRuleProcessor:
    def __init__(self):
        self.CLASS_NAMES = [
            'car', 'closed-road-stand', 'crosswalk-sign', 'highway-entry-sign',
            'highway-exit-sign', 'no-entry-road-sign', 'one-way-road-sign',
            'parking-sign', 'parking-spot', 'pedestrian', 'priority-sign',
            'round-about-sign', 'stop-line', 'stop-sign', 'traffic-light',
            'traffic-green', 'traffic-red', 'traffic-yellow'
        ]
        # Cập nhật tốc độ cho các tình huống khác nhau
        self.SPEEDS = {
            'highway': {
                'normal': 250,    # Tăng từ 50 lên 250
                'caution': 200,   # Tăng từ 45 lên 200
                'slow': 180       # Tăng từ 40 lên 180
            },
            'city': {
                'normal': 200,    # Tăng từ 30 lên 200
                'caution': 180,   # Tăng từ 25 lên 180
                'slow': 150       # Tăng từ 20 lên 150
            }
        }

    def get_default_speed(self, vehicle_state, speed_type='normal'):
        zone = vehicle_state.zone_type
        return self.SPEEDS[zone][speed_type]

    def process_traffic_signal(self, signal_class, vehicle_state):
        try:
            if signal_class == self.CLASS_NAMES.index('traffic-red'):
                return 'stop', 0
            elif signal_class == self.CLASS_NAMES.index('traffic-yellow'):
                return 'caution', self.get_default_speed(vehicle_state, 'caution')
            elif signal_class == self.CLASS_NAMES.index('traffic-green'):
                return 'proceed', self.get_default_speed(vehicle_state, 'normal')
            return None, self.get_default_speed(vehicle_state, 'normal')
        except ValueError as e:
            print(f"Error processing traffic signal: {e}")
            return None, self.get_default_speed(vehicle_state, 'normal')

    def process_traffic_sign(self, sign_class, vehicle_state):
        try:
            current_time = time.time()
            if sign_class == self.CLASS_NAMES.index('stop-sign'):
                if vehicle_state.last_state != 'stop':
                    vehicle_state.stop_timer = current_time
                    vehicle_state.last_state = 'stop'
                    return 'stop', 0
                elif current_time - vehicle_state.stop_timer >= 3:
                    vehicle_state.last_state = None
                    return 'proceed', self.get_default_speed(vehicle_state, 'normal')
                return 'stop', 0
            elif sign_class == self.CLASS_NAMES.index('parking-sign'):
                vehicle_state.parking_mode = True
                return 'parking', self.get_default_speed(vehicle_state, 'slow')
            elif sign_class == self.CLASS_NAMES.index('crosswalk-sign'):
                return 'caution', self.get_default_speed(vehicle_state, 'caution')
            elif sign_class == self.CLASS_NAMES.index('highway-entry-sign'):
                vehicle_state.zone_type = 'highway'
                return 'proceed', self.SPEEDS['highway']['normal']
            elif sign_class == self.CLASS_NAMES.index('highway-exit-sign'):
                vehicle_state.zone_type = 'city'
                return 'proceed', self.SPEEDS['city']['normal']
            elif sign_class == self.CLASS_NAMES.index('round-about-sign'):
                vehicle_state.in_roundabout = True
                return 'caution', self.get_default_speed(vehicle_state, 'caution')
            elif sign_class == self.CLASS_NAMES.index('priority-sign'):
                return 'proceed', self.get_default_speed(vehicle_state, 'normal')
            elif sign_class == self.CLASS_NAMES.index('no-entry-road-sign'):
                return 'stop', 0
            elif sign_class == self.CLASS_NAMES.index('one-way-road-sign'):
                return 'proceed', self.get_default_speed(vehicle_state, 'normal')
            return None, self.get_default_speed(vehicle_state, 'normal')
        except ValueError as e:
            print(f"Error processing traffic sign: {e}")
            return None, self.get_default_speed(vehicle_state, 'normal')

    def process_pedestrian(self, detection, frame_height):
        try:
            x1, y1, x2, y2 = detection[:4]
            relative_pos = y2 / frame_height
            if relative_pos > 0.8:
                return 'stop', 0
            elif relative_pos > 0.6:
                return 'caution', 150  # Tăng từ 20 lên 150
            else:
                return 'caution', 180  # Tăng từ 25 lên 180
        except Exception as e:
            print(f"Error processing pedestrian detection: {e}")
            return 'caution', 150  # Tăng từ 20 lên 150

    def process_vehicle(self, detection, lane_type, frame_width, frame_height):
        try:
            x1, y1, x2, y2 = detection[:4]
            vehicle_center_x = (x1 + x2) / 2
            vehicle_width = x2 - x1
            vehicle_height = y2 - y1
            relative_distance = 1 - (y2 / frame_height)
            
            # Tăng base_following_speed từ 30 lên 180
            base_following_speed = 180
            
            if relative_distance < 0.2:
                following_speed = base_following_speed * 0.6  # 108
            elif relative_distance < 0.4:
                following_speed = base_following_speed * 0.8  # 144
            else:
                following_speed = base_following_speed  # 180
            
            in_our_lane = (vehicle_center_x > frame_width * 0.4 and 
                          vehicle_center_x < frame_width * 0.6)
            
            if in_our_lane:
                if lane_type == 'continuous':
                    return 'follow', int(following_speed)
                else:
                    if vehicle_height > frame_height * 0.4:
                        return 'overtake', 220  # Tăng từ 40 lên 220
                    else:
                        return 'follow', int(following_speed)
            return None, None
        except Exception as e:
            print(f"Error processing vehicle detection: {e}")
            return None, None

    def is_safe_following_distance(self, vehicle_height, frame_height):
        relative_size = vehicle_height / frame_height
        return relative_size < 0.4

    def calculate_following_speed(self, relative_distance, base_speed):
        if relative_distance < 0.2:
            return base_speed * 0.6
        elif relative_distance < 0.4:
            return base_speed * 0.8
        return base_speed
