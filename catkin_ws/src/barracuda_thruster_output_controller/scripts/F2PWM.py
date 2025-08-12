import rospkg
import csv
import os
from bisect import bisect_left

NEWTONS_PER_KGF = 9.80665
STOPPED_PWM_WIDTH = 1500

class F2PWM:
    def __init__(self, csv_filename, interpolation_mode):
        rospack = rospkg.RosPack()
        csv_path = os.path.join(rospack.get_path('barracuda_thruster_output_controller'), 'data', csv_filename)
        self.pwm_widths = []
        self.force_vals = []
        self._create_lut_lists(csv_path)
    
    # convert force (newtons) to PWM width (microseconds)
    def to_us(self, force_newtons):
        if force_newtons == 0:
            return STOPPED_PWM_WIDTH
        # Find index of leftmost value greater than or equal to force_newtons
        lut_idx = bisect_left(self.force_vals, force_newtons)
        if lut_idx == 0:
            raise ValueError("less than lowest allowed force")
        if lut_idx == len(self.force_vals):
            raise ValueError("greater than highest allowed force")

        # Get the surrounding values for interpolation
        force_under = self.force_vals[lut_idx - 1]
        force_over = self.force_vals[lut_idx]
        pwm_width_under = self.pwm_widths[lut_idx - 1]
        pwm_width_over = self.pwm_widths[lut_idx]
        
        # Linear interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        pwm_us_interpolated = pwm_width_under + (force_newtons - force_under) * ((pwm_width_over - pwm_width_under) / (force_over - force_under))
        return pwm_us_interpolated

    # creates lookup table between force values (N) and PWM widths (microseconds)
    def _create_lut_lists(self, csv_path):
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                pwm_width, force_newtons = int(row['PWM (µs)']), float(row['Force (N)'])
                self.pwm_widths.append(pwm_width)
                self.force_vals.append(force_newtons)
                print(pwm_width, force_newtons)