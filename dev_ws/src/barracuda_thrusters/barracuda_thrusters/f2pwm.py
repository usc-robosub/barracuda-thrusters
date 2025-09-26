# the f2pwm module holds information about the T200 thruster 
# and about the configuration of the PWM pins on the microcontroller,
# and uses them to convert a force value (N) to a duty cycle 
# (between 0 and 2^(pwm pin bit resolution))

import csv
import os
from bisect import bisect_left
from ament_index_python.packages import get_package_share_directory

PWM_FREQ = 333
PWM_BIT_RES = 8;
T200_STOPPED_PWM_WIDTH_US = 1500

# creates lookup table implemented with two lists where pwm_widths[i] gives the pwm width in microseconds
# required to output a force of force_vals[i] in Newtons 
def _create_lut_lists(csv_filename='t200_18v_data.csv'):
    pwm_widths = []
    force_vals = []
    package_share_dir = get_package_share_directory('barracuda_thrusters')
    csv_path = os.path.join(package_share_dir, csv_filename)
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            pwm_width, force_newtons = int(row['PWM (µs)']), float(row['Force (N)'])
            pwm_widths.append(pwm_width)
            force_vals.append(force_newtons)
            # print(pwm_width, force_newtons)
    return pwm_widths, force_vals
    
# convert force (newtons) to PWM width (microseconds)
def _to_us(force_newtons):
    if force_newtons == 0:
        return T200_STOPPED_PWM_WIDTH_US
    # Find index of leftmost value greater than or equal to force_newtons
    lut_idx = bisect_left(_force_vals, force_newtons)
    if lut_idx == 0:
        raise ValueError("less than lowest allowed force")
    if lut_idx == len(_force_vals):
        raise ValueError("greater than highest allowed force")

    # Get the surrounding values for interpolation
    force_under = _force_vals[lut_idx - 1]
    force_over = _force_vals[lut_idx]
    pwm_width_under = _pwm_widths[lut_idx - 1]
    pwm_width_over = _pwm_widths[lut_idx]
    
    # Linear interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
    pwm_us_interpolated = pwm_width_under + (force_newtons - force_under) * ((pwm_width_over - pwm_width_under) / (force_over - force_under))
    return pwm_us_interpolated

def to_duty_cycle(force_newtons):
    # pwm_us / pwm_period = pwm_us * pwm_frequency
    # need to divide pwm_us * pwm_frequency by 10^6 to account for us/s difference, 
    # dividing by 10^3 twice to keep intermediate values smaller
    pwm_width_us = _to_us(force_newtons)
    return int(round(((pwm_width_us / 10**3) * (PWM_CONFIG['FREQ'] / 10**3)) * (2**PWM_CONFIG['BIT_RES'])))

# create lut on module import
_pwm_widths, _force_vals = _create_lut_lists()

__all__ = ['PWM_PIN_CONFIG', 'T200_STOPPED_PWM_WIDTH_US', 'to_duty_cycle']