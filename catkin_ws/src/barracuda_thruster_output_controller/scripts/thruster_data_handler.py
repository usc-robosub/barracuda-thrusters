#!/usr/bin/python3

import os
import sys
import numpy as np
import pandas as pd
import rospkg

class ThrusterDataHandler:
    """
    Handler for T200 thruster data that provides conversion between kgf force and PWM values.
    """
    
    def __init__(self, spreadsheet_path=None):
        """
        Initialize the handler with the path to the T200 performance data spreadsheet.
        
        Args:
            spreadsheet_path (str): Path to the Excel spreadsheet with thruster data.
                                   If None, attempts to find it in the package's data directory.
        """
        if spreadsheet_path is None:
            # Get the package path using ROS package utilities
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('barracuda_thruster_output_controller') 
            
            # Construct path to the data file (in the data directory)
            spreadsheet_path = os.path.join(package_path, 'data', 
                                           'T200-Public-Performance-Data-10-20V-September-2019.xlsx')
        
        self.spreadsheet_path = spreadsheet_path
        
        # Load and process the data from the Excel file
        self._load_data()
    
    def _load_data(self):
        """
        Load kgf and PWM data from the 18V sheet in the Excel spreadsheet.
        """
        try:   
            df = pd.read_excel(self.spreadsheet_path, sheet_name="18 V")
            
            self.kgf_values = df[" Force (Kg f)"].values
            self.pwm_values = df[" PWM (µs)"].values
            
        except Exception as e:
            raise Exception(f"Error loading thruster data: {e}")
    
    def kgf_to_pwm_us(self, kgf_input):
        """
        Convert kgf force value to PWM (μs) value using linear interpolation.
        
        Args:
            kgf_input (float): The input force in kgf to convert
            
        Returns:
            float: The corresponding PWM value in microseconds (μs)
        
        Raises:
            ValueError: If the input kgf value is outside the range of the available data
        """
        
        # Special case: return 1500us if force is zero (at the center of the range of pulse widths that translate to zero force)
        if (kgf_input == 0):
            return 1500
        
        # Check if the exact value exists in our dataset
        if kgf_input in self.kgf_values:
            index = np.where(self.kgf_values == kgf_input)[0][0]
            return self.pwm_values[index]
        
        # Use binary search to find where the input value would fit
        index = np.searchsorted(self.kgf_values, kgf_input)
        
        # Handle edge cases where input is outside data range
        if index == 0:
            raise ValueError(f"Input value {kgf_input} kgf is below the minimum value in the dataset ({self.kgf_values[0]} kgf)")
        elif index == len(self.kgf_values):
            raise ValueError(f"Input value {kgf_input} kgf is above the maximum value in the dataset ({self.kgf_values[-1]} kgf)")
        
        # Get the surrounding values for interpolation
        kgf_low = self.kgf_values[index - 1]
        kgf_high = self.kgf_values[index]
        pwm_low = self.pwm_values[index - 1]
        pwm_high = self.pwm_values[index]
        
        # Linear interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        pwm_us_interpolated = pwm_low + (kgf_input - kgf_low) * ((pwm_high - pwm_low) / (kgf_high - kgf_low))
        
        return pwm_us_interpolated

if __name__ == "__main__":
    try:
        handler = ThrusterDataHandler()
        
        # Test with a sample kgf value given as a command line arg, value defaults to 0 if no arg/there's an issue with the arg
        try:
            kgf_test = float(sys.argv[1])
        except (ValueError, IndexError):
            kgf_test = 0
        pwm_result = handler.kgf_to_pwm_us(kgf_test)
        
        print(f"For {kgf_test} kgf at 18V, the PWM value is {pwm_result:.2f} μs")
    except Exception as e:
        print(f"Error: {e}")