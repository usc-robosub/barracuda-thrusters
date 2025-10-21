import argparse
import time

import f2pwm
import teensy

n_thrusters=8

def _write_to_thruster_reg( thruster_idx, val):
    try:
        # writes to teensy 0 for thrusters 0-3, teensy 1 for thrusters 4-7
        teensy.write_i2c_16(
            teensy.i2c_addresses[thruster_idx // (n_thrusters // 2)], 
            teensy.thruster_registers[thruster_idx % (n_thrusters // 2)], 
            val
        )
    except Exception as e:  
        print(f"Write failed at addr {teensy.i2c_addresses[thruster_idx // (n_thrusters // 2)]:#04x}, reg {teensy.thruster_registers[thruster_idx % (n_thrusters // 2)]}: {e}")

def main():
    parser = argparse.ArgumentParser(description="thruster params")
    parser.add_argument("thruster_id", help="thruster to turn")
    parser.add_argument("turn_force", help="turn force")
    parser.add_argument("turn_time_ms", help="time of turning(ms)")

    args = parser.parse_args()

    print("thruster_id", args.thruster_id)
    print("turn_force", args.turn_force)
    print("turn_time_ms", args.turn_time_ms)

    _write_to_thruster_reg(args.thruster_id, f2pwm.to_duty_cycle(force_newtons=args.turn_force))
    time.sleep(args.turn_time_ms/1000)
    _write_to_thruster_reg(args.thruster_id, f2pwm.to_duty_cycle(force_newtons=0))
if __name__ == "__main__":
    main()