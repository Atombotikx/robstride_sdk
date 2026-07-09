#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../build')))
import pyrobstride
import time
import threading
import math

def rx_thread_func(transport, motors):
    """
    Background thread to continuously poll the shared CAN interface
    and dispatch received frames to all motor instances.
    """
    pyrobstride.Motor.receive_all(transport, motors)

def setup_motor(motor, name):
    print(f"[{name}] Pinging motor...")
    if motor.ping() != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to ping motor. Is it connected and powered?")
        return False
    
    status = motor.get_status()
    print(f"[{name}] Ping successful! MCU ID: 0x{status.mcu_id:X}")
    time.sleep(0.1)

    print(f"[{name}] Clearing errors...")
    if motor.clear_error() != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to clear errors.")
        return False
    time.sleep(0.1)

    print(f"[{name}] Setting current position as absolute zero...")
    if motor.set_zero() != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to set zero position.")
        return False
    time.sleep(0.1)

    print(f"[{name}] Setting up OPERATION mode...")
    if motor.setup_mode(pyrobstride.RunMode.OPERATION) != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to enter OPERATION mode.")
        return False

    print(f"[{name}] Enabling pyrobstride.Motor...")
    if motor.enable() != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to enable motor.")
        return False
    time.sleep(0.5)
    return True

def main():
    print("Connecting to CAN interface...")
    transport = pyrobstride.SocketCANTransport("can0")
    if transport.connect() != pyrobstride.Result.SUCCESS:
        print("Failed to connect to can0. Is it up?")
        return

    # 1. Initialize Two Motors on the shared transport
    motor5 = pyrobstride.Motor(5, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
    motor6 = pyrobstride.Motor(6, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
    motors = [motor5, motor6]

    # 2. Start shared receive thread
    rx_thread = threading.Thread(target=rx_thread_func, args=(transport, motors), daemon=True)
    rx_thread.start()

    # 3. Setup both motors
    if not setup_motor(motor5, "Motor 5") or not setup_motor(motor6, "Motor 6"):
        return

    print("Starting high-frequency dual sweep. Press Ctrl+C to stop.")
    try:
        start_time = time.time()
        Kp = 5.0
        Kd = 0.5
        
        last_angle5 = None
        last_angle6 = None

        while True:
            t = time.time()
            # Smooth sine wave from 0 to 2*PI (0 to 360 degrees)
            target_rad = math.pi * (1.0 - math.cos(2.0 * math.pi * 0.25 * (t - start_time)))
            
            # Send MIT control packet to BOTH motors and check for library-level overrun aborts
            # We check res5 immediately so we don't command motor6 if motor5 just went limp (prevents jerks)
            res5 = motor5.send_mit_control(0.0, target_rad, 0.0, Kp, Kd)
            if res5 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 5 with error: {res5.name}! Aborting loop.")
                break
                
            res6 = motor6.send_mit_control(0.0, target_rad, 0.0, Kp, Kd)
            if res6 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 6 with error: {res6.name}! Aborting loop.")
                break
            
            # Read status and update terminal in-place on a single line using carriage return (\r)
            status5 = motor5.get_status()
            status6 = motor6.get_status()
            
            if status5.angle != last_angle5 or status6.angle != last_angle6:
                s5 = status5
                s6 = status6
                # Formatted compactly to fit on a single line without wrapping
                out = (f"\rM5 Act:{s5.angle:>5.2f} Vel:{s5.velocity:>5.2f} Trq:{s5.torque:>4.1f} Tmp:{s5.temperature:>2.0f} F:{s5.fault_value:X} | "
                       f"M6 Act:{s6.angle:>5.2f} Vel:{s6.velocity:>5.2f} Trq:{s6.torque:>4.1f} Tmp:{s6.temperature:>2.0f} F:{s6.fault_value:X} ")
                print(out, end="", flush=True)
                
                last_angle5 = status5.angle
                last_angle6 = status6.angle
                
            # Yield slightly to prevent 100% CPU lock, targeting >1000Hz loop
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping loop due to Ctrl+C...")

    print("Stopping Motors...")
    motor5.stop()
    motor6.stop()
    transport.disconnect()
    rx_thread.join(timeout=1.0)
    print("Done.")

if __name__ == "__main__":
    main()
