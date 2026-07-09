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

    print(f"[{name}] Setting up POSITION (Profile Position) mode...")
    # mode, max_vel (rad/s), acc_set (rad/s^2), dec_set (rad/s^2)
    if motor.setup_mode(pyrobstride.RunMode.POSITION, 10.0, 5.0, 5.0) != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to enter POSITION mode.")
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

    # Initialize motors
    motor5 = pyrobstride.Motor(5, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
    motor6 = pyrobstride.Motor(6, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
    motors = [motor5, motor6]

    # 2. Start the RX Thread
    rx_thread = threading.Thread(target=rx_thread_func, args=(transport, motors), daemon=True)
    rx_thread.start()

    # 3. Setup and Enable Motors
    if not setup_motor(motor5, "Motor 5") or not setup_motor(motor6, "Motor 6"):
        transport.disconnect()
        return

    # 4. Position Control Loop (Stepping between points)
    print("Starting High-Frequency Position Control. Press Ctrl+C to stop.")
    
    start_time = time.time()
    last_angle5 = 0.0
    last_angle6 = 0.0
    
    try:
        while True:
            t = time.time() - start_time
            # Create a smooth sine wave from 0 to 2*PI (0 to 360 degrees)
            # Period = 4 seconds
            target = math.pi * (1.0 - math.cos(2.0 * math.pi * 0.25 * t))
            
            # Send position target
            res5 = motor5.send_position_control(target)
            if res5 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 5 with error: {res5.name}! Aborting loop.")
                break
                
            res6 = motor6.send_position_control(target)
            if res6 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 6 with error: {res6.name}! Aborting loop.")
                break
                
            status5 = motor5.get_status()
            status6 = motor6.get_status()
            if status5.angle != last_angle5 or status6.angle != last_angle6:
                out = (f"\rM5 Tgt:{target:>5.2f} Act:{status5.angle:>5.2f} Vel:{status5.velocity:>5.2f} Trq:{status5.torque:>4.1f} | "
                       f"M6 Tgt:{target:>5.2f} Act:{status6.angle:>5.2f} Vel:{status6.velocity:>5.2f} Trq:{status6.torque:>4.1f}")
                print(out, end="", flush=True)
                last_angle5 = status5.angle
                last_angle6 = status6.angle
                
            # Yield slightly targeting ~1000Hz loop
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping loop due to Ctrl+C...")

    print("Stopping Motors...")
    motor5.stop()
    motor6.stop()
    transport.disconnect()

if __name__ == '__main__':
    main()
