#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../build')))
import pyrobstride
import time
import threading
import math

def rx_thread_func(transport, motors):
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
        return False
    time.sleep(0.1)

    print(f"[{name}] Setting current position as absolute zero...")
    if motor.set_zero() != pyrobstride.Result.SUCCESS:
        return False
    time.sleep(0.1)

    print(f"[{name}] Setting up CURRENT (Direct Current) mode...")
    if motor.setup_mode(pyrobstride.RunMode.CURRENT) != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to enter CURRENT mode.")
        return False

    print(f"[{name}] Enabling pyrobstride.Motor...")
    if motor.enable() != pyrobstride.Result.SUCCESS:
        return False
    time.sleep(0.5)
    return True

def main():
    print("Connecting to CAN interface...")
    transport = pyrobstride.SocketCANTransport("can0")
    if transport.connect() != pyrobstride.Result.SUCCESS:
        print("Failed to connect to can0. Is it up?")
        return

    motor5 = pyrobstride.Motor(5, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
    motor6 = pyrobstride.Motor(6, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
    motors = [motor5, motor6]

    rx_thread = threading.Thread(target=rx_thread_func, args=(transport, motors), daemon=True)
    rx_thread.start()

    if not setup_motor(motor5, "Motor 5") or not setup_motor(motor6, "Motor 6"):
        transport.disconnect()
        return

    print("Starting High-Frequency Direct Current Control. Press Ctrl+C to stop.")
    
    start_time = time.time()
    last_torque5 = 0.0
    last_torque6 = 0.0
    
    try:
        while True:
            t = time.time() - start_time
            # Generate a smooth 0.5Hz sine wave with amplitude of 2.0 Amps
            target_iq = 2.0 * math.sin(2.0 * math.pi * 0.5 * t)
            
            res5 = motor5.send_current_control(target_iq)
            if res5 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 5 with error: {res5.name}! Aborting loop.")
                break
                
            res6 = motor6.send_current_control(target_iq)
            if res6 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 6 with error: {res6.name}! Aborting loop.")
                break
                
            status5 = motor5.get_status()
            status6 = motor6.get_status()
            if status5.torque != last_torque5 or status6.torque != last_torque6:
                out = (f"\rM5 TgtCur:{target_iq:>6.2f}A ActTrq:{status5.torque:>5.2f}Nm Pos:{status5.angle:>5.2f} Vel:{status5.velocity:>5.2f} | "
                       f"M6 TgtCur:{target_iq:>6.2f}A ActTrq:{status6.torque:>5.2f}Nm Pos:{status6.angle:>5.2f} Vel:{status6.velocity:>5.2f}")
                print(out, end="", flush=True)
                last_torque5 = status5.torque
                last_torque6 = status6.torque
            
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
