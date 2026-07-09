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
    print(f"[{name}] Enabling Motor in MIT mode...")
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

    # Initialize motors for MIT Protocol
    motor5 = pyrobstride.Motor(5, 0, pyrobstride.Protocol.MIT, pyrobstride.PROFILE_RS00, transport)
    motor6 = pyrobstride.Motor(6, 0, pyrobstride.Protocol.MIT, pyrobstride.PROFILE_RS00, transport)
    motors = [motor5, motor6]

    rx_thread = threading.Thread(target=rx_thread_func, args=(transport, motors), daemon=True)
    rx_thread.start()

    if not setup_motor(motor5, "Motor 5") or not setup_motor(motor6, "Motor 6"):
        transport.disconnect()
        return

    print("Starting High-Frequency MIT Velocity Control. Press Ctrl+C to stop.")
    
    start_time = time.time()
    last_vel5 = 0.0
    last_vel6 = 0.0
    
    Kp = 0.0  # Zero stiffness for velocity control
    Kd = 5.0  # High damping tracks velocity
    
    try:
        while True:
            t = time.time() - start_time
            # 20-second period triangle wave hitting MAX VELOCITY (33.0 rad/s)
            phase = t % 20.0
            if phase < 5.0:
                target_spd = phase * 6.6
            elif phase < 15.0:
                target_spd = (10.0 - phase) * 6.6
            else:
                target_spd = (phase - 20.0) * 6.6
            
            # send_mit_control(torque, pos, vel, kp, kd)
            res5 = motor5.send_mit_control(0.0, 0.0, target_spd, Kp, Kd)
            if res5 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 5 with error: {res5.name}! Aborting loop.")
                break
                
            res6 = motor6.send_mit_control(0.0, 0.0, target_spd, Kp, Kd)
            if res6 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 6 with error: {res6.name}! Aborting loop.")
                break
                
            status5 = motor5.get_status()
            status6 = motor6.get_status()
            if status5.velocity != last_vel5 or status6.velocity != last_vel6:
                out = (f"\rM5 TgtSpd:{target_spd:>6.2f} ActSpd:{status5.velocity:>6.2f} Pos:{status5.angle:>6.2f} Trq:{status5.torque:>5.1f} | "
                       f"M6 TgtSpd:{target_spd:>6.2f} ActSpd:{status6.velocity:>6.2f} Pos:{status6.angle:>6.2f} Trq:{status6.torque:>5.1f}")
                print(out, end="", flush=True)
                last_vel5 = status5.velocity
                last_vel6 = status6.velocity
                
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
