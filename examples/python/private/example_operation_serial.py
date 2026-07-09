#!/usr/bin/env python3
"""
example_operation_serial.py — Private protocol operation mode over Serial (UART).

Hardware setup:
  - Motor connected via USB-to-UART adapter (default: /dev/ttyUSB0, 921600 baud)
  - Motor IDs: 5 and 6
  - Protocol: PRIVATE
  - Change SERIAL_PORT below to match your actual device node.
"""
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../build')))
import pyrobstride
import time
import threading
import math
import signal

SERIAL_PORT = "/dev/ttyUSB0"  # Change this to your serial port (e.g. /dev/ttyACM0)

keep_running = True

def sigint_handler(sig, frame):
    global keep_running
    keep_running = False

signal.signal(signal.SIGINT, sigint_handler)

def rx_thread_func(transport, motors):
    """
    Background thread to continuously poll the shared serial interface
    and dispatch received frames to all motor instances.
    """
    pyrobstride.Motor.receive_all(transport, motors)

def setup_motor(motor, name):
    print(f"[{name}] Pinging motor...")
    res = motor.ping()
    if res != pyrobstride.Result.SUCCESS:
        print(f"[{name}] ping() returned {res}")
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

    print(f"[{name}] Enabling Motor...")
    if motor.enable() != pyrobstride.Result.SUCCESS:
        print(f"[{name}] Failed to enable motor.")
        return False
    time.sleep(0.5)
    return True

def main():
    global keep_running

    print(f"Connecting to Serial interface on {SERIAL_PORT}...")
    transport = pyrobstride.SerialTransport(SERIAL_PORT)
    if transport.connect() != pyrobstride.Result.SUCCESS:
        print(f"Failed to connect to {SERIAL_PORT}. Is it plugged in and have permissions?")
        print("Hint: Try: sudo chmod 666 /dev/ttyUSB0")
        return

    # Initialize ONE Motor on the shared serial transport
    motor5 = pyrobstride.Motor(5, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
    motors = [motor5]

    # Start shared receive thread
    rx_thread = threading.Thread(target=rx_thread_func, args=(transport, motors), daemon=True)
    rx_thread.start()

    # Setup motor
    if not setup_motor(motor5, "Motor 5"):
        transport.disconnect()
        return

    print("Starting high-frequency sweep over Serial. Press Ctrl+C to stop.")
    try:
        start_time = time.time()
        Kp = 5.0
        Kd = 0.5

        last_angle5 = None
        while keep_running:
            t = time.time()
            # Smooth sine wave from 0 to 2*PI (0 to 360 degrees), Period = 4 seconds
            target_rad = math.pi * (1.0 - math.cos(2.0 * math.pi * 0.25 * (t - start_time)))

            # NOTE: Serial has lower bandwidth than CAN. One motor at 1000Hz is well within
            # the 921600 baud limit.
            res5 = motor5.send_mit_control(0.0, target_rad, 0.0, Kp, Kd)
            if res5 != pyrobstride.Result.SUCCESS:
                print(f"\n[FATAL] Transmission failed on Motor 5 with error: {res5.name}! Aborting loop.")
                break

            status5 = motor5.get_status()

            if status5.angle != last_angle5:
                s5 = status5
                out = (f"\rM5 Act:{s5.angle:>5.2f} Vel:{s5.velocity:>5.2f} Trq:{s5.torque:>4.1f} Tmp:{s5.temperature:>2.0f} F:{s5.fault_value:X}    ")
                print(out, end="", flush=True)

                last_angle5 = status5.angle

            # 1ms yield
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass

    print("\nStopping Motor...")
    motor5.stop()
    transport.disconnect()
    rx_thread.join(timeout=1.0)
    print("Done.")

if __name__ == "__main__":
    main()
