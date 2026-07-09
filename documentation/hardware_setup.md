# Hardware Setup (Real Actuator)

This guide walks you through connecting a physical Robstride motor and configuring your Linux system to communicate with it.

## 1. Hardware Connections

1. **Power**: Connect the Robstride motor to a suitable DC power supply (e.g., 24V or 48V depending on your specific motor model). **Do not turn on the power yet.**
2. **CAN Bus**: 
   - Connect the CAN High (`CAN_H`) and CAN Low (`CAN_L`) pins from the motor to your USB-to-CAN adapter or CAN HAT.
   - Ensure you have a **120-ohm terminating resistor** connected across `CAN_H` and `CAN_L` at both ends of your CAN bus network. Many CAN adapters have built-in termination that can be enabled via a jumper.
3. **Common Ground**: Ensure there is a common ground connection between your CAN adapter and the motor's logic ground.

## 2. Configure the Motor ID and Protocol

Out of the box, Robstride motors are typically configured to a specific CAN ID (default is often 1) and a specific baud rate. 
Ensure you know the expected CAN baud rate of your motor. Robstride motors generally use:
- **1 Mbps (1000000)** for Private Protocol.
- **1 Mbps (1000000)** for MIT Protocol.

*Note: If you need to change the Motor ID or protocol mode, refer to the Robstride official software tool (usually provided for Windows) to configure the EEPROM parameters via serial/USB before connecting to Linux.*

## 3. Configure the Linux CAN Interface

Once your USB-to-CAN adapter is plugged into your Linux machine, it will typically show up as a network interface (e.g., `can0`).

1. **Find your CAN interface:**
   ```bash
   ip link show
   ```
   Look for an interface starting with `can` (e.g., `can0`, `can1`).

2. **Set the bitrate and bring up the interface:**
   Assuming your interface is `can0` and the motor communicates at 1 Mbps:
   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set up can0
   ```
   *If you need to change the bitrate later, you must bring the interface down first:*
   ```bash
   sudo ip link set down can0
   ```

3. **Verify the connection:**
   You can monitor the CAN bus to ensure it's functioning using the `can-utils` package:
   ```bash
   sudo apt install can-utils
   candump can0
   ```
   *If you power on the motor while `candump` is running, you may see startup/heartbeat frames appear in the terminal.*

## 4. Running the Examples on Real Hardware

Once the `can0` interface is `UP` and your motor is powered on, you can run the compiled C++ or Python examples.

**Note: Safety First!** Ensure your motor is securely mounted and has clearance to rotate safely before running any examples.

### Running C++ Examples

Assuming you compiled the SDK and examples (as shown in [Getting Started](getting_started.md)):

```bash
cd build
# Run the benchmark/example program
./example_benchmark
```
*Make sure the source code of the example uses `"can0"` as the transport interface string and matches your motor's actual ID and protocol type.*

### Running Python Examples

If you built the Python bindings:
```bash
cd build
python3 ../examples/example_private.py
```
*(You may need to ensure `pyrobstride.so` is in your PYTHONPATH or run the script from the directory containing the compiled module).*

## Troubleshooting

- **`Network is down` error**: You forgot to run `sudo ip link set up can0`.
- **`candump` shows no data**: 
  - Check your 120-ohm termination resistors.
  - Check that CAN_H and CAN_L are not swapped.
  - Verify the CAN bitrate matches the motor's configured baud rate.
- **Motor doesn't respond to commands**:
  - Verify the `Motor ID` in your code matches the hardware ID.
  - Verify you are using the correct `Protocol::PRIVATE` vs `Protocol::MIT` based on how the motor was flashed/configured.
