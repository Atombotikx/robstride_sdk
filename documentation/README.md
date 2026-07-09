# Robstride SDK Documentation

Welcome to the documentation for the Robstride SDK, a high-performance C++17 library with Python bindings for controlling Robstride CAN motors.

This documentation is split into several guides to help you get started quickly and understand the core mechanics of the SDK.

## Contents

1. [**Getting Started**](getting_started.md)
   *Prerequisites, compilation instructions, and virtual CAN testing setup.*

2. [**Hardware Setup**](hardware_setup.md)
   *How to connect a physical Robstride motor, configure the 1 Mbps CAN interface in Linux, and run examples on real hardware.*
   
3. [**Architecture Overview**](architecture.md)
   *A deep dive into the Strategy pattern, transport layer, multi-threading model, and atomicity guarantees that make this SDK robust.*

4. [**API Reference**](api_reference.md)
   *Detailed breakdown of the `Motor` class, parameter IO, telemetry structures, and connection settings.*

5. [**Code Examples**](examples.md)
   *Copy-paste ready templates in both C++ and Python showing how to run the Private Protocol, the MIT Protocol, and multi-motor shared-socket setups.*

---

## Design Philosophy

The Robstride SDK was designed to feel like an **industrial-grade robotics driver**, addressing common pain points in custom motor control:

- **Type Safety**: Uses modern C++ features (`std::optional`, strongly typed Enums, `union` wrappers) to prevent byte-packing bugs.
- **Thread Safety**: Uses fine-grained `std::mutex` locking and `std::condition_variable` synchronization to ensure zero deadlocks during high-frequency control.
- **Zero-Allocation Data Paths**: The core telemetry receive loop parses frames without any heap allocation to guarantee low-latency.
- **Mockability**: Transport layer inversion-of-control makes it simple to run `GTest` unit tests entirely without hardware.
