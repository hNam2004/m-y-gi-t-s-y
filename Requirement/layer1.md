# Layer 1 - System-Level Requirements

This document defines the high-level system requirements for the IoT System for Turning Machine.

## Requirements

### **L1.1 - Vibration Detection & Maintenance Trigger**
- The IoT system must detect vibration levels in the turning machine.
- When abnormal vibration patterns are detected, the system should trigger a maintenance alert.
- The system should log vibration data for further analysis.

### **L1.2 - Safety Functionality with Thermal Camera**
- The IoT system must ensure safety by monitoring the workspace with a thermal camera.
- If the system detects a person touching the workpiece while the machine is running, it must stop operation immediately.
- The system should provide a warning signal before stopping the machine.

For detailed subsystem requirements, refer to [Layer 2 Requirements](layer2.md).
