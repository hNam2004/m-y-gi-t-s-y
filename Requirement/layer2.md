# Layer 2 - Subsystem Requirements

This document defines the subsystem-level requirements for the IoT System for Turning Machine.

## Requirements

### **L2.1 - Web Application for Management**
- The system must include a web application for managing devices.
- The web application must provide a graphical user interface (GUI) for monitoring.
- The application should store and display real-time and historical data.
- **Mapping to L1:** Supports **L1.1** and **L1.2** by providing monitoring and data visualization.

### **L2.2 - Embedded Vibration Measurement**
- The system must have an embedded component to measure vibrations in the turning machine.
- The vibration data should be transmitted to the web application for analysis.
- **Mapping to L1:** Supports **L1.1** by detecting machine vibrations.

### **L2.3 - Embedded Thermal Camera Measurement**
- The system must include an embedded module to capture and process thermal images.
- The thermal camera should detect human presence near the workpiece for safety purposes.
- **Mapping to L1:** Supports **L1.2** by ensuring safety using a thermal camera.

### **L2.4 - Internet Access & Communication**
- The system must support internet access for remote monitoring and control.
- Communication should be secured using encryption protocols.
- **Mapping to L1:** Supports **L1.1** and **L1.2** by enabling remote monitoring and control.

### **L2.5 - Control System for Turning Machine**
- The system must include a control module to manage turning machine operations.
- The control module should respond to safety triggers and external commands.
- **Mapping to L1:** Supports **L1.2** by stopping the machine when safety triggers are detected.

### **L2.6 - Speed Rotation Measurement**
- The system must measure the rotational speed of the turning machine.
- Speed data should be stored and available for analysis.
- **Mapping to L1:** Supports **L1.1** by providing additional machine status data.

### **L2.7 - Flow Sensor for Cooling Fluid**
- The system must have a flow sensor to measure the speed of the cooling fluid.
- **Mapping to L1:** Supports **L1.1** by monitoring the cooling system's effectiveness.

### **L2.8 - Cooling Fluid Speed Control**
- The system must be able to adjust the cooling fluid speed based on system requirements.
- **Mapping to L1:** Supports **L1.1** by optimizing cooling system performance.

### **L2.9 - Power Consumption Measurement**
- The system must measure and monitor the power consumption of the turning machine.
- Power data should be logged and analyzed for efficiency optimization.
- **Mapping to L1:** Supports **L1.1** by tracking machine energy usage for efficiency.

For detailed component-level requirements, refer to [Layer 3 Requirements](layer3.md).
