## Humidity and Flow Control Project

### Description
This project implements a humidity and flow control system using an ESP32 microcontroller. The system consists of two independent lines, each with its own humidity and flow sensor, electric valve, and control buttons.

### Hardware Requirements
- ESP32 Microcontroller (developed and tested on ESP32)
- Humidity Sensors (4 in total)
- Flow Sensors (2 in total)
- Electric Valves (2 in total)
- Control Buttons (4 in total)
- Connection to a WiFi network
- Additional electrical components as per the wiring diagram

### Dependencies
- FreeRTOS Library
- lwIP (Lightweight IP) Library
- MQTT (Message Queuing Telemetry Transport) Library for communication with an MQTT server
- ESP-IDF (IoT Development Framework) Library for ESP32 development

### Configuration
1. Ensure you have the ESP-IDF development environment installed.
2. Configure the WiFi network in the `sdkconfig` file with your network credentials.
3. Configure MQTT server details in the `main/main.c` file under the "MQTT Constants" section.

### Usage
1. Connect the ESP32 and ensure it is connected to the WiFi network.
2. The ESP32 will connect to the configured MQTT server and start publishing and receiving data according to the implemented control logic.
3. Monitor system data and status through an MQTT client or the development interface.

### Features
- Automatic, manual, and timer-based humidity control for each line.
- Real-time monitoring of water flows.
- Data publication to an MQTT server for external monitoring.
- Remote adjustment of desired humidity values for each line.
- Remote change of timer mode time.

### Notes
This project is an example and may require adaptations based on the specific conditions and requirements of the environment in which it will be implemented. Refer to the ESP-IDF documentation and the used libraries for more details on configuration and specific functionalities.
