## __Zigbee_TempMeas_And_ColorSens_Server_Router Application Description__

**SSY projekt 2025/2026**

Students: **Pokorný: 247162, Šebestyán: 247177**


This project demonstrates a Zigbee network with a Coordinator and multiple Router devices that publish sensor endpoints with various peripherals:
- **Coordinator**: Client with LED indicators (E15, E16) and ePaper display
- **Router**: Server with Temperature Measurement (Endpoint 17) and Color Control (Endpoint 18) 
- **Router2**: Server with button controls (for managing LED1 and LED2 on E15 and E16)

The application creates or joins a Zigbee network, then periodically updates both the temperature value and the color sensor value. The temperature is read from a real DHT11 sensor or simulated by random variation. The color sensor is simulated as a Hue/Saturation source and also logged as equivalent RGB values.

### __Keywords__

Connectivity, Zigbee, 802.15.4, Temperature Measurement cluster, Color Control cluster, Hue, Saturation, RGB

### __Hardware Requirements__

For this application it is requested to have:  

* One STM32WBA55xx Nucleo board loaded with application : **Zigbee_TempMeas_Client_Coord**  
  - Connected peripherals: LED diodes (E15 red, E16 green), ePaper display
* One STM32WBA55xx board loaded with application : **Zigbee_TempMeas_Server_Router2**  
  - Temperature Measurement and Color Control endpoints
* One STM32WBA55xx board loaded with application : **Zigbee_OnOff_Client_Router**  
  - Button controls (SW1, SW2) for LEDs


### __Demo use case__

* **Coordinator (Client)** displays:
	* Temperature from connected sensor
	* Color information from sensor
	* Status on LED diodes: E15 (red) and E16 (green) indicate connection status
	* Information is also displayed on the ePaper display

* **Router TempMeas Server** provide:
	* Endpoint 17: Temperature Measurement - updates regularly (500 ms) in **ZCL_TEMP_MEAS_ATTR_MEAS_VAL**
	* Endpoint 18: Color Control - updates regularly (500 ms) in **ZCL_COLOR_ATTR_CURRENT_HUE** and **ZCL_COLOR_ATTR_CURRENT_SAT**
	* Color values are also converted locally to RGB for logging

* **Router OnOff client**:
	* PushB SW1 : E15 Toggle LED1 - LED can be turn on or off **ZCL_DEVICE_ONOFF_SWITCH; ZCL_CLUSTER_ONOFF**
	* PushB SW2 : E16 Toggle LED2 - LED can be turn on or off **ZCL_DEVICE_ONOFF_SWITCH; ZCL_CLUSTER_ONOFF**

* When the Client configures reporting, it can receive regular reports from both clusters on both Router devices.

* Read-only attributes used on the Server side:
    * Temperature Value (16-bit integer)
    * Hue (uint8)
    * Saturation (uint8)

<pre>
                  Coordinator                             Router                            Router2
                +----------+                         +----------+                       +----------+
                |          |                         |          |                       |          |
                | TempMeas |                         | TempMeas |                       | OnOff    |
                | Client   |                         | Server   |                       | Client   |
                |          |                         |          |                       |          |
                |          |                         |          |                       |          |
                | - LED E15|                         | - Temp s.|                       |          |
                | - LED E16|                         | simulated|                       |          |
                | - ePaper |                         | - Color s|                       |          |
                |          |                         | simulated|                       |          |
                |          | Config Report (T,C)     |          |                       |          |
                |          |-----------------------> |          |                       |          |
                |          | <---------------------- |          |                       |          |
                |          |                         |          |                       |          |
                |          | Config Report (T,C)     |          |                       |          |
                |          |----------------------------------------------------------> |          |
                |          | <--------------------------------------------------------| |          |
                |          |                         |          |                       |          |
                |          | <------ Report (every 5s or change)|                       |          |
                |          | <------ Report (every 5s or change)----------------------< |          |
                |          |                         |          |                       |          |
                |          |                         |          |  <= PushB SW1: Start  |          | <= PushB SW1: Toggle LED1
                |          |                         |          |     periodic timer    |          |    Endpoint 15
                |          |                         |          |                       |          |
                |          |                         |          |  <= PushB SW2: Stop   |          | <= PushB SW1: Toggle LED2
                |          |                         |          |     periodic timer    |          |    Endpoint 16
                |          |                         |          |                       |          |
                +----------+                         +----------+                       +----------+
             
             LEDs on Coordinator:                    Periodic Updates:                  Button Controls on Router2:
             - E15 (Red): Connection status/LED1     - E17 Temperature: every 5s        - SW1: ON/OFF LED1
             - E16 (Green): Data status/LED2         - E18 Color (Hue/Sat): every 5s    - SW2: ON/OFF LED2

             Display on Coordinator:
             - ePaper: Shows temp, color (WIP)
</pre>

### __Application Setup__

* First, build all three applications and program the STM32WBA55xx boards.
* To run the application:
	1. Start the first board as the Coordinator. This board runs the **Zigbee_TempMeas_Client_Coord** application.
	   - Wait for the Red LED (E15) to become ON, indicating connection status.
	   - The ePaper display will show startup information.
	2. Start the second board as the Router. This board runs the **Zigbee_TempMeas_Server_Router2** application.
	   - Wait until the Blue LED stays ON before proceeding.
	3. Start the third board as the Router2. This board runs the **Zigbee_OnOff_Client_Router** application.
	   - Wait until the Blue LED stays ON. The buttons are now active.

**Note:** Red/Green/Blue blinking may indicate an application error on any device.

### __Serial monitor__

* Connect the board to a terminal using the STLink Virtual COM Port.
* UART settings:
  - BaudRate = 115200 baud
  - Word Length = 8 Bits
  - Stop Bit = 1 bit
  - Parity = none
  - Flow control = none
  - Terminal line ending: `<LF>`

### __References__
* https://mcuxpresso.nxp.com/mcuxsdk/25.09.00/html/middleware/wireless/zigbee/Docs/JNUG3132/ZCL_intro/topics/zigbee_cluster_library_zcl.html
* https://www.youtube.com/watch?v=DLmbNfUh62E
* https://botland.cz/svetelne-a-barevne-senzory/4932-barevny-snimac-menic-kmitoctu-svetla-tcs3200-waveshare-9520-5904422374211.html
* 
