---
title: "Research Paper Title"
date: 2024-11-18
draft: false
---

# Research Paper Title

**Author Name\*, Co-Author Name\*\***

*Affiliations or Institution*

---

## Abstract
The abstract is a brief summary of the research. It usually covers the objectives, methodology, results, and conclusion.

---

## Introduction
The introduction provides context to the study, defines the problem, and outlines the purpose and objectives of the research.

## Methodology
Explain the methods used to gather data and conduct the research. Describe any tools, techniques, or approaches used.

## Results
Summarize the findings of the research. Use bullet points, tables, or charts if necessary to present data clearly.

## Discussion

### ESP-Now and MQTT Connection Challenges 

Integrating ESP-Now and MQTT posed a significant challenge due to their conflicting requirements. ESP-Now is designed for low-power, peer-to-peer communication without relying on traditional WiFi networks, whereas MQTT depends on WiFi for client-to-broker communication. Attempting to use both on the same ESP32 created conflicts that resulted in inconsistent data transmission and frequent disconnections. Despite experimenting with different configurations, such as alternating between ESP-Now and WiFi modes, the implementation remained unreliable. This led us to explore potential alternatives, such as separating roles across devices, which aligns with the professor's suggestions. One approach was to designate a single ESP32 as an ESP-Now master gateway and relay data through a second ESP32 or a laptop using a USB serial connection.

### Synchronizing Four ESP32 Devices and Professor's Recommendations 

Managing four ESP32s within the network added complexity, particularly in ensuring seamless communication and role allocation. Assigning specific tasks, such as one ESP32 as an ESP-Now master and another as an MQTT relay, required significant troubleshooting to address connectivity drops and data loss. The professor’s insights offered practical solutions to these challenges. Specifically, the recommendation to designate an ESP-Now master connected to another ESP32 via RS232 or i2c for MQTT forwarding was a promising approach. Alternatively, connecting the ESP-Now master to a laptop via USB and using a Python-based MQTT client provided an efficient and flexible workaround. These strategies helped reframe the project’s architecture to better accommodate the limitations of ESP-Now and MQTT coexistence. 

## Conclusion

The project demonstrated the feasibility of integrating ESP-Now and MQTT to create a reliable IoT network using ESP32 microcontrollers. The process began with establishing a connection among multiple ESP32s using ESP-Now, which provided low-latency peer-to-peer communication. This was followed by utilizing MQTT to transmit data gathered from the DHT11 sensors to Node-RED for visualization and monitoring.Ultimately, the project contributes valuable insights into IoT architecture design, emphasizing the significance of balancing technical feasibility with system scalability and performance. This endeavor sets a strong foundation for future work, including refining communication protocols and exploring hybrid solutions for IoT networks.While the dual integration of ESP-Now and MQTT proved to be challenging, particularly due to conflicts arising from simultaneous usage of WiFi and ESP-Now, the project successfully illustrated how data transmission can be achieved efficiently within constrained IoT ecosystems. This work underscores the importance of creative problem-solving, such as implementing a gateway ESP32 or alternative communication methods as highlighted by the professor, in overcoming technical limitations.
 
---

## References
1. **Reference 1**: Author, Title, Journal/Book, Year.
2. **Reference 2**: Author, Title, Journal/Book, Year.
3. **Reference 3**: Author, Title, Journal/Book, Year.
