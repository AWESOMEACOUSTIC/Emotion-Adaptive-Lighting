Emotion-Adaptive Lighting System README

Overview


This README provides a comprehensive overview of the Emotion-Adaptive Lighting System, a project designed to improve well-being and productivity in distributed work and online learning environments. By dynamically adjusting lighting based on environmental conditions and user activity, this system addresses issues like eye strain, mental fatigue, and disrupted circadian rhythms. This prototype uses affordable hardware and simple software to create an adaptive, human-centered lighting solution.

The project is inspired by real-world user personas—such as remote workers, designers, and students—and builds on research in chronobiology and ergonomics. It's scalable, with potential for future enhancements like IoT integration and machine learning.

Problem Statement


In today’s era of distributed work and online learning, people often spend long hours under unchanging artificial lighting. Such static illumination fails to support our natural circadian rhythms, leading to eye strain, mental fatigue, disrupted sleep cycles, and reduced focus. Consider a software engineer coding through the afternoon, a freelance designer juggling deadlines, or a university student attending back-to-back virtual lectures—each struggles against the discomfort of harsh or dim light and the cognitive drag of monotony. These real-world use cases underscore a need for an adaptive lighting system that senses both environmental conditions and user state, then dynamically adjusts color temperature and brightness to sustain well-being and productivity over the day.

To illustrate, meet three user personas. Emma is a remote developer who often works from dawn until dusk; she battles midday slumps and late-night overstimulation that wreck her sleep. Raj is a graphic designer whose home studio floods with sunlight in the afternoon, causing glare on his screen and forcing him into awkward postures. Sofia is an online student who loses concentration during evening study sessions and forgets to take breaks until her eyes ache. All three inhabit different routines and spaces, yet they share a common need: context-aware lighting that responds intelligently to their activity, ambient brightness, and personal comfort.

Solution Approach


Our emotion-adaptive lighting system fuses simple sensors with a microcontroller to close the loop between environment, physiology, and illumination. A light-dependent resistor tracks ambient lux, a PIR sensor estimates user movement and “micro-activity,” and a DHT22 module measures temperature and humidity. An Arduino Uno processes these inputs in real time, applying an exponential moving average to detect waning activity or thermal discomfort. Depending on the hour, the system selects a target correlated color temperature—cooler whites for morning focus, neutral midday tones to maintain flow, warm hues in the evening to ease wind-down—and modulates LED brightness to avoid glare. When prolonged inactivity is detected, gentle amber pulses on the desk lamp and a prompt on the LCD nudge the user to stand and hydrate. Through this closed-loop approach, Emma’s circadian alignment is reinforced, Raj avoids over-illumination and glare, and Sofia gains subtle engagement cues—all without manual fiddling.

Hardware Components


At the heart of the prototype sits the Arduino Uno R3, a versatile 8-bit microcontroller board offering analog and digital I/O for sensor integration and PWM signals for LED control. The LDR (photoresistor) forms a voltage divider whose analog voltage maps to ambient light levels, guiding brightness capping. A PIR motion sensor uses a pyroelectric element to detect changes in body heat and subtle movements, supplying a binary motion flag. The DHT22 digital sensor combines a calibrated capacitive humidity element and a thermistor to deliver 0.1-degree and 0.1-percent resolution readings over a single data line. Three discrete 5 mm common-anode RGB LEDs provide the tunable white output, each channel driven through a current-limit resistor and switched by a 2N2222 NPN transistor under Arduino PWM control. A 16×2 character LCD (HD44780) in 4-bit mode displays status and prompts, its contrast set by a 10 kΩ potentiometer. A solderless breadboard and jumper wires tie everything together, while optional modules such as a relay or Bluetooth add future expandability.

Rationale for the Problem Statement


The shift to remote work and digital learning has accelerated demand for home-office wellness innovations. Yet most off-the-shelf “smart lamps” focus on scheduling or voice commands rather than continuous, human-centered adaptation. Research in chronobiology and ergonomics underscores the impact of dynamic lighting on cognitive performance, mood regulation, and ocular comfort. By addressing eye strain, thermal discomfort, circadian misalignment, and sedentary fatigue in an integrated system, our project fills a gap between basic occupancy sensing and complex, high-cost tunable-white installations. The problem is timely, relevant, and scalable, touching anyone reliant on screens and artificial light for work or study.

Future Scalability


This prototype lays a foundation for an IoT-enabled ecosystem. A networked dashboard could aggregate lamp data from multiple rooms, offering scheduling, personalized profiles, and long-term wellness analytics. Machine-learning models trained on user behavior and environmental trends might predict optimal lighting transitions or preemptively suggest breaks. Integration with calendars, voice assistants, or biometric wearables could provide richer context—freeing the system from fixed sensors. Hardware upgrades to full 24 W tunable-white LED fixtures or wireless mesh bulbs would extend coverage. In shared office spaces, centralized management of dozens of adaptive luminaires could harmonize group comfort while conserving energy.

Working Prototype Overview


Upon power-up, the Arduino initializes its I/O and the LCD, then enters the main loop. Each cycle reads the LDR’s analog voltage, queries the PIR for motion, and bit-bangs the DHT22 for precise temperature and humidity. An exponential filter on the PIR count yields an activityScore that decays or rises based on movement frequency. The hour-of-day index determines a baseline correlated color temperature (CCT), which shifts warmer by 300 K if comfortIndex (derived from temperature and humidity) exceeds comfort thresholds. Ambient lux triggers a glare guard, capping brightness at half power if the workspace is already bright. PWM outputs to the RGB LEDs blend the mapped red, green, and blue channels to achieve the calculated CCT and brightness. On the LCD and Serial Monitor, the system reports current temperature, humidity, CCT, brightness percentage, LDR value, PIR state, and activityScore. When activityScore drops below 0.1, the lamp gently pulses amber and the LCD displays a “Take a short break + hydrate” reminder before resuming normal operation.

Code Explanation


The sketch begins with pin definitions and instantiation of the LiquidCrystal object. A self-contained readDHT22() function implements the open-collector, time-critical protocol to fetch raw bytes, validate CRC, and convert them into human-readable temperature and humidity. Global variables track the moving average of motion and timing intervals for sensor reads and display updates. In setup(), serial I/O is started, sensor and actuator pins are configured, and the LCD is initialized with a welcome message. The loop() function orchestrates the sensing-processing-actuating cycle: analogRead on A0 captures lux, digitalRead on D2 reflects motion, and readDHT22() returns new comfort data. A ternary cascade selects CCT based on (millis()/1000)%24 to simulate real clock hours. Comfort nudges and glare guards adjust CCT and brightness respectively. PWM writes drive the RGB channels, and a timed LCD refresh prints formatted strings for temperature, humidity, CCT, and brightness. A conditional block triggers the break-nudge animation when the smoothed activityScore falls below a threshold. Sub-50 ms delays regulate loop pace, ensuring responsiveness without hogging CPU. This lean but complete codebase demonstrates how low-cost sensors, simple algorithms, and basic actuators can create a human-centric lighting environment that learns and adapts to real-time needs.

Getting Started


To build and test this prototype:


1. Hardware Setup: Assemble the components on a breadboard using the described connections. Ensure all sensors and LEDs are properly wired to the Arduino Uno.

2. Software: Upload the Arduino sketch (not included in this README; refer to the code explanation for details) to the Arduino board.

3. Testing: Power on the system and monitor the Serial Monitor and LCD for outputs. Observe how the lighting adapts to simulated conditions like motion changes or ambient light variations.

4. Improvements: Feel free to expand the project with additional features, such as wireless connectivity or advanced algorithms.
