🧊 ColdGuard

**ESP32-Based Intelligent Vaccine Fridge Monitoring System**

**Course:** 3707ICT - Automation & Internet of Things
**Specialisation:** Option A - Embedded Intelligence (Wokwi)

**Group Members**

- Kyle Lancelot
- Tyler Lambert
- Lachlan Brooks

**Project Overview**

ColdGuard is an ESP32-based intelligent monitoring and control system designed for vaccine and medication refrigerators. 

Vaccines and many termperature-sensitive medications must be stored between 2 °C and 8 °C. Traditional manual monitoring may only record refrigerator temperatures a few times per day, meaning equipment failures can remain undetected for hours. 

Coldguard continously monitors refrigerator conditions, evaluates whether changes represent normal activity or a genuine cold-chain failure, responds locally through actuators, and publishes readings and system status to a cloud dashboard. 

Rather than triggering an alarm whenever a single temperature reading exceeds a threshold, ColdGuard combines:

- Temperature
- Relative humidity
- Door state
- Time spent outside normal conditions
- Temperature trend

This allows the system to distinguish temporary events, such as someone opening the refrigerator door, from developing faults such as compressor failure. 

