Created with an ESP32 microcontroller, an Infineon i2s Microphone and the Edge Impulse machine learning library for microcontrollers. 


The red LED lets the user know that the program is waiting to hear the keyword that the Edge Impulse model has been trained to detect. If the machine learning model recognises what the user has said as the correct keyword, then the green LED is activated. 

Initially this project was meant to be integrated into an automated gardening system. The goal of this feature was to allow the user to activate a watering pump via speech.
It is easy to see how the green LED could be replaced with a connection to a transistor or a relay which in turn can activate a 12V pump.
Unfortunately the feature could not be implemented into the automatic gardening system because of time constraints.

Make sure to place the Edge Impulse Library in the Arduino IDE's "Library" folder.
