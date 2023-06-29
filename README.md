# health-monitor

Arduino code for health wristband that monitors the following -

Heart abnormality: Through the reading of the pulse sensor, the arduino will process the
data for transmission to the phone. Once received the app will interpret the data and if the
average values are irregular or too high, the app will then trigger a help or warning signal to
the user about a possible heart condition.

Blood Oxygen: The arduino will read the blood oxygen sensor data and send to the phone. 
Phone app will interpret the data and output the % blood oxygen value on the screen, and warn
if it reaches below a safe threshold.

Temperature monitoring: If the system detects a high or low body temperature than normal
of the person, the app will give a temperature warning.

Fall detection: If the system detects a fall from a combination of data from the
accelerometer and the gyroscope, the app will then send out a warning or an alert to the
device user as well as send a text message notification to an emergency contact.

Combination: In the case where the app registers abnormal readings from multiple sensors,
the app will give a combined warning to the device holder as well as notify anyone also
connected to the warning system of the app.
