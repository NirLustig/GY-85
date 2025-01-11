# GY-85
Small project for reading through gy-85 roll, pitch, and yaw angles and sending them over to python
Hardware - I've used Arduino Mega or ESP32-WRoom-32D (it can be operated in any hardware that has SDA and SCL ports).

I've used the GY-85 module to read the raw data from the gyro \ accelerometer \ magnetometer.
At the end of the Arduino script, you can find the calibration which is finding the bias of the sensor and reducing it while reading the data each cycle.
Roll and Pitch are calculated by projecting g over the y and x-axis while rolling and pitching.
Roll and Pitch were also calculated by using numerical integral with the gyro's raw data (rate of angular velocity), due to the drift I performed data fusing between the roll and pitch calculated by the accelerometer and gyro.
Yaw was calculated using the magnetometer's raw data. to avoid noise when rolling and pitching (while calculating yaw) I added rotational matrix calculations for the x and y magnetometer values.
Lastly, the serial print is for sending the data over to the Python  IDE (my case was VS code).

At the Python script, you can find a visualization of the axis in real time.
