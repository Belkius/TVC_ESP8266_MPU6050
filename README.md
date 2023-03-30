# TVC_ESP8266_MPU6050

Program written in arduino for TVC of a model rocket. The model is using a ESP8266 board with MPU6050 module and two basic servos. 
This code uses an MPU6050 sensor to read the pitch and roll angles and apply them to control two servos for TVC (thrust vector control). The Kalman Filter algorithm is used to improve the stability of the readings. The code is written in C++ and is compatible with Arduino boards.
The code being used for the Kalman filter is from the "KalmanFilter_MPU6050.ino" file in the "Arduino-KalmanFilter" repository on GitHub. Here's the link to the specific version of the file being used: https://github.com/jarzebski/Arduino-KalmanFilter/blob/a6edd0524e8502b80910fce535a1004f980cf097/KalmanFilter_MPU6050/KalmanFilter_MPU6050.ino

## Dependencies
This code requires the following libraries to be installed:

- Adafruit_MPU6050
- Adafruit_Sensor
- Wire
- Servo
- Math
- KalmanFilter

## How to Use
1. Connect the MPU6050 sensor to the I2C bus of your Arduino board.
2. Connect two servos to two PWM pins of your Arduino board. The pins used in the code are 2 for the X servo and 0 for the Y servo.
3. Upload the code to your Arduino board.
4. Open the Serial Monitor to see the output of the code.
5. Adjust the PID values Kp, Ki, and Kd to tune the response of the TVC system.
