# TVC_ESP8266_MPU6050

Program written in arduino for TVC of a model rocket. The model is using a ESP8266 board with MPU6050 module and two basic servos. 
The program is taking information from the accelerometer and adjusting the thrust accordingly with the two servo motors attached.
To use it you have to figure out the values of P, I and D for your model.
Kalman filter used in the project is available here: 
https://github.com/jarzebski/Arduino-KalmanFilter/blob/a6edd0524e8502b80910fce535a1004f980cf097/KalmanFilter_MPU6050/KalmanFilter_MPU6050.ino
