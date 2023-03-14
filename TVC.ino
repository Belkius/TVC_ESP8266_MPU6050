#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <Math.h>
#define _USE_MATH_DEFINES
#include <KalmanFilter.h>

Adafruit_MPU6050 mpu;

Servo x;
Servo y;
float theta=16;
float ox,oy,px=0,py=0,R,P,I,D,Kp,Ki,Kd,calx=0,caly=0,roz,od=0,oddx=0,oddy=0,dt,pos=0;
float xmid=48;
float ymid=30;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;


void setup() {

x.attach(2); //D4
y.attach(0); //D3

x.write(xmid);
y.write(ymid);

Serial.begin(115200);
while (!Serial)
    delay(10);

if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
}

Serial.println("Center Position");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  
  dt=0.01;
  Kp=0.3;
  Ki=0.01;
  Kd=0.005;

  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ox=-(atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))*180.0)/M_PI;;
  oy=(atan2(a.acceleration.y, a.acceleration.z)*180.0)/M_PI;
 
  Serial.println("");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

 /* Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x-ox);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y-oy);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
*/
  Serial.println("");



  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))*180.0)/M_PI;
  accRoll  = (atan2(a.acceleration.y, a.acceleration.z)*180.0)/M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, g.gyro.y);
  kalRoll = kalmanX.update(accRoll, g.gyro.x);


  Serial.print(accPitch);
  Serial.print(":");
  Serial.print(accRoll);
  Serial.print(":");
  Serial.println("R:");

    Serial.println("");

  Serial.print(kalPitch);
  Serial.print(":");
  Serial.print(kalRoll);
  Serial.print(":");
  Serial.println("K:");

    Serial.println("");
 
  
  if(abs(ox-kalRoll)>0.5){
    od=ox-kalRoll;
    P=Kp*od;
    calx+=od*dt;
    I=Ki*calx;
    roz=(od-oddx)/dt;
    D=Kd*roz;
    R=P+I+D;
    oddx=od;
    //Serial.println(R);Serial.println("");
   
    if(R>theta){
    pos=xmid+theta;
    }
    else if (R<(-theta)){
    pos=xmid-theta;
    }
    else{
    pos=xmid+R;
    }
   // Serial.println(R);Serial.println("pos: ");
   // Serial.println(pos);Serial.println("");
    x.write(pos);
    }
    else
    x.write(xmid);
  
   //////////////////////////////////////////////////////////////////////////////////////////////////////
   
    if(abs(oy-kalPitch)>0.5){
    od=oy-kalPitch;
    P=Kp*od;
    caly+=od*dt;
    I=Ki*caly;
    roz=(od-oddy)/dt;
    D=Kd*roz;
    R=P+I+D;
    oddy=od;
    //Serial.println(R);Serial.println("");
   
    if(R>theta){
    pos=ymid+theta;
    }
    else if (R<(-theta)){
    pos=ymid-theta;
    }
    else{
    pos=ymid+R;
    }
    //Serial.println(R);Serial.println("pos: ");
    //Serial.println(pos);Serial.println("");
    y.write(pos);
    }
    else
    y.write(ymid);
    

  delay(10);
}
