#include <MPU6050_tockn.h>
#include <Wire.h>
#include "SoftPWM.h" //you have to download from arduino.cc
// define function of GPIO
#define PWM1    7
#define DIR1    8
#define PWM2    9
#define DIR2    10
#define CT3     11
#define CT1     4
#define CT2     6

const int MPU = 0x68; // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float roll, pitch, yaw, froll, fpitch, fyaw = 0;
float elapsedTime, currentTime, previousTime;
float dt = 0.015;
float t;

//variables for PID control
float target = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;
float angle = 0;
//the 'k' values are the ones you need to fine tune before your program will work. Note that these are arbitrary values that you just need to experiment with one at a time.
float Kp = 100;
float Ki = 10;
float Kd = 70;

int mtrSpd = 200;



void forward(int left, int right);

void setup() {
  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission


  SoftPWMBegin();
  SoftPWMSet(PWM1, 0);
  SoftPWMSet(PWM2, 0);
  SoftPWMSetFadeTime(PWM1, 2, 2);
  SoftPWMSetFadeTime(PWM2, 2, 2);

  pinMode(CT1,INPUT_PULLUP);
  pinMode(CT2,INPUT_PULLUP);
  pinMode(CT3, INPUT_PULLUP);

  pinMode(DIR1, OUTPUT); //DIR1
  pinMode(PWM1, OUTPUT); //PWM1
  pinMode(PWM2, OUTPUT); //PWM2
  pinMode(DIR2, OUTPUT); //DIR2

  fillter_gyro();

}


void loop() {
   //goc_do();
   bam_nut();

}

void bam_nut() {
  if (digitalRead(CT1) == 0) {
    t = millis();
    goc_do();
    if (t >= 5000) {
      target = -90;
    }

  }
  // } else if () {

  // } else if () {

  // }
}
void fillter_gyro() {
  Serial.println("Start probe fillter: ");
  delay(500);
  for(int x = 0; x< 10; x++) {
    Serial.print("*");
    delay(100);
  }
  Serial.println("*");
  for (int x = 0; x < 1000; x++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read())/131.0;
    GyroY = (Wire.read() << 8 | Wire.read())/131.0;
    GyroZ = (Wire.read() << 8 | Wire.read())/131.0;
    
    froll += GyroX;
    fpitch += GyroY;
    fyaw += GyroZ;
  }

  froll /= 1000;
  fpitch /= 1000;
  fyaw /= 1000;

  Serial.println("Probe fillter done!!!!");
  Serial.print("X fillter: "); Serial.print(froll);
  Serial.print(", Y fillter: "); Serial.print(fpitch);
  Serial.print(", Z fillter: "); Serial.println(fyaw);

}

void goc_do() {
  previousTime = millis();
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read())/131.0;
  GyroY = (Wire.read() << 8 | Wire.read())/131.0;
  GyroZ = (Wire.read() << 8 | Wire.read())/131.0;

  roll += (GyroX - froll) * dt;
  pitch += (GyroY - fpitch) * dt;
  yaw += (GyroZ - fyaw) * dt;

  // Serial.print("X : "); Serial.print((int)roll);
  // Serial.print(", Y : "); Serial.print((int)pitch);
  Serial.print("Z = "); Serial.println((int)yaw);
  pid();
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  dt = elapsedTime / 1000;


}

void pid() {
  error = target - yaw*3.141592654/180;// proportional
  integral = integral + error; //integral
  derivative = error - last_error; //derivative

  angle = (error * Kp) + (integral * Ki) + (derivative * Kd);
  Serial.print("angle = "); Serial.println(angle);


  forward(mtrSpd, mtrSpd);


  if (yaw > 0) {
    forward(mtrSpd+abs(angle), mtrSpd-abs(angle));
  } else if (yaw < 0) {
    forward(mtrSpd-abs(angle), mtrSpd+abs(angle));
  }
  last_error = error;
}

void stopCar(){
  control_motor(0, 0, 0);
  control_motor(1, 0, 0);
}

void forward(int left, int right){ //drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
  control_motor(0, 0, left);
  control_motor(1, 0, right);

}

void left(int left, int right){ //rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
  control_motor(0, 0, left);
  control_motor(1, 1, right);
}

void right(int left, int right){
  control_motor(0, 1, left);
  control_motor(1, 0, right);
}

void control_motor(int dc, int dir, int spd) {
  switch (dc) {
    case 1: {
        if (dir == 1) {
          digitalWrite(DIR1, 0);
          SoftPWMSet(PWM1, spd);
        } else {
          digitalWrite(DIR1, 1);
          SoftPWMSet(PWM1, 255 - spd);
        }
        break;
      }
    case 0: {
        if (dir == 1) {
          digitalWrite(DIR2, 0);
          SoftPWMSet(PWM2, spd);
        } else {
          digitalWrite(DIR2, 1);
          SoftPWMSet(PWM2, 255 - spd);
        }
        break;
      }
  }
}