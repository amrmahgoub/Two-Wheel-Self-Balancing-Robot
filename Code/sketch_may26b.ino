
#include <Wire.h>
#include<SoftwareSerial.h>
#include <LiquidCrystal.h>


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const char rs = 12, en = 11, d4 = 2, d5 = 7, d6 = 8, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float accAngleX, accAngleY, gyroAngleY;
float AccErrorX, AccErrorY, GyroErrorY,GyroAngleErrorY;
float GyroY;
float KalmanAnglePitch = 0 , KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};
float elapsedTime, currentTime, previousTime;
int c = 0;
volatile int motorPower;
volatile float  currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
#define Kp  84
#define Kd  2.943
#define Ki  150
#define sampleTime  0.005
//#define Kp  120
//#define Kd  7
//#define Ki  180
//#define sampleTime  0.005
#define leftMotorPWMPin 5
#define rightMotorPWMPin  6
#define targetAngle 0
volatile byte count = 0;


void kalman_1d(float KalmanState , float KalmanUncertainty , float KalmanInput , float KalmanMeasurement) {
  KalmanState = KalmanState +  0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState +  KalmanGain * ( KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}



void setMotors(int MotorSpeed) {
  if (MotorSpeed >= 0) {
    digitalWrite(4, LOW);
    digitalWrite(3, LOW);

    analogWrite(leftMotorPWMPin, MotorSpeed);
    analogWrite(rightMotorPWMPin, MotorSpeed);

  }
  else {
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    analogWrite(leftMotorPWMPin, MotorSpeed * -1);
    analogWrite(rightMotorPWMPin, MotorSpeed * -1);

  }

}
void init_PID() {
  // initialize Timer1=
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
void setup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  LowPassFilter();
  calculate_IMU_error();
  delay(20);
  init_PID();
  lcd.begin(16, 2);
}
void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x45); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = GyroY - GyroErrorY;
  gyroAngleY = gyroAngleY + GyroY * 0.005;

  accAngleX = (atan(AccZ / AccX) * 180 / PI) - AccErrorX;
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, GyroY, accAngleX);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower);
  lcd.setCursor(0, 0);
  lcd.print("Curr Angle: ");
  lcd.print(gyroAngleY);
  
  Serial.println(gyroAngleY);
}
void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + (atan(AccZ / AccX) * 180 / PI);
    c++;
  }
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroY = Wire.read() << 8 | Wire.read();
    GyroAngleErrorY = GyroAngleErrorY + GyroY * 0.005;
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    c++;
  }
  GyroErrorY = GyroErrorY / 200;
}
void LowPassFilter(void) {
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission();
}
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  // accAngleX = (atan(AccZ / AccX) * 180 / PI) -AccErrorX;
  //gyroAngleY = gyroAngleY + GyroY * 0.005;
  currentAngle = gyroAngleY;
// if(currentAngle>= 2 || currentAngle<=-2){
//  gyroAngleY = 2;
// }
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -20, 20);
  //calculate output from P, I and D values
  motorPower = Kp * (error) + Ki * (errorSum) * sampleTime + Kd * (error - prevError) / sampleTime;
  //  motorPower = constrain(motorPower, -255, 255 );
  prevError = error;
  



}
