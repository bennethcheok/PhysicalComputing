#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1);
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

//Calibration
float HeadingMax, HeadingMin, RollMax, RollMin, PitchMax, PitchMin;
long lastDisplayTime;
int count;

void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void setupVariables()
{
  HeadingMax = 0.0;
  HeadingMin = 0.0;
  RollMax = 0.0;
  RollMin = 0.0;
  PitchMax = 0.0;
  PitchMin = 0.0;

  count = 0;
  lastDisplayTime = millis();
}

void setup(void) {
  Serial.begin(115200);

  if(!lsm.begin())
  {
    while(1)
      Serial.println("Initislisation Error");
  }

  setupSensor();
  setupVariables();
}

void loop(void)
{
  RAW();
  //AHRS();
  //QT();
}

//Quick tests
void QT()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  sensors_vec_t orientation;

  if(ahrs.getOrientation(&orientation))
  {
    //Accel orientation
    
    Serial.print(F("data:"));
    Serial.print(orientation.roll);
    Serial.print(F(":"));
    Serial.print(orientation.pitch);
    Serial.print(F(":"));
    Serial.print(orientation.heading);
    Serial.print(F(":"));
    

    //Accel raw
    /*
    Serial.print(F("data:"));
    Serial.print(accel.acceleration.x);
    Serial.print(F(":"));
    Serial.print(accel.acceleration.y);
    Serial.print(F(":"));
    Serial.print(accel.acceleration.z);
    Serial.print(F(":"));
    */
    
    //Gyro Raw
    Serial.print(gyro.gyro.x);
    Serial.print(F(":"));
    Serial.print(gyro.gyro.y);
    Serial.print(F(":"));
    Serial.println(gyro.gyro.z);
  }
}

//Adafruit provided AHRS
void AHRS()
{
  sensors_vec_t orientation;

  if(ahrs.getOrientation(&orientation))
  {
    sendData(orientation);
    //showData(orientation);
    //calibrateData(orientation);
  }
}

void RAW()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  //Accel measurement: m/(s^2)
  
  Serial.print("raw:");
  Serial.print(accel.acceleration.x);
  Serial.print(":");
  Serial.print(accel.acceleration.y);
  Serial.print(":");
  Serial.print(accel.acceleration.z);
  Serial.print(":");
  //delay(100);

  //Gyro measurement: degree/second
  
  Serial.print(gyro.gyro.x);
  Serial.print(":");
  Serial.print(gyro.gyro.y);
  Serial.print(":");
  Serial.print(gyro.gyro.z);
  Serial.print(":");
  
  
  //Mag measurement: gauss
  
  Serial.print(mag.magnetic.x);
  Serial.print(":");
  Serial.print(mag.magnetic.y);
  Serial.print(":");
  Serial.println(mag.magnetic.z);
}

void showData(sensors_vec_t orientation)
{
  Serial.print(F("Orientation: "));
  Serial.print(orientation.roll);
  Serial.print(F(" "));
  Serial.print(orientation.pitch);
  Serial.print(F(" "));
  Serial.print(orientation.heading);
  Serial.println(F(""));
}

void sendData(sensors_vec_t orientation)
{
  Serial.print(F("data:"));
  Serial.print(orientation.roll);
  Serial.print(F(":"));
  Serial.print(orientation.pitch);
  Serial.print(F(":"));
  Serial.println(orientation.heading);
  //delay(100);
}

void calibrateData(sensors_vec_t orientation)
{
  if(orientation.roll > RollMax) RollMax = orientation.roll;
  if(orientation.roll < RollMin) RollMin = orientation.roll;

  if(orientation.pitch > PitchMax) PitchMax = orientation.pitch;
  if(orientation.pitch < PitchMin) PitchMin = orientation.pitch;

  if(orientation.heading > HeadingMax) HeadingMax = orientation.heading;
  if(orientation.heading < HeadingMin) HeadingMin = orientation.heading;

  if((millis() - lastDisplayTime) > 1000)
  {
    Serial.print("RollMax: ");
    Serial.print(RollMax);
    Serial.print("\tRollMin: ");
    Serial.println(RollMin);
    
    Serial.print("PitchMax: ");
    Serial.print(PitchMax);
    Serial.print("\tPitchMin: ");
    Serial.println(PitchMin);

    Serial.print("HeadingMax: ");
    Serial.print(HeadingMax);
    Serial.print("\tHeadingMin: ");
    Serial.println(HeadingMin);

    count+=1;
    Serial.println(count);
    
    lastDisplayTime = millis();
  }
}
