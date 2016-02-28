#include <Adafruit_LSM9DS0.h>
#include <Adafruit_LSM303_U.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>

#define TCAADDR 0x70

/* Assign a unique ID to this sensor at the same time */
//Adafruit_HMC5883_Unified mag1 = Adafruit_HMC5883_Unified(1);
//Adafruit_HMC5883_Unified mag2 = Adafruit_HMC5883_Unified(2);

Adafruit_LSM303_Mag_Unified mag1 = Adafruit_LSM303_Mag_Unified(1);
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

/*
  void displaySensorDetails(Adafruit_HMC5883_Unified *mag)
  {
  sensor_t sensor;
  mag->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
  }
*/

void displaySensorDetails(Adafruit_LSM303_Mag_Unified *mag)
{
  sensor_t sensor;
  mag->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


void setup(void)
{
  Serial.begin(9600);
  Serial.println("LSM303 Magnetometer Test"); Serial.println("");

  /* Initialise the 1st sensor */
  /*tcaselect(5);
    if(!mag1.begin())
    {
    /* There was a problem detecting the HMC5883 ... check your connections */
  //while(1)
  //Serial.println("Error in initialisation");
  //}
  //else while(1) Serial.println("Success");

  /* Initialise the 2nd sensor */
  tcaselect(2);
  if (lsm.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    while (1) Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  else
  {
    while (1) Serial.println("success");
  }

  /* Display some basic information on this sensor */
  /*tcaselect(5);
    displaySensorDetails(&mag1);
    tcaselect(6);
    displaySensorDetails(&mag2);
  */
}

void loop(void)
{
  /* Get a new sensor event */
  //sensors_event_t event;

  /*
    tcaselect(5);
    mag1.getEvent(&event);

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //Serial.print("Sensor #1 - ");
  //Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  Serial.println("check");
  tcaselect(2);
  lsm.read();
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("Sensor #2 - ");
  Serial.print("X: "); Serial.print(lsm.magData.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(lsm.magData.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(lsm.magData.z); Serial.print("  "); Serial.println("uT");

  //delay(500);
}
