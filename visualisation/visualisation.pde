//Basic relation between imu readings and processing environment
//axisX: > +
//axisY: ^ -
//axisZ: towards viewever +
//yaw: (rotateAroundYBottomToTopClockwise+) (on imu: around Z Y>X-)
//pitch: (rotateAroundZViewFromViewerClockwise+) (on imu: around Y X^+)
//roll: (rotateAroundXViewLeftToRightClockwise+) (on imu: around X Y^-)
//accelX: (on imu X^+) 
//accelY: (on imu Y^+)
//accelZ: (on imu top of IMU/Z^ +)

import ddf.minim.*;
import ddf.minim.analysis.*;
import ddf.minim.effects.*;
import ddf.minim.signals.*;
import ddf.minim.spi.*;
import ddf.minim.ugens.*;
import processing.serial.*;
import cc.arduino.*;

//Serial Readings
Serial myPort;
int lf = 10;
String myString = null;

//Quaternion
//float [] quaternion = {1, 0, 0, 0};
Quaternion quat1, quat2;
boolean initQuat;
float [] initRot;

//Raw data
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float accelX, accelY, accelZ;

//Rotation
boolean init;
float accelRoll, accelYaw, accelPitch;
float gyroRoll, gyroYaw, gyroPitch;
float compRoll, compYaw, compPitch;
float pastAX, pastAY, pastAZ;
float FAX, FAY, FAZ;

//Position
float velX, velY, velZ;
float pastVelX, pastVelY, pastVelZ;
float OAX, OAY, OAZ;
boolean posInit;
float gravX, gravY, gravZ;
float pastTime, time;
float posX, posY, posZ;

//flex
//Angle of bending
float angle;
//Joint vector
PVector [] vec;
boolean soundPlayed;

//Audio
Minim minim;
AudioPlayer [] song;

//Initial Setup
void setup()
{
  size(500, 500, P3D);
  //String portName = Serial.list();
  myPort = new Serial(this, "COM3", 115200);
  initQuat = false;
  quat1 = new Quaternion(1, 0, 0, 0);
  
  accelX = 0.0;
  accelY = 0.0;
  accelZ = 0.0;
  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;
  
  init = true;
  accelRoll = 0.0;
  accelPitch = 0.0;
  accelYaw = 0.0;
  gyroRoll = 0.0;
  gyroYaw = 0.0;
  gyroPitch = 0.0;
  compRoll = 0.0;
  compYaw = 0.0;
  compPitch = 0.0;
  pastAX = 0.0;
  pastAY = 0.0;
  pastAZ = 0.0;
  FAX = 0.0;
  FAY = 0.0;
  FAZ = 0.0;
  
  velX = 0.0;
  velY = 0.0;
  velZ = 0.0;
  pastVelX = 0.0;
  pastVelY = 0.0;
  pastVelZ = 0.0;
  OAX = 0.0;
  OAY = 0.0;
  OAZ = 0.0;
  gravX = 0.0;
  gravY = 0.0;
  gravZ = 0.0;
  posX = 0.0;
  posY = 0.0;
  posZ = 0.0;
  posInit = true;
  
  pastTime = 0.0;
  time = millis();
  
  angle = 0.0;
  vec = new PVector[5];
  vec[0] = new PVector(0, 0);
  vec[1] = new PVector(-50, 0);
  vec[2] = new PVector(50, 0);
  
  soundPlayed = false;
  minim = new Minim(this);
  song = new AudioPlayer[2];
  song[0] = minim.loadFile("hihat_012a.wav");
  song[1] = minim.loadFile("snare.wav");
}

void draw()
{
  background(255);
  translate(width/2, height/2);
  //translate(posX*5, 0, 0);
  //translate(0, posY*5, 0);
  //translate(0, 0, posZ);
  
  //Reading serial Data
  readData();
  
  //Quaternion test
  //if(initQuat)
  //{
  //Object
  //IMU
  
  //Accel rotation data
  //accelRotation();
  
  //Gyro rotation data
  //gyroRotate();
  
  //Filtered rotation 1
  //filterRotate1();
  
  //Filtered rotation 2
  //filterRotate2();
  
  //Basic positioning using acceleration, velocity and displacement
  positionV1();
  
  //Position V2
  //positionV2();
  
  
  
  //Tests
  /*
  float alpha = 0.5;
  //println(a[0] + " " + a[1] + " " + a[2]);
  
  pX = accelX* alpha - (pX * (1.0 - alpha));
  pY = accelY * alpha - (pY * (1.0 - alpha));
  pZ = accelZ * alpha - (pZ * (1.0 - alpha));
  
  float roll = (atan2(-pX, pZ) * 180.0)/PI;
  float pitch = (atan2(pX, sqrt(pY*pY + pZ*pZ)) * 180.0)/PI;
  
  println(pitch);
  println(roll);
  
  //println(pX);
  //println(pY);
  //println(pZ);
  translate(width/2, height/2);
  rotateY(pitch);
  rotateZ(roll);
  */
  
  //Initial altitude test
  /*
  float [] test = initialAltitude(accelX, accelY, accelZ);
  compYaw = test[0];
  compPitch = test[1];
  compRoll = test[2];
  
  translate(width/2, height/2);
  rotateX(radians(-compRoll));
  rotateY(radians(compYaw));
  rotateZ(radians(-compPitch));
  */
  
  //Acceleration and positioning test
  //Gravity correction (only for flat surface)
  /*
  float distanceX = 0.0;
  float distanceY = 0.0;
  float distanceZ = 0.0;
  
  gravX = 0.9 * gravX + 0.1 * accelX;
  gravY = 0.9 * gravY + 0.1 * accelY;
  gravZ = 0.9 * gravZ + 0.1 * accelZ;
  
  //velX += accelX - OAX;
  //velY += accelY - OAY;
  //velZ += accelZ - OAZ;
  
  velX += accelX - gravX;
  velY += accelY - gravY;
  velZ += accelZ - gravZ;
  
  distanceX += velX;
  distanceY += velY;
  distanceZ += velZ;
  
  println(distanceZ);
  
  translate(width/2, height/2);
  translate(distanceX, distanceZ, distanceY);
  */
  //}
  
  box(200, 200, 200);
  
  //Flex
  //flexTest();
  
  //println("hypo: " + hypo(v1, v2, v3));
}

//Rotation with accelerometer value
void accelRotation()
{
  //if(accelX - pastAX > 0.01)
    //FAX = (accelX - pastAX) * (time - pastTime);
    
  //if(accelY - pastAY > 0.01)
    //FAY = (accelY - pastAY) * (time - pastTime);
    
  //if(accelZ - pastAZ > 0.01)
    //FAZ = (accelZ - pastAZ) * (time - pastTime);
  
  //println(accelX + ", " + FAX);
  //println(FAX + ", " + FAY + ", " + FAZ);
  
  //accelRoll = atan2(accelY, accelZ) * 180 / PI;
  //accelYaw = atan2(accelX, sqrt(accelY*accelY + accelZ * accelZ)) * 180 / PI;
  //accelYaw = atan2(accelY, accelX) * 180 * PI;
  
  accelRoll = atan2(accelY, accelZ);
  accelPitch = atan2(accelX, sqrt(accelY*accelY + accelZ * accelZ));
  accelYaw = (float)atan2(magZ * sin(accelRoll) - magY * cos(accelRoll), magX * cos(accelPitch) + magY * sin(accelPitch) * sin(accelRoll) + magZ * sin(accelPitch) * cos(accelRoll));
  
  accelRoll *= 180/PI;
  accelPitch *= 180/PI;
  accelYaw *= 180/PI;
  
  //if(FAY != 0 && FAZ != 0)
    //accelYaw = atan(FAX/sqrt(FAY*FAY + FAZ*FAZ)) * 180 / PI;
  
  //if(FAX != 0 && FAZ != 0)
    //accelRoll = atan(FAY/sqrt(FAX * FAX + FAZ * FAZ)) * 180 / PI;
    
  //if(FAX != 0 && FAZ != 0)
    //accelPitch = atan(FAZ/sqrt(FAX*FAX + FAZ*FAZ)) * 180 / PI;
    
  //accelYaw = atan(accelX/sqrt(accelY*accelY + accelZ*accelZ)) * 180 / PI;
  //accelRoll = atan(accelY/sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  //accelPitch = atan(accelZ/sqrt(accelX*accelX + accelZ*accelZ)) * 180 / PI;
  
  rotateX(radians(-accelRoll));
  rotateY(radians(accelYaw));
  rotateZ(radians(-accelPitch));
}

//Rotation with Gyroscope value
void gyroRotate()
{
  gyroRoll += gyroX * ((time - pastTime)/1000);
  gyroYaw += gyroY * ((time - pastTime)/1000);
  gyroPitch += gyroZ * ((time - pastTime)/1000);
  
  println(time + ", " + pastTime);
  
  rotateX(radians(-gyroRoll));
  rotateY(radians(gyroYaw));
  rotateZ(radians(-gyroPitch));
}

//Basic implementation of complementary filter
void filterRotate1()
{
  /*
  gyroRoll += gyroX * ((time - pastTime)/1000);
  gyroYaw += gyroY * ((time - pastTime)/1000);
  gyroPitch += gyroZ * ((time - pastTime)/1000);
  
  compRoll = 0.95*(compRoll + gyroRoll) + 0.05*(accelRoll);
  compYaw = 0.95*(compYaw + gyroYaw) + 0.05*(accelYaw);
  compPitch = 0.95*(compPitch + gyroPitch) + 0.05*(accelPitch);
  */
  
  accelRoll = atan2(accelY, accelZ);
  accelPitch = atan2(accelX, sqrt(accelY*accelY + accelZ * accelZ));
  accelYaw = (float)atan2(magZ * sin(accelRoll) - magY * cos(accelRoll), magX * cos(accelPitch) + magY * sin(accelPitch) * sin(accelRoll) + magZ * sin(accelPitch) * cos(accelRoll));
  
  accelRoll *= 180/PI;
  accelYaw *= 180/PI;
  accelPitch *= 180/PI;
  
  compRoll += gyroX * ((time - pastTime)/1000);
  compPitch += gyroY * ((time - pastTime)/1000);
  compYaw += gyroZ * ((time - pastTime)/1000);
  
  compRoll = 0.95*(compRoll) + 0.05 * (accelRoll);
  compYaw = 0.95*(compYaw) + 0.05 * (accelYaw);
  compPitch = 0.95*(compPitch) + 0.05 * (accelPitch);
  
  rotateX(radians(-compRoll));
  rotateY(radians(compYaw));
  rotateZ(radians(-compPitch));
}

//Testing complementary filter with gravity correction
void filterRotate2()
{
  float intervalS = (time-pastTime)/1000;
  
  compRoll += gyroX * ((time - pastTime)/1000);
  compPitch += gyroY * ((time - pastTime)/1000);
  compYaw += gyroZ * ((time - pastTime)/1000);
  
  //quat1 = quat(compYaw, compPitch, compRoll);
  quat1 = quat(gyroZ * intervalS, gyroY * intervalS, gyroX * intervalS);
  //quat1 = multQuat(quat1, quat2);
  
  gravX = 2 * (quat1.x * quat1.z - quat1.w * quat1.y);
  gravY = 2 * (quat1.w * quat1.x + quat1.y * quat1.z);
  gravZ = quat1.w * quat1.w - quat1.x * quat1.x - quat1.y * quat1.y + quat1.z * quat1.z;
  
  gravX *= 9.8;
  gravY *= 9.8;
  gravZ *= 9.8;
  
  //println(quaternion[0] + ", " + quaternion[1] + ", " + quaternion[2] + ", " + quaternion[3]);
  //println("initRot: " + initRot[0] + ", " + initRot[1] + ", " + initRot[2]);
  println("grav: " + gravX + ", " + gravY + ", " + gravZ);
  //println(quat2.w + ", " + quat2.x + ", " + quat2.y + ", " + quat2.z);
  println(quat1.w + ", " + quat1.x + ", " + quat1.y + ", " + quat1.z);
  println("raw accel: " + accelX + ", " + accelY + ", " + accelZ);
  //println("comp: " + compRoll + ", " + compYaw + ", " + compPitch);
  
  //accelX -= gravX[0];
  //accelY -= gravY[1];
  //accelZ -= gravZ[2];
  
  //println("corrected: " + accelX + ", " + accelY + ", " + accelZ);
  /*
  accelRoll = (float)atan2(accelY, accelZ);
  
  if (accelY * sin(accelRoll) + accelZ * cos(accelRoll) == 0)
    accelPitch = accelX > 0 ? (PI / 2) : (-PI / 2);
  else accelPitch = (float)atan(-accelX / (accelY * sin(accelRoll) + accelZ * cos(accelRoll)));
  
  accelYaw = (float)atan2(magZ * sin(accelRoll) - magY * cos(accelRoll), magX * cos(accelPitch) + magY * sin(accelPitch) * sin(accelRoll) + magZ * sin(accelPitch) * cos(accelRoll));
  */
  
  accelRoll = atan2(accelY, accelZ);
  accelPitch = atan2(accelX, sqrt(accelY*accelY + accelZ * accelZ));
  accelYaw = (float)atan2(magZ * sin(accelRoll) - magY * cos(accelRoll), magX * cos(accelPitch) + magY * sin(accelPitch) * sin(accelRoll) + magZ * sin(accelPitch) * cos(accelRoll));
  //accelYaw = atan2(accelZ, sqrt(sq(accelX) + sq(accelZ)));
  
  //accelRoll = atan2(accelX, accelZ);
  //accelPitch = atan2(accelY, accelZ);
  
  accelRoll *= 180/PI;
  accelYaw *= 180/PI;
  accelPitch *= 180/PI;
  
  float magnitude = (abs(accelX) + abs(accelY) + abs(accelZ))/9.8;
  //println("magnitude: " + magnitude);
  
  if(magnitude > 0.3 && magnitude < 2)
  {
    compRoll = 0.95*(compRoll) + 0.05 * (accelRoll);
    compYaw = 0.95*(compYaw) + 0.05 * (accelYaw);
    compPitch = 0.95*(compPitch) + 0.05 * (accelPitch);
    //println("merge");
  }
  else
  {
    compRoll += gyroRoll;
    compYaw += gyroYaw;
    compPitch += gyroPitch;
  }
  
  println("accel: " + accelYaw + ", " + accelPitch + ", " + accelRoll);
  println("comp: " + compYaw + ", " + compPitch + ", " + compRoll);
  
  //translate(width/2, height/2);
  
  rotateX(radians(compRoll));
  rotateY(radians(compYaw));
  rotateZ(radians(compPitch));
  
  //println(gyroRoll);
}

//Basic movement
void positionV1()
{
  //println("Position: " + posX + ", " + posY + "," + posZ);
  //println("Raw Accel: " + accelX + ", " + accelY + ", " + accelZ);
  //println("Velocity: " + velZ);
  float intervalS = (time-pastTime)/1000;
  println(posX);
  
  velX = pastVelX + (accelX * intervalS);
  velY = pastVelY + (accelY * intervalS);
  velZ = pastVelZ + (accelZ * intervalS);
  
  posX += (velX * intervalS) + (0.5 * accelX * sq(intervalS));
  posY += (velY * intervalS) + (0.5 * accelY * sq(intervalS));
  posZ += (velZ * intervalS) + (0.5 * accelZ * sq(intervalS));
  
  //velZ = velZ + (accelZ * (time - pastTime)/1000);
  //posZ += (velZ * intervalS);
  //posX += (0.5 * (pastVelX + velX) * intervalS);
  
  pastVelX = velX;
  pastVelY = velY;
  pastVelZ = velZ;
  
  //translate(width/2, height/2);
  //translate(posX*10, -posY*10, 0);
}

//Movement with gravity correction test
void positionV2()
{
  gravX = 0.9 * gravX + 0.1 * accelX;
  gravY = 0.9 * gravY + 0.1 * accelY;
  gravZ = 0.9 * gravZ + 0.1 * accelZ;
  
  float intervalS = (time-pastTime)/1000;
  println(intervalS);
  
  velX = pastVelX + (accelX * intervalS);
  velY = pastVelY + (accelY * intervalS);
  velZ = pastVelZ + (accelZ * intervalS);
  
  posX += (pastVelX * intervalS) + (0.5 * accelX * sq(intervalS));
  posY += (pastVelY * intervalS) + (0.5 * accelY * sq(intervalS));
  posZ += (pastVelZ * intervalS) + (0.5 * accelZ * sq(intervalS));
  
  //velZ = velZ + (accelZ * (time - pastTime)/1000);
  //posZ += (velZ * intervalS);
  //posX += (0.5 * (pastVelX + velX) * intervalS);
  
  /*
  if(accelX - gravX > 1)
  {
    velX = velX + ((accelX - gravX) * (time - pastTime));
    posX += (velX * ((time - pastTime)) );
  }
  
  if(accelY - gravY > 1)
  {
    velY = velY + ((accelY - gravY) * (time - pastTime));
    posY += (velY * ((time - pastTime)) );
  }
  
  if(accelZ - gravZ > 1)
  {
    velZ = velZ + ((accelZ - gravZ) * (time - pastTime));
    posZ += (velZ * ((time - pastTime)) );
  }
  */
  
  pastVelX = velX;
  pastVelY = velY;
  pastVelZ = velZ;
}

//Main function for reading serial data
void readData()
{
  while(myPort.available() > 0)
  {
    myString = myPort.readStringUntil(lf);
    if(myString != null)
    {
      //println(myString);
      
      String [] data = myString.split(":");
      
      //Raw Data
      if(data.length < 10)
        break;
      //Raw accel data
      accelX = float(data[1]);
      accelY = float(data[2]);
      accelZ = float(data[3]);
      //Raw gyro data
      gyroX = float(data[4]);
      gyroY = float(data[5]);
      gyroZ = float(data[6]);
      //Raw mag data
      magX = float(data[7]);
      magY = float(data[8]);
      magZ = float(data[9]);
      
      if(!initQuat)
      {
        initQuat = true;
        initRot = initialAltitude(accelX, accelY, accelZ);
        
        //compYaw += initRot[0];
        //compPitch += initRot[1];
        //compRoll += initRot[2];
      }
      
      //Adafruit AHRS readings
      /*
      if(data.length < 4)
        break;
      
      if(posInit)
      {
        OAX = (float)Double.parseDouble(data[1]);
        OAY = (float)Double.parseDouble(data[2]);
        OAZ = (float)Double.parseDouble(data[3]);
        posInit = !posInit;
      }
      println(OAZ + "," + accelPitch);
      
      accelRoll = (float)Double.parseDouble(data[1]);
      accelYaw = (float)Double.parseDouble(data[2]);
      accelPitch = (float)Double.parseDouble(data[3]);
      */
      
      //Flex
      /*
      if(data.length < 2)
        break;
        
      //println(Double.parseDouble(data[1])/360*3.142);
      //Raw flex value
      //angle = Double.parseDouble(data[1]);
      
      //Adjusted flex value
      //With mapped arduino value
      //angle = Double.parseDouble(data[1])/360*3.142;
      
      //Without raw arduino value
      angle = map((float)Double.parseDouble(data[1]), 700, 1000, 0, 3.142);
      */
    }
  }
  pastTime = time;
  time = millis();
  myPort.clear();
}

void flexTest()
{
  //println("angle: " + angle);
  
  vec[2] = v(angle-90);
  line(vec[1].x, vec[1].y, vec[0].x, vec[0].y);
  line(vec[0].x, vec[0].y, vec[2].x, vec[2].y);
  
  vec[3] = new PVector(30, 0);
  vec[4] = new PVector(-30, -70);
  
  rect(vec[3].x, vec[3].y, 50, 50);
  rect(vec[4].x, vec[4].y, 50, 50);
  

  if(vec[2].x > vec[3].x && vec[2].x < (vec[3].x+50) && vec[2].y > vec[3].y && vec[2].y < (vec[3].y+50))
  {
    if(!soundPlayed)
    {
      soundPlayed = true;
      song[0].rewind();
      song[0].play();
    }
  }
  else if(vec[2].x > vec[4].x && vec[2].x < (vec[4].x+50) && vec[2].y > vec[4].y && vec[2].y < (vec[4].y+50))
  {
    if(!soundPlayed)
    {
      soundPlayed = true;
      song[0].rewind();
      song[0].play();
    }
  }
  else
  {
    soundPlayed = false;
  }
  
  //println(vec[4].y + ", " + (vec[4].y+50));
  //println(vec[2].y);
}

//Flex test
float hypo(PVector v1, PVector v2, PVector v3)
{
  float side1, side2;
  side1 = side2 = 0.0;
  
  side1 = sqrt(sq(v2.x-v1.x)+sq(v2.y-v1.y));
  side2 = sqrt(sq(v3.x-v1.x)+sq(v3.y-v1.y));
  
  //println("side1: " + side1);
  //println("side2: " + side2);
  
  return sqrt(sq(side1)+sq(side2));
}

//v(angle, length, length)
PVector v(float angle)
{
  //tan(angle);
  
  //Cosine
  float cos = cos(angle) * 50;
  
  //Sine
  float sin = sin(angle) * 50;
  
  return new PVector(cos , sin);
}

//Get Quaternion from euler
Quaternion quat(float yaw, float pitch, float roll)
{
  //float [] comp = {heading + initRot[0], elevation + initRot[1], bank + initRot[2]};
  
  float c1 = cos(yaw/2);
  float c2 = cos(pitch/2);
  float c3 = cos(roll/2);
  
  float s1 = sin(yaw/2);
  float s2 = sin(pitch/2);
  float s3 = sin(roll/2);
  
  
  /*
  float c1 = cos(comp[0]/2);
  float c2 = cos(comp[1]/2);
  float c3 = cos(comp[2]/2);
  
  float s1 = sin(comp[0]/2);
  float s2 = sin(comp[1]/2);
  float s3 = sin(comp[2]/2);
  */
  
  float [] result = new float [4];
  
  result[0] = c1*c2*c3 - s1*s2*s3;
  result[1] = s1*s2*c3 + c1*c2*s3;
  result[2] = s1*c2*c3 + c1*s2*s3;
  result[3] = c1*s2*c3 - s1*c2*s3;
  
  //quat2 = new Quaternion(result[0], result[1], result[2], result[3]);
  quat2 = quat1;
  
  return new Quaternion(result[0], result[1], result[2], result[3]);
}

//Attitude compensation test
float [] initialAltitude(float x, float y, float z)
{
  PVector v1 = new PVector(x, y, z);
  PVector v2 = new PVector(0, 0, 9.8);
  
  PVector v3 = v1.cross(v2);
  v3.normalize();
  
  v1.normalize();
  v2.normalize();
  float dot = v1.dot(v2);
  float angle = acos(dot);
  
  float half_sin = sin(0.5*angle);
  float half_cos = cos(0.5*angle);
  float [] quat = {half_cos, half_sin*v3.x, half_sin*v3.y, half_sin*v3.z};
  
  float [] result = new float[3];
  
  float test = quat[1]*quat[2] + quat[3]*quat[0];
  if(test > 0.499)
  {
    result[0] = 2 * atan2(quat[1], quat[0]);
    result[1] = PI/2;
    result[2] = 0;
    return result;
  }
  if(test < -0.499)
  {
    result[0] = - 2 * atan2(quat[1], quat[0]);
    result[1] = - PI/2;
    result[2] = 0;
    return result;
  }
  
  float sqx = sq(quat[1]);
  float sqy = sq(quat[2]);
  float sqz = sq(quat[3]);
  
  result[0] = atan2(2*quat[2]*quat[0] - 2*quat[1]*quat[3] , 1 - 2*sqy - 2*sqz) * 180 / PI;
  result[1] = asin(2*test) * 180 / PI;
  result[2] = atan2(2*quat[1]*quat[0] - 2*quat[2]*quat[3] , 1 - 2*sqx - 2*sqz) * 180 / PI;
  
  println(result[0] + ", " + result[1] + ", " + result[2]);
  
  return result;
}

//Quaternion multiplication
Quaternion multQuat(Quaternion q1, Quaternion q2)
{
  float [] result = new float[4];
  
  result[0] = q1.w*q2.w - (q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);
  result[1] = q1.w*q2.x + q2.w*q1.x + q1.y*q2.z - q1.z*q2.y;
  result[2] = q1.w*q2.y + q2.w*q1.y + q1.z*q2.x - q1.x*q2.z;
  result[3] = q1.w*q2.z + q2.w*q1.z + q1.x*q2.y - q1.y*q2.x;
  
  return new Quaternion(result[0], result[1], result[2], result[3]);
}