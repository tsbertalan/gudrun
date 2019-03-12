#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(30303);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no accel detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no mag detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    Serial.println("Oops, no gyro detected ... Check your wiring!");
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
}


/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t gyro_event;
  sensors_vec_t   orientation;

  typedef struct {
    float ax, ay, az, gx, gy, gz, roll, pitch, heading;
  } DataPackage;

  DataPackage package;

  // /* Calculate pitch and roll from the raw accelerometer data */
  // accel.getEvent(&accel_event);

  // Serial.print(accel_event.acceleration.x);
  // Serial.print(F(", "));
  // Serial.print(accel_event.acceleration.y);
  // Serial.print(F(", "));
  // Serial.println(accel_event.acceleration.z);

  // if (dof.accelGetOrientation(&accel_event, &orientation))
  // {
  //   /* 'orientation' should have valid .roll and .pitch fields */
  //   Serial.print(F("Roll: "));
  //   Serial.print(orientation.roll);
  //   Serial.print(F("; "));
  //   Serial.print(F("Pitch: "));
  //   Serial.print(orientation.pitch);
  //   Serial.print(F("; "));
  // }
  
  // /* Calculate the heading using the magnetometer */
  // mag.getEvent(&mag_event);
  // if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  // {
  //   /* 'orientation' should have valid .heading data now */
  //   Serial.print(F("Heading: "));
  //   Serial.print(orientation.heading);
  //   Serial.print(F("; "));
  // }
  // Serial.println(F(""));



  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  gyro.getEvent(&gyro_event);

  // Serial.print(accel_event.acceleration.x, 4);
  // Serial.print(F(","));
  // Serial.print(accel_event.acceleration.y, 4);
  // Serial.print(F(","));
  // Serial.print(accel_event.acceleration.z, 4);
  // Serial.print(F(","));

  // Serial.print(gyro_event.gyro.x, 4);
  // Serial.print(F(","));
  // Serial.print(gyro_event.gyro.y, 4);
  // Serial.print(F(","));
  // Serial.print(gyro_event.gyro.z, 4);
  
  package.ax = accel_event.acceleration.x;
  package.ay = accel_event.acceleration.y;
  package.az = accel_event.acceleration.z;

  package.gx = gyro_event.gyro.x;
  package.gy = gyro_event.gyro.y;
  package.gz = gyro_event.gyro.z;


  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
    // Serial.print(F(","));
    // Serial.print(orientation.roll);
    // Serial.print(F(","));
    // Serial.print(orientation.pitch);
    // Serial.print(F(","));
    // Serial.print(orientation.heading);

    package.roll = orientation.roll;
    package.pitch = orientation.pitch;
    package.heading = orientation.heading;
  
    Serial.write('S');
    Serial.write((byte*)&package, sizeof(package));
    Serial.write('E');
  }

  // Serial.println();

  
  delayMicroseconds(100);
}
