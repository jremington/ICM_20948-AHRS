//
// Tilt compensated compass  S.J. Remington 2/2021
// Requires the Sparkfun ICM_20948 library

// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion program ICM_20948_get_cal_data
//
/*
  Sparkfun ICM_20948
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  ICM_20948 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VIN ------------- 5V or 3.3V
   GND ------------- GND

*/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//////////////////////////
// ICM_20948 Library Init //
//////////////////////////
// default settings for accel and magnetometer 

#define WIRE_PORT Wire // desired Wire port.
#define AD0_VAL 1      // value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when
// the ADR jumper is closed the value becomes 0

ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using ICM_20948_cal and Magneto
//The compass will NOT work well or at all if these are not correct

//Accel scale: divide by 16604.0 to normalize
float A_B[3]
{   79.60,  -18.56,  383.31};

float A_Ainv[3][3]
{ {  1.00847,  0.00470, -0.00428},
  {  0.00470,  1.00846, -0.00328},
  { -0.00428, -0.00328,  0.99559}
};

//Mag scale divide by 369.4 to normalize
float M_B[3]
{ -156.70,  -52.79, -141.07};

float M_Ainv[3][3]
{ {  1.12823, -0.01142,  0.00980},
  { -0.01142,  1.09539,  0.00927},
  {  0.00980,  0.00927,  1.10625}
};

// local magnetic declination in degrees
float declination = 14.84;

/*
  This tilt-compensated code assumes that the ICM_90248 sensor board is oriented with Accel X pointing
  to the North, Y pointing West, and Z pointing up for heading = 0 degrees
  The code compensates for tilts of up to 90 degrees away from horizontal.
  Facing vector p is the direction of travel and allows reassigning these directions.
  It should be defined as pointing forward,
  parallel to the ground, with coordinates {X, Y, Z} (in magnetometer frame of reference).
*/
float p[] = {1, 0, 0};  //X marking on sensor board points toward yaw = 0

#define PRINT_SPEED 1000 // ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

void setup()
{
  Serial.begin(9600);
  while (!Serial); //wait for connection
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }
}

void loop()
{
  static float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data

  // Update the sensor values whenever new data is available
  if ( imu.dataReady() ) imu.getAGMT();

  if (millis() - lastPrint > PRINT_SPEED)
  {
    get_scaled_IMU(Axyz, Mxyz);  //apply relative scale and offset to RAW data. UNITS are not important

    // reconcile mag and accel coordinate axes
    // Note: the illustration in the ICM_90248 data sheet implies that the magnetometer
    // Y and Z axes are inverted with respect to the accelerometer axes, verified to be correct (SJR).

    Mxyz[1] = -Mxyz[1]; //align magnetometer with accelerometer (reflect Y and Z)
    Mxyz[2] = -Mxyz[2];

    Serial.print(Axyz[0]);
    Serial.print(", ");
    Serial.print(Axyz[1]);
    Serial.print(", ");
    Serial.print(Axyz[2]);
    Serial.print(", ");
    Serial.print(Mxyz[0]);
    Serial.print(", ");
    Serial.print(Mxyz[1]);
    Serial.print(", ");
    Serial.println(Mxyz[2]);
    //  get heading in degrees
    Serial.print("Heading: ");
    Serial.println(get_heading(Axyz, Mxyz, p, declination));
    lastPrint = millis(); // Update lastPrint time
  }
  // consider averaging a few headings for better results
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
// applies magnetic declination
int get_heading(float acc[3], float mag[3], float p[3], float magdec)
{
  float W[3], N[3]; //derived direction vectors

  // cross "Up" (acceleration vector, g) with magnetic vector (magnetic north + inclination) with  to produce "West"
  vector_cross(acc, mag, W);
  vector_normalize(W);

  // cross "West" with "Up" to produce "North" (parallel to the ground)
  vector_cross(W, acc, N);
  vector_normalize(N);

  // compute heading in horizontal plane, correct for local magnetic declination in degrees

  float h = -atan2(vector_dot(W, p), vector_dot(N, p)) * 180 / M_PI; //minus: conventional nav, heading increases North to East
  int heading = round(h + magdec);
  heading = (heading + 720) % 360; //apply compass wrap
  return heading;
}

// subtract offsets and correction matrix to accel and mag data

void get_scaled_IMU(float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply offsets (bias) and scale factors from Magneto
  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// basic vector operations
void vector_cross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}
