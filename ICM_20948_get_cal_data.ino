// get raw data, using Sparkfun library
// Derived by SJR from
/****************************************************************
   Example1_Basics.ino
   ICM 20948 Arduino Library Demo
   Use the default configuration to stream 9-axis IMU data
   Owen Lyke @ SparkFun Electronics
   Original Creation Date: April 17 2019

   Please see License.md for the license information.

   Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
  // On the SparkFun 9DoF IMU breakout the default is 1, and when \
  // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// gyro offset values for calibration
long gyro[3] = {0};
int offset_count = 500; //average this many values for gyro
int acc_mag_count = 300; //collect this many values for acc/mag calibration

void setup()
{

  SERIAL_PORT.begin(9600);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  // find gyro offsets
  SERIAL_PORT.println(F("Hold sensor still for gyro offset calibration ..."));
  delay(5000);

  float goff;
  int i;

  for (i = 0; i < offset_count; i++) {
    if (myICM.dataReady())
    {
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
      gyro[0] += myICM.agmt.gyr.axes.x;
      gyro[1] += myICM.agmt.gyr.axes.y;
      gyro[2] += myICM.agmt.gyr.axes.z;
    }
  } //done with gyro
  SERIAL_PORT.print("Gyro offsets x, y, z: ");
  for (i = 0; i < 3; i++) {
    goff = (float)gyro[i] / offset_count;
    SERIAL_PORT.print(goff, 1);
    SERIAL_PORT.print(", ");
  }
  SERIAL_PORT.println();

  SERIAL_PORT.println(F("Turn sensor SLOWLY and STEADILY in all directions until done"));
  delay(5000);
  SERIAL_PORT.println(F("Starting..."));

  // get values for calibration of acc/mag
  for (i = 0; i < acc_mag_count; i++) {
    if (myICM.dataReady())
    {
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
      printRawAGMT( myICM.agmt );     // raw values, taken directly from the agmt structure
      delay(200);
    }
    else
    {
      delay(100); //wait for data ready
    }
  }
  SERIAL_PORT.print(F("Done collecting"));
}

void loop() {}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(agmt.acc.axes.z);
  SERIAL_PORT.print(", ");

  SERIAL_PORT.print(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(agmt.mag.axes.z);

  SERIAL_PORT.println();
}
