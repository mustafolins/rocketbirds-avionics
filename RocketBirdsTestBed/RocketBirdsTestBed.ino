#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

// Creating bmp 280 instance
#define BMP_SCK 9
#define BMP_MISO 8
#define BMP_MOSI 7 
#define BMP_CS 6
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

// Creating lsm instance
//1 using i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

const int chipSelect = 10;

bool bmpInitialized = false;
bool lsmInitialized = false;
bool sdInitialized = false;

void setupLsmSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void setupBmpSensor(){
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void setup() {
  //Setting the Serial Baud Rate
  Serial.begin(115200);

  bmpInitialized = bmp.begin();
  lsmInitialized = lsm.begin();

  sdInitialized = SD.begin(chipSelect);

  setupBmpSensor();
                  
  setupLsmSensor();
}

void readBmpData(){
  Serial.println("BMP 280 Data:");
  Serial.print("Temperature: ");
  Serial.println(bmp.readTemperature());
  Serial.print("Pressure: ");
  Serial.println(bmp.readPressure());
  Serial.print("Altitude: ");
  Serial.println(bmp.readAltitude(1013.25));
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println("BMP 280 Data:");
  dataFile.print("Temperature: ");
  dataFile.println(bmp.readTemperature());
  dataFile.print("Pressure: ");
  dataFile.println(bmp.readPressure());
  dataFile.print("Altitude: ");
  dataFile.println(bmp.readAltitude(1013.25));
  
  dataFile.close();
}

void printLsmData(sensors_event_t a, sensors_event_t m, sensors_event_t g, sensors_event_t temp){
  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.print("Accel X: "); dataFile.print(a.acceleration.x); dataFile.print(" m/s^2");
  dataFile.print("\tY: "); dataFile.print(a.acceleration.y);     dataFile.print(" m/s^2 ");
  dataFile.print("\tZ: "); dataFile.print(a.acceleration.z);     dataFile.println(" m/s^2 ");

  dataFile.print("Mag X: "); dataFile.print(m.magnetic.x);   dataFile.print(" gauss");
  dataFile.print("\tY: "); dataFile.print(m.magnetic.y);     dataFile.print(" gauss");
  dataFile.print("\tZ: "); dataFile.print(m.magnetic.z);     dataFile.println(" gauss");

  dataFile.print("Gyro X: "); dataFile.print(g.gyro.x);   dataFile.print(" dps");
  dataFile.print("\tY: "); dataFile.print(g.gyro.y);      dataFile.print(" dps");
  dataFile.print("\tZ: "); dataFile.print(g.gyro.z);      dataFile.println(" dps");

  dataFile.close();
}

void readLsmData(){
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  Serial.println("Accelerometer");

  lsm.getEvent(&a, &m, &g, &temp); 
  printLsmData(a, m, g, temp);

  Serial.println();
}

void loop() {
  delay(1000);
  readBmpData();
  readLsmData();
}
