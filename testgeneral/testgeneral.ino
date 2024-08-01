// Tested code with Arduino UNO (also with multiple I2C sensors) 
// Code done by Alejandro Boadella EETAC student

//Download libraries: Compass, GPS, "Fake/imitation" Serial Sender, SPI bus, I2C bus
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>

// Creation of compass object
QMC5883LCompass compass;

// GPS bauds, gps object and serial connection to pins 4 and 3
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

//Declaration of variables and Timer
static const float latt;
static const float longi;
static const float alt;
unsigned long previousMillis=millis();

// Vctor that will conatin all the data that will be send and a counter for the vector
byte  byteArray[]={0x01,0x02,0x03,0x04};
int counter=0;

// Setup loop only run once
void setup() {
   //Serial bus comunication bus the rasp at 115200 bauds
  Serial.begin(115200);
  // Initiation of the compass
  compass.init();
  // Creation of the 'fake' serial bus with the GPS at 9600 bauds
  ss.begin(GPSBaud);
   // Calibrate the compass, in order to change and recalibrate the compass: use the library of the compass --> QMC5883LCompass.h 
  compass.setCalibrationOffsets(1221.00, -4.00, -1818.00);
  compass.setCalibrationScales(1.13, 1.19, 0.78);
  
  // Declaration of Miso output for SPI bus and settign device as a slave
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
}

static void measure_compass() {
    // Read compass values (the output will be the magnetic north, not the true north!!, 2º of variation respect Catalonia)
    compass.read(); 
    // Return Azimuth reading, from 0 to 359 degrees
    int longInt = compass.getAzimuth();
    if (longInt<0){
      longInt=360+longInt;
    }
    // Save the information in the data vector in byte form
       
    //Serial.print("Compass: ");
    //Serial.println(longInt);
    byteArray[5] = (int)((longInt >> 8) & 0XFF);
    byteArray[4] = (int)((longInt & 0XFF));
    //Serial.println(byteArray[5],HEX);
    //Serial.println(byteArray[4],HEX);
}

static void measure_gps() {
    // See if the gps is available and read data
    if (ss.available()){
        gps.encode(ss.read());
      }
    // If the location is valid, read the data and save in the vector data the lattitude and the longitude
    if (!(gps.location.isValid())){
    }
    else{
      float latt=gps.location.lat();
      float longi=gps.location.lng();
      //Creation of an extra byte of information to determine signs and others
      if (latt<0 and longi<0){
         byteArray[6]=0x00;
      }
      
      if (latt<0 and longi>0){
         byteArray[6]=0x01;
      }
      
      if (latt>0 and longi<0){
         byteArray[6]=0x02;
      }
      else{
         byteArray[6]=0x03;
      }
      //save the data in another format and multiply the data by 10^10
      uint64_t latts= trunc((abs(latt*pow(10,10))));
      uint64_t longis= trunc(abs(longi*pow(10,10)));
      
      //Pass the dat from integer to byte format and save the data inside the vector
      byteArray[12] = ((latts >> 40) & 0XFF);
      byteArray[11] = ((latts >> 32) & 0XFF);
      byteArray[10] = ((latts >> 24) & 0XFF);
      byteArray[9] = ((latts >> 16) & 0XFF);
      byteArray[8] = ((latts >> 8) & 0XFF);
      byteArray[7] = ((latts & 0XFF));

      //Serial.println(latt,20);
      //Serial.println(byteArray[12],HEX);
      //Serial.println(byteArray[11],HEX);
      //Serial.println(byteArray[10],HEX);
      //Serial.println(byteArray[9],HEX);     
      //Serial.println(byteArray[8],HEX);
      //Serial.println(byteArray[7],HEX);
      
      //Pass the dat from integer to byte format and save the data inside the vector
      byteArray[18] = ((longis >> 40) & 0XFF);
      byteArray[17] = ((longis >> 32) & 0XFF);
      byteArray[16] = ((longis >> 24) & 0XFF);
      byteArray[15] = ((longis >> 16) & 0XFF);
      byteArray[14] = ((longis >> 8) & 0XFF);
      byteArray[13] = ((longis & 0XFF));

      //Serial.println(longi,20);
      //Serial.println(byteArray[18],HEX);
      //Serial.println(byteArray[17],HEX);
      //Serial.println(byteArray[16],HEX);
      //Serial.println(byteArray[15],HEX);     
      //Serial.println(byteArray[14],HEX);
      //Serial.println(byteArray[13],HEX);
    
    } 
}

static void measure_height(){
    // Smartdelay is used to not lose connection and see if it is available
    smartDelay(0);  
    if (!(gps.altitude.isValid())){
    }
    else{
      float alt=gps.altitude.meters();
      int alti= (int) alt;
      byteArray[20] = (int)((alti >> 8) & 0XFF);
      byteArray[19] = (int)((alti & 0XFF));
      //Serial.print("ALT: ");
      //Serial.println(alti);
    }
    // Finally save the height data inside the vector
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void send_bus(){
    //If the SPI bus detects signal from the raspberry pi it will send a byte every loop of the arduino, thus the counter.
    if ((SPSR & (1 << SPIF)) != 0)
    {
      // Send the byte of the data vector to the rasp and then change into another byte
      SPDR = byteArray[counter];
      counter++;
      
      if (counter==21 ){
        counter=0;
      }
    }
}
//Main loop
void loop() {
  // With the timer, every 1s measure compass, gps
  // Measure height must be done periodically with the objective of receiveing data
  if (millis()- previousMillis> 1000ul)
  {
    measure_compass();
    measure_gps();
    previousMillis = millis();
  }
  // Send 1 byte data through the SPI bus every loop
  measure_height();
  send_bus();
}
