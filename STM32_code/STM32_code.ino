// Tested code with STM32F405 (also with multiple I2C sensors) 
// Code done by Alejandro Boadella EETAC student

//Download libraries: Compass, GPS, "Fake/imitation" Serial Sender,I2C bus
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Creation of compass object
QMC5883LCompass compass;

// GPS bauds, gps object and serial connection to pins RX=PC2 and TX=PC3
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(PC2, PC3);

// Creation of the vector that contains all the data transmitted to the raspberry pi
float DATA[3]={};
// Timer
unsigned long previousMillis=millis();

// Setup loop only run once
void setup() {
  //Serial bus comunication bus the rasp at 115200 bauds
  Serial.begin(115200);
  // Initiation of the compass
  compass.init();
  // Creation of the 'fake' serial bus with the GPS at 9600 bauds
  ss.begin(GPSBaud);
  // Calibrate the compass, in order to change and recalibrate the compass: use the library of the compass --> QMC5883LCompass.h 
  compass.setCalibrationOffsets(1006.00, 75.00, -1510.00);
  compass.setCalibrationScales(1.15, 1.22, 0.76);

}

void calib_compass()
{
  delay(5000);
  Serial.println("CALIBRATING. Move your sensor!!");
  compass.calibrate();
  compass.setCalibrationOffsets(compass.getCalibrationOffset(0),compass.getCalibrationOffset(1),compass.getCalibrationOffset(2));
  compass.setCalibrationScales(compass.getCalibrationScale(0), compass.getCalibrationScale(1),compass.getCalibrationScale(2));
  Serial.println("DONE");
}

void measure_compass() {
  
    // Read compass values (the output will be the magnetic north, not the true north!!, 2º of variation respect Catalonia)
    compass.read();
    // Return Azimuth reading, from 0 to 359 degrees
    float comp = compass.getAzimuth();
    //if (comp<0){
    //  comp=360+comp;
    //}
    // Save the information in the data vector
    DATA[0]=comp+180;
}

void measure_gps() {
    // See if the gps is available and read data
    if (ss.available()){
        gps.encode(ss.read());
      }
    // If the location is valid, read the data and save in the vector data the lattitude and the longitude
    if (!(gps.location.isValid())){

    }
    else{
      DATA[1]=gps.location.lat();
      DATA[2]=gps.location.lng();
    }  
    DATA[1]=gps.location.lat();
    DATA[2]=gps.location.lng();
}

void measure_height(){
    // Smartdelay is used to not lose connection and see if it is available
    smartDelay(0);
    if (!(gps.altitude.isValid())){ 
    }
    else{
      DATA[3]=gps.altitude.meters();
    }
    // Finally save the height data inside the vector
}

void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void send_bus(){
    // Send via serial and ASCII all the data retained in ther vector
    Serial.print(DATA[0]);
    Serial.print(",");
    Serial.print(DATA[1],7);
    Serial.print(",");
    Serial.print(DATA[2],7);
    Serial.print(",");
    Serial.println(DATA[3],1);
}

//Main loop
void loop() {
  // With the timer, every 1,5s measure compass, gps and send all the data through the serial bus
  // Measure height must be done periodically with the objective of receiveing data
  if (millis()- previousMillis> 1500ul)
  {
    measure_compass();
    measure_gps();
    send_bus();
    previousMillis = millis();
    
  }
  measure_height();
}
