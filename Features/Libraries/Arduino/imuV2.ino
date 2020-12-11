#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(9600);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  /* Display the floating point data */
  Serial.println(event.orientation.x, 4);
  delay(100);
  if (Serial.available()) {
    int counter = 0;
    bool flag {true};
    if (Serial.read() == 'h') {
        Serial.println(event.orientation.x,4);
        while(flag){
        if (Serial.read() == 'g'){
          flag = false;  
        }
    } 
    }
  }
}
