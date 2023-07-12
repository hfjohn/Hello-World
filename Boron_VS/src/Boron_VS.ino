// Windows Desktop version
//Libraries
#include <Wire.h>
#include "SparkFun_LPS28DFW_Arduino_Library.h"
#include "Adafruit_VEML7700.h"
#include "SparkFunMAX17043.h"

// Create a new pressure sensor object
LPS28DFW pressureSensor;
// Light Sensor Object
Adafruit_VEML7700 veml = Adafruit_VEML7700();   //VEML7700 instance

//floating point variables
float initial_Light_Sample = 0;
float InitialPressureSensorData;
int PressureSensorData;
float turbBaseline;
float turbCorrected;
float lightMultiplier = 1.11;
float turbCalculated;
float ALS_Filtered = 0;
 float avgLightLevel = 0;
float tempSense = 0;
float DepthDeltaPressure;
float tds;
float calculated_Depth = 0;
float batLevel = 0;
int analogPin = A0;



  String var1 = ("");
  String var2 = ("");
  String var3 = ("");
  String var4 = ("");
  String var5 = ("");
  String var6 = ("");
  String var7 = ("");
  String time_Sent = ("");
  
  int x =0; 
  int pwr_On = D3; 
  int sentx = 0;//count for serial data request
  int laser_Cntrl = D2;
    // Set up battery monitor variables (voltage, state of charge, and alert):
    float voltage = (voltage);
    float soc = (soc);
   FuelGauge fuel;

// I2C address selection
//uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_DEFAULT; // 0x5C
uint8_t i2cAddress = LPS28DFW_I2C_ADDRESS_SECONDARY; // 0x5D//define pressure sensor address
//===========================================================

//=============================================================
void setup()
{

  //Boron Setup......
 SYSTEM_MODE(SEMI_AUTOMATIC)
     Particle.connect();// connect to particle platform
     Time.isValid();
       delay (1000);
   
  Time.setFormat(TIME_FORMAT_ISO8601_FULL);
  Particle.syncTime(); // Synchronize the time with Particle cloud
   Time.zone(-7); // Set the time zone if needed
  
      Serial1.begin (9600); //this is the TX and RX pins on the device
      Serial1.setTimeout(1000); // set new value to "X" milliseconds
  Serial.begin(9600); // set line end to Newline atbottom of serial monitor
  // pinMode D6=output; 
   pinMode(pwr_On, OUTPUT);// activate D4 for aux. power up of external circuitry
   digitalWrite(pwr_On, LOW);
   pinMode(laser_Cntrl, OUTPUT);
   delay (100);// likely not needed
   
   	// Set up the MAX17043 LiPo fuel gauge:
	lipo.begin(); // Initialize the MAX17043 LiPo fuel gaugelipo.quickStart();
    lipo.quickStart();//startup gauge
    initializePowerCfg();//run this function on startup to set pwr configuarations
 

                     


  //********************Begin setup of light sensor************************************
  analogWrite(4,128); //PWM used to set threshold brightness at 0 ntu at orthoganal LED
 //analogWrite(5,200); //PWM used to set threshold brightness for 180 deg. LED
    // Start serial
  while (!Serial) { delay(10); }
  //Serial.begin(9600);
    //AuxSerial.begin(9600);// Port for remote serial TX
    Serial.println("Depth,Temperature, Turbidity and TDS Data Acquisition");
if (!veml.begin()) {
    Serial.println("VEML7700 Sensor not found, hanging here...");
    //while (1);
  }
  Serial.println("Sensor found");
  veml.setGain(VEML7700_GAIN_2);
  veml.setIntegrationTime(VEML7700_IT_800MS);
veml.setGain(VEML7700_GAIN_2);
  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }

  //veml.setLowThreshold(10000); not used
  //veml.setHighThreshold(20000);not used
  veml.interruptEnable(false);//not used

  
    // Initialize the I2C library
    Wire.begin();
    
     delay (2000); 
       //Sample light levels 10 times and average the result. 
       // Used for the initial calibration of sensor
       get_ALS_Reading();
  initial_Light_Sample = avgLightLevel;
  delay (100); 


   //initial_Light_Sample=(ALS_Filtered/10);// store result in var. avgLightLevel
    Serial.print("*******Initial ALS:******* "); Serial.println(initial_Light_Sample);
    ALS_Filtered = 0;// clear lightLevelAvg varable
     // turbBaseline= (avgLightLevel);
//=============================== begin pressure sensor =======================================
 
 
    // Check if pressure sensor is connected and initialize
    // Address is optional (defaults to 0x5C)
    delay(100);
    if (pressureSensor.begin(i2cAddress) != LPS28DFW_OK)//changed while to if to pass through once
    {
        // Not connected, inform user
        Serial.println("Error: LPS28DFW not connected, check wiring and I2C address!");
        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("Depth Sensor Connected...");
      pressureSensor.getSensorData();
    InitialPressureSensorData = (pressureSensor.data.pressure.hpa);//check initial pressure reading
 Serial.print("Initial air pressure reading: "); Serial.println(InitialPressureSensorData);
 delay (100);
}

void loop()
{
  Get_Bat_Level();
  get_Temperature();  
  get_Pressure_Data();
  get_ALS_Reading(); 
  send_Over_USB_Link();//so we can look at it on the serial monitor
  

  if (Particle.connected() && Time.isValid()) {
        
    // This is where new combined Boron code goes....
      send_Remote_Data();
}
 //delay (100);
 
  //send_Over_Remote_Serial();// send to particle device
  
  /*
  while(AuxSerial.available() > 0)
  {
    int request_Data  = AuxSerial.parseInt();
    //Serial.println(request_Data);
    if (request_Data == 99){ 
  Serial.print("Received Request from Master...=");
   Serial.println(request_Data);
  Serial.println("");
  Serial.println(""); 
  //z = 1; //used for flow control

  send_Over_Remote_Serial();// send to particle device
    }//otherwise do nothing
  
  }
      
  */
}


void Get_Bat_Level(){
batLevel = analogRead(analogPin);  // read the analog ADC Pin A0, output = 0-4095 (for 3.3V)
batLevel = (batLevel*.001611); // convert to a voltage level, i,e for 6.6V, X/2  res.divider, out -= 3.3V, count = 4095
 }

void get_Temperature(){
  pressureSensor.getSensorData();
  // Print temperature and pressure
 //Serial.print("Water Temperature: ");
 tempSense = (pressureSensor.data.heat.deg_c);// post temperature
 //Serial.print(pressureSensor.data.heat.deg_c);
}
 void get_ALS_Reading(){
    Serial.println("");
     Serial.println("");//White Space
   
    // Only print every second
    //delay(1000);
    //Sample light levels 10 times and average the result. Store in X4
   for (int i= 0; i<10; i++){
  ALS_Filtered = (veml.readALS()) + (ALS_Filtered);
  delay (100);//delay to chaeck longer term readings
  //Serial.println  (veml.readALS());
  //delay (1000); 
  
   }
   avgLightLevel=(ALS_Filtered/10);// store result in var. avgLightLevel
   ALS_Filtered = 0;// clear varable
      
      turbCorrected = 1;//(avgLightLevel-turbBaseline;
      //Serial.println("");
      //Serial.println("");  
       //Serial.print("Raw, Uncorrected Laser Output: "); Serial.print(avgLightLevel);//uncorrected light sensor output
       //Serial.print(":;"); 
       //Serial.println(""); 
       lightMultiplier = 1.0; //set initialy to nominal value on each reading
       turbBaseline = (avgLightLevel-initial_Light_Sample);
       //Serial.print("Baseline ALS Output: "); Serial.println(turbBaseline);//
      //Serial.print("Corrected ALS: "); Serial.println(turbBaseline);//  = (Light Sensor outtput - Initial Cal sensor value)
    //Serial.println(""); 
    if (avgLightLevel < 10.0){
      //Serial.println("Less than NTU 10...");
      lightMultiplier = .50;
      //avgLightLevel=avgLightLevel-4;
    }
    if (avgLightLevel < 1){
      //lightMultiplier = .50;
      //avgLightLevel=avgLightLevel+.5;
      //if (avgLightLevel >500){
      //lightMultiplier = .50;
      //avgLightLevel=avgLightLevel+.5;
    }
    turbCalculated = (avgLightLevel * lightMultiplier);
    //Serial.print("  Average Turbidity: "); Serial.println(abs(turbCalculated));// Correction divisor for actual turbidity calculation
     //Serial.println("");  


  
 }
void get_Pressure_Data(){
   
    //Serial.print("Atmospheric Pressure (hPa): ");
    //Serial.print(pressureSensor.data.pressure.hpa);
    //Altitude (in feet) = 100 + 145366.45 * (1 - (1010 / 1013.25)^(0.190284))
    PressureSensorData = (pressureSensor.data.pressure.hpa);
    //Serial.print (":  ");
   float  Altitude = (145366.45 );
   float pressure1 =  pow((PressureSensorData/1013.25),0.190284);
   pressure1 = 1 - pressure1;
   // (1 - (pressure / 1013.25)^(0.190284)));
   Altitude = 1 + (pressure1*Altitude);// not accurate at sea level but close
    //Serial.print (Altitude);
    //Serial.println (" Feet Altitude");
    DepthDeltaPressure = (((pressureSensor.data.pressure.hpa)-(InitialPressureSensorData))*1);
     //Serial.print ("Depth Reading, In meters:"); Serial.println (((pressureSensor.data.pressure.hpa)-(InitialPressureSensorData))/98.1);
     //Serial.println ("");
    //Serial.print ("Delta Pressure Readings: "); Serial.println (DepthDeltaPressure);
     //Serial.println ("");  
     calculated_Depth = (DepthDeltaPressure/98.1);
     
}
void send_Over_USB_Link(){
    Serial.print(" Temp:");
    Serial.print (tempSense);
    Serial.print (",");
    Serial.print(" Depth:");
    Serial.print(((pressureSensor.data.pressure.hpa)-(InitialPressureSensorData))/98.1);
    Serial.print (",");
    Serial.print(" Pressure:");
    Serial.print(PressureSensorData);
    Serial.print (",");
    Serial.print(" ALS: ");
    Serial.print(avgLightLevel);
    Serial.print (",");
    Serial.print(" Dissolved Solids (ppm): ");
    Serial.print(200);
    Serial.print (",");
    Serial.print(" ILS: ");
    Serial.print(initial_Light_Sample);
    Serial.print  ("Pwr.Supply:");
    Serial.print(batLevel);

    float datachksum = (tempSense+PressureSensorData+200+calculated_Depth);
    //
    Serial .print ("Checksum: ");
    Serial.println (datachksum);
    

    //delay (3000);
}

//Add Boron Code here


                     
            // Connect the remote Micro to the TX and RX lines on the Particle device, which are not the same as the USB serial   
           




   bool initializePowerCfg() {
    Log.info("Initializing Power Config");
    const int maxCurrentFromPanel = 900;            // Not currently used (100,150,500,900,1200,2000 - will pick closest) (550mA for 3.5W Panel, 340 for 2W panel)
    SystemPowerConfiguration conf;
    System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration

    conf.powerSourceMaxCurrent(maxCurrentFromPanel) // Set maximum current the power source can provide  3.5W Panel (applies only when powered through VIN)
        .powerSourceMinVoltage(4000) // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(maxCurrentFromPanel) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        //.feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); 
        .feature(SystemPowerFeature::DISABLE_CHARGING);// For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
}
void send_Remote_Data(){ 
    
    FuelGauge fuel;
//Log.info( "voltage=%.2f", fuel.getVCell() );
//float vsys =(fuel.getVCell() );
    //use Serial1 to acquire sensor data, post to PC via Serial
    //But forst check fuel gauge...
    	// lipo.getVoltage() returns a voltage value (e.g. 3.93)
	voltage = lipo.getVoltage();
	// lipo.getSOC() returns the estimated state of charge (e.g. 79%)
	soc = lipo.getSOC();
	soc = soc/2.56;
	
    digitalWrite(pwr_On, HIGH);
    analogWrite(laser_Cntrl, 255); //PWM output on D2 for 180 Deg. LED Reflective control

  //delay(10);
  var2= String(tempSense,1);
  //delay(10);
  var3 = (PressureSensorData);
  //delay(10);
  float als_Correction =(avgLightLevel*1.00);// correction factor for turbidity
  var4 = String(als_Correction,1);
    //var5 = soc;
  var5 = String(batLevel,2);//state of charge or voltage
  //delay(10);
 
  //delay(10);
  var6 = String(calculated_Depth,2);
   var7 = (300);
 x = 0;
 // Send to serial monitor to check syntax;
Serial.println("");
Serial.println("String Concantenations:"); 
Serial.print("Server Time Added");
Serial.println(var1);//output to serial console: NOTE: Time variable is added at Server!! No Output here currently for var1
//delay(10);// Wed May 21 01:08:47 2014
Serial.print("Temperature: ");//delay(10);
Serial.println(var2);
  //Serial.println(temp);
//Serial.println(temp);
//delay(10);
Serial.print("Atm. Pressure: ");
Serial.println(var3);
Serial.print("ALS: ");//delay(10);
Serial.println(var4);
Serial.print("Input Voltage: ");///delay(10);
Serial.println(var5);
//delay(10);
Serial.print("Depth: ");
Serial.println(var6);
Serial.print("TDS Placeholder: ");
Serial.println(var7);
Serial.print  ("Pwr.Supply Voltage: ");
    Serial.println(batLevel);
delay (1);
digitalWrite(pwr_On, LOW);
 sentx = 0;
send_Webhook();

        }
    
    


 



void go_To_Sleep(){
    
    digitalWrite(pwr_On, LOW);
    delay (1000); 
    sentx = 0; //reset counter for data tx
   SystemSleepConfiguration config;
   
   int wakeInSeconds = 60; // Figure out how long to sleep , Set seconds until wake
config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .duration(wakeInSeconds* 1000L)
      .network(NETWORK_INTERFACE_CELLULAR, SystemSleepNetworkFlag::INACTIVE_STANDBY);// Network Standby mode for rapid connect
      //cellular_off();
// Ready to sleep
//Particle.disconnect();
Serial.println ("Powering off, going to sleep...");
//Cellular.off();//===================================================================================================================
SystemSleepResult result = System.sleep(config);    // Device sleeps here

if (result.wakeupPin() == D2) {              // Woke by pin interrupt
   // pin wake code here
}
else {   // time wake code here
     send_Remote_Data();
    
}

// // 
//SystemSleepConfiguration config;
//config.mode(SystemSleepMode::ULTRA_LOW_POWER)
//      .duration(15min)
//      .network(NETWORK_INTERFACE_CELLULAR, SystemSleepNetworkFlag::INACTIVE_STANDBY);
}

//////
    
void send_Webhook(){

 
//Send data to site via webhook;
Serial.println ("***************************");
Serial.println ("");
 Serial.println ("Sending Webhook...");
 Serial.println(Time.timeStr()); // Wed May 21 01:08:47 2014
   String eventData = String (String("{")+"\"var2\"") + String(":") + (var2) + "," + String ("\"var3\"") + String(":") + (var3)+String(",")+ String ("\"var4\"") + String(":") + (var4)+String(",") +String ("\"var5\"") + String(":") + (var5)+String("}");// Combine the two variables into a single string, separated by a comma
  Particle.publish("vars-test", eventData, PRIVATE); // Publish the event with the combined string as the data
  delay (1); // wait for 60 seconds to take next reading....print time at serial and publish (in mSeconds)
   go_To_Sleep();
   //And repeat!
}


void TimeNow() {
    Time.zone(-7);

Serial.println(Time.minute());
 Particle.syncTime(); // Synchronize the time with Particle cloud
  
//TimeElements wakeupTime;
 //wakeupTime.Hour = wakeupHour;
 Serial.print(Time.hour());
 //wakeupTime.Minute = wakeupMinute;

  
}

/*
MAX17043_Simple_Serial.cpp
SparkFun MAX17043 Example Code
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 22, 2015
https://github.com/sparkfun/SparkFun_MAX17043_Particle_Library

This file demonstrates the simple API of the SparkFun MAX17043 Particle library.
Pair the Photon up to a SparkFun Photon Battery Shield
(https://www.sparkfun.com/products/13626), and away you go!

This example will print the gauge's voltage and state-of-charge (SOC) readings
to both serial (9600 baud) and out to the Particle cloud. Navigate to
https://api.particle.io/v1/devices/{DEVICE_ID}/voltage?access_token={ACCESS_TOKEN}
https://api.particle.io/v1/devices/{DEVICE_ID}/soc?access_token={ACCESS_TOKEN}
https://api.particle.io/v1/devices/{DEVICE_ID}/alert?access_token={ACCESS_TOKEN}

And read your Photon's battery charge from the Cloud!

Development environment specifics:
	IDE: Particle Build
	Hardware Platform: Particle Photon
	                   SparkFun Photon Battery Shield

This code is released under the MIT license.

Distributed as-is; no warranty is given.
****************************************************************************

#include "SparkFunMAX17043.h" // Include the SparkFun MAX17043 library

double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
bool alert; // Variable to keep track of whether alert has been triggered

void setup()
{
	Serial.begin(9600); // Start serial, to output debug data

	// Set up Spark variables (voltage, soc, and alert):
	Particle.variable("voltage", voltage);
	Particle.variable("soc", soc);
	Particle.variable("alert", alert);
	// To read the values from a browser, go to:
	// http://api.particle.io/v1/devices/{DEVICE_ID}/{VARIABLE}?access_token={ACCESS_TOKEN}



	// Quick start restarts the MAX17043 in hopes of getting a more accurate
	// guess for the SOC.
	lipo.quickStart();

	// We can set an interrupt to alert when the battery SoC gets too low.
	// We can alert at anywhere between 1% - 32%:
	lipo.setThreshold(20); // Set alert threshold to 20%.
}

void loop()
{
	// lipo.getVoltage() returns a voltage value (e.g. 3.93)
	voltage = lipo.getVoltage();
	// lipo.getSOC() returns the estimated state of charge (e.g. 79%)
	soc = lipo.getSOC();
	// lipo.getAlert() returns a 0 or 1 (0=alert not triggered)
	alert = lipo.getAlert();

	// Those variables will update to the Spark Cloud, but we'll also print them
	// locally over serial for debugging:
	Serial.print("Voltage: ");
	Serial.print(voltage);  // Print the battery voltage
	Serial.println(" V");

	Serial.print("Alert: ");
	Serial.println(alert);

	Serial.print("Percentage: ");
	Serial.print(soc); // Print the battery state of charge
	Serial.println(" %");
	Serial.println();

	delay(500);
}

*/


