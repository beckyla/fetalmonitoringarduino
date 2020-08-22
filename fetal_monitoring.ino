/*INCLUDE LIBRARIES*/
#include <ArduinoBLE.h> //Include the ArduinoBLE library
#include <SPI.h> //Include the SPI library 
#include "ADAS1000.h" //include ADAS100 library 
#include "Filter.h"
#include <vector> 
using std::vector;
#include <string>
using namespace std;

/*FETAL KICK VARIABLES*/
int pLED_pin[] = {2, 3, 4, 5, 6, 7, 8};
int piezo_pin[] = {A0, A1, A2, A3, A4, A5};
float sensorValue[] = {0, 0, 0, 0, 0, 0};
int fetalkick_count = 0;
long previousPiezoSensorMillis;
vector<int> sensor0;
vector<int> sensor1;
vector<int> sensor2;
vector<int> sensor3;
vector<int> sensor4;
vector<int> sensor5;
int piezoSend = 0;
int toggle[] = {0, 0, 0, 0, 0, 0};
char buff[20];
String sendStr = "";

/*BLUETOOTH VARIABLES*/
int LED_pin = A6;
int ONBOARD_LED = 13;
int button_pin = 10;
int button_value = 0;
int lastBtnPressMillis = 0;
int interval = 3000; //3 seconds 
int bluetoothState = 0;

/*BLE CONNECTION VARIABLES*/
// Create a BLE service 
BLEService monitoringService("19B10000-E8F2-537E-4F6C-D104768A1214"); //Temporary UUID
// Create the BLECharacteristics
BLECharacteristic fetalKickChar("19B10000-E8F2-537E-4F6C-D104768A1216", BLERead, 2); //Temprary UUID
BLECharacteristic fetalECGChar("19B10000-E8F2-537E-4F6C-D104768A1218", BLERead|BLENotify, 2); //Temporary UUID
BLECharacteristic rawPiezoChar("19B10000-E8F2-537E-4F6C-D104768A1224", BLERead|BLENotify, 20); //Temporary UUID

/*PIEZO FILTERING VARIABLES*/
float AveValue = 0;
int AveNum = 16;
const int RunningAveNum = 16;
float RunningAveBuffer[RunningAveNum];
int NextRunningAve;
float volt;

void setup() {
  /*SERIAL MONITOR*/
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // Wait for serial port to connect
  //while (!Serial) {
    //; // Wait for serial port to connect. Needed for native USB port only
  //}

  /*INPUT AND OUTPUT PIN INITILISATION*/
  pinMode(LED_pin, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(button_pin, INPUT);

  for (int i = 0; i < 6; i++){
    pinMode(piezo_pin[i], INPUT);
    pinMode(pLED_pin[i], OUTPUT);
  }
  
  /*BLE CONNECTION SETUP*/
  //Begin initilisation of BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while(1);
  }
      
  /*BLE SET UP*/
  BLE.setDeviceName("Arduino Nano BLE"); //Set the device name
  BLE.setLocalName("Arduino Nano BLE"); //Set advertised local name
  BLE.setAdvertisedService(monitoringService); //Set the advertised service as monitoringService
  monitoringService.addCharacteristic(fetalKickChar); //Add pizeoelectric sensor charactersitic to monitorService
  monitoringService.addCharacteristic(rawPiezoChar); 
  monitoringService.addCharacteristic(fetalECGChar); //Add ECG electrode charactersitic to monitorService
  BLE.addService(monitoringService); //Add "monitorService" to set of services the BLE device provides
  BLE.setAdvertisingInterval(200); //Set an advertising interval
  //Intervals of 0.625ms

  //Assign event handlers for connection and disconnection to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  //Assign event handler for characteristic
  rawPiezoChar.setEventHandler(BLERead, updateRawPiezoWritten);
  //rawPiezoChar.setValue(0);
}

void loop() {
  BLE.poll(); 
  
  // Read the state of button
  button_value = digitalRead(button_pin); 
  if (button_value == HIGH) {    // assumes btn is LOW when pressed
    lastBtnPressMillis = millis();   // btn not pressed so reset clock
  }

  // Check button has been pressed for longer than interval
  if (millis() - lastBtnPressMillis >= interval) {

    if (!bluetoothState){
      bluetoothState = ~bluetoothState;
      
      // Turn LED on to indicate bluetooth on 
      digitalWrite(LED_pin, HIGH);
      delay(1000); // Delay between LED turning on for stability
  
      // Set advertising state 
      BLE.advertise(); //Advertise the BLE device
      
      Serial.println("Bluetooth device active, waiting for connections...");
    }else{
      bluetoothState = ~bluetoothState;
      
      //Stop advertising
      BLE.stopAdvertise();
        
      //Turn LED off to indicate bluetooth off
      digitalWrite(LED_pin, LOW);
        
      delay(1000); // Delay between LED turning on for stability
      Serial.println("Turn bluetooth off...");
    }     
  }  

  if(!bluetoothState){
  /* PIEZO ELECTRIC SENSORS */
  long currentPiezoSensorMillis = millis();
  
  // Check piezoelectric sensors every 200m
  if (currentPiezoSensorMillis - previousPiezoSensorMillis >= 200) {
   previousPiezoSensorMillis = currentPiezoSensorMillis;

    ExponentialFilter<float> FilteredValue(50, 0);
    sensorValue[0] = analogRead(piezo_pin[0]);
    sensor0.push_back(sensorValue[0]);
    sensorValue[1] = analogRead(piezo_pin[1]);
    sensor1.push_back(sensorValue[1]);
    sensorValue[2] = analogRead(piezo_pin[2]);
    sensor2.push_back(sensorValue[2]);
    sensorValue[3] = analogRead(piezo_pin[3]);
    sensor3.push_back(sensorValue[3]);
    sensorValue[4] = analogRead(piezo_pin[4]);
    sensor4.push_back(sensorValue[4]);
    sensorValue[5] = analogRead(piezo_pin[5]);
    sensor5.push_back(sensorValue[5]);
 
    //FilteredValue.Filter(volt);
    //float SmoothValue = FilteredValue.Current();;
  
    Serial.print(sensorValue[0]); //Print sensor value to serial monitor for debugging 
    Serial.print("\t");
    Serial.print(sensorValue[1]);
    Serial.print("\t");
    Serial.print(sensorValue[2]);
    Serial.print("\t");
    Serial.print(sensorValue[3]);
    Serial.print("\t");
    Serial.print(sensorValue[4]);
    Serial.print("\t");
    Serial.println(sensorValue[5]);
    
    if (sensorValue[0] > 400){
      triggerLED(0);
      fetalkick_count++;
    }
    if (sensorValue[1] > 400){
      triggerLED(1);
      fetalkick_count++;
    }
    if (sensorValue[2] > 400){
      triggerLED(2);  
      fetalkick_count++;  
    }
    if (sensorValue[3] > 400){
      triggerLED(3);  
      fetalkick_count++;   
    }
    if (sensorValue[4] > 400){
      triggerLED(4);  
      fetalkick_count++;
    }
    if (sensorValue[5] > 400){
      triggerLED(5); 
      fetalkick_count++;
    }
    
    delay(500);
  }
 }
}

void blePeripheralConnectHandler(BLEDevice central) {
  Serial.println("Connected event, central: ");
  Serial.println(central.address());

  //Stop advertising
  BLE.stopAdvertise();
  
  //Turn on on-board LED to idicate connection 
  digitalWrite(ONBOARD_LED, HIGH);
       
  //Send data to central
  fetalKickChar.writeValue((byte)fetalkick_count);

  Serial.print("Fetal Kick Count Send: ");
  Serial.println(fetalkick_count);  
  
  sendStr = "*";
 
  //Send data to central 
  sendStr.toCharArray(buff,20);
  rawPiezoChar.writeValue(buff,20);

  Serial.print("Start Sequence Sent: ");
  Serial.println(sendStr);  
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());

  //Reset fetal kick count
  fetalkick_count = 0;

  //Reset bluetooth state 
  bluetoothState = ~bluetoothState;
  
  //Turn off on-board LED to indicate disconnection
  digitalWrite(ONBOARD_LED, LOW);
  
  BLE.disconnect();

  //Turn LED off to indicate bluetooth off
  digitalWrite(LED_pin, LOW);
        
  delay(1000); // Delay between LED turning on for stability
  
  // Prints title with ending line break
  Serial.println("BLE end");    
}

void triggerLED(int num){
   // Enable the LEDs
   digitalWrite(pLED_pin[num], HIGH);
   
   // Delay the program for X milliseconds, to see the LED
   delay(500);
      
   // Turn LEDs off 
   digitalWrite(pLED_pin[num], LOW);
}

void updateRawPiezoWritten(BLEDevice central, BLECharacteristic characteristic){

  vector<int> sensorSend;
  
  for (int num = 0; num < 6; num++){
    switch (num) {
      case 0:
        sensorSend = sensor0;
        break;
      case 1:
        sensorSend = sensor1;
        break;
      case 2:
        sensorSend = sensor2;
        break;
      case 3:
        sensorSend = sensor3;
        break;
      case 4:
        sensorSend = sensor4;
        break;
      case 5:
        sensorSend = sensor5;
        break;
      default:
        break;
   }

   //Indicate start of sequence
   sendStr = "*";

   auto i = sensorSend.begin();

   int startSeq = 1;

   while(startSeq == 1){
    //Read i value
    String charValue = String(*i);
        
    //check if larger than 20 bytes; if so then write to rawPiezoChar
    if(sendStr.length()+charValue.length()> 20 || i == sensorSend.end()){
      //Send data to central 
      sendStr.toCharArray(buff,20);
      rawPiezoChar.writeValue(buff,20);
  
      //Print to Serial Monitor for Debugging 
      Serial.print("Sensor Send: ");
      Serial.print(sendStr); 

      //Reset sendStr
      sendStr = "";

     if(i == sensorSend.end()){
       //Send "!" to indicate sequence end 
       sendStr = "!";
       Serial.print("End: ");
       Serial.print(sendStr);
       
       //Send data to central 
       sendStr.toCharArray(buff,20);
       rawPiezoChar.writeValue(buff,20);
  
       //Reset sendStr
       sendStr = "";
       startSeq = 0;
     }
      
    }else{
      //Add value to string
      sendStr += charValue;
      sendStr += ",";
      i++;
    } 
   } 
  } 
}

/*
void averageFilter() {
  for (int i = 0; i < AveNum; ++i){

      // Read value from piezoelectric sensors
      sensorValue = analogRead(piezo_pin);
      AveValue += sensorValue; 
      delay(1);
    }

    AveValue /= AveNum;

    Serial.print(sensorValue); //Print sensor value to serial monitor for debugging 
    Serial.print("\t");
    Serial.println(AveValue); //Print ave sensor value 
}
*/

/*
void runningAveFilter() {
  sensorValue = analogRead(piezo_pin);
  RunningAveBuffer[NextRunningAve++] = sensorValue;
  
  if (NextRunningAve >= RunningAveNum){
    NextRunningAve = 0; 
  }
  float RunningAveValue = 0;
  
  for(int i=0; i< RunningAveNum; ++i){
    RunningAveValue += RunningAveBuffer[i];
  }
  
  RunningAveValue /= RunningAveNum;

  Serial.print(sensorValue); //Print sensor value to serial monitor for debugging 
  Serial.print("\t");
  Serial.println(RunningAveValue); //Print ave sensor value 
}
*/

/*
void expFilter(){
  ExponentialFilter<float> FilteredValue(50, 0);
  float sensorValue = analogRead(piezo_pin);
  FilteredValue.Filter(sensorValue);
  float SmoothValue = FilteredValue.Current();
}
 */

  /* ECG ELECTRODES */
  /*
  //Declare variable to read incoming serial data 
  int incomingByte;
  
  //Read serial when avaliable
  while (Serial.available() > 0){

    //Read oldest byte in the serial buffer 
    incomingByte = Serial.read();
    //Check for a newline
    if (Serial.read() == '\n'){
      
    }

    Serial.println(incomingByte,DEC);
    
  }
  */
