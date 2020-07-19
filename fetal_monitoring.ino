/*INCLUDE LIBRARIES*/
#include <ArduinoBLE.h> //Include the ArduinoBLE library
#include <SPI.h> //Include the SPI library 
//#include "ADAS1000.h" //include ADAS100 library 

/*FETAL KICK VARIABLES*/
/*
//Select the input pin for the piezoelectric sensors 
int inputPiezo[] = {A0, A1, A2, A3, A4, A5};    

//Select the output pins for the LED
int outputLED[] = {20, 21, 22, 23, 24, 25}; 

// variable to store the value coming from the pizeoelelectric sensors
int sensorValue[6] = {}; 
*/

int pLED_pin = 3;
int piezo_pin = A0;
int sensorValue  = 0;
int fetalkick_count = 0;
long previousPiezoSensorMillis;

/*BLUETOOTH VARIABLES*/
int LED_pin = 2;
int ONBOARD_LED = 13;
int button_pin = 10;
int button_value = 0;
int lastBtnPressMillis = 0;
int interval = 3000; //3 seconds 
int bluetoothState = 0;

// Declare interval time 

/*BLE CONNECTION VARIABLES*/
// Create a BLE service 
BLEService monitoringService("19B10000-E8F2-537E-4F6C-D104768A1214"); //Temporary UUID
// Create the BLECharacteristics
BLECharacteristic fetalKickChar("19B10000-E8F2-537E-4F6C-D104768A1216", BLERead, 2); //Temprary UUID
BLECharacteristic fetalECGChar("19B10000-E8F2-537E-4F6C-D104768A1218", BLERead, 2); //Temporary UUID

void setup() {
  /*SERIAL MONITOR*/
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // Wait for serial port to connect
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  /*INPUT AND OUTPUT PIN INITILISATION*/
  pinMode(LED_pin, OUTPUT);
  pinMode(pLED_pin, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(button_pin, INPUT);
  pinMode(piezo_pin, INPUT);
  
  /*
    //Declare the all the LED pins as OUTPUTec
    for (int i = 0; i < 6; i++){
      pinMode(outputLED[i], OUTPUT);
    }
  */

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
  monitoringService.addCharacteristic(fetalECGChar); //Add ECG electrode charactersitic to monitorService
  BLE.addService(monitoringService); //Add "monitorService" to set of services the BLE device provides
  BLE.setAdvertisingInterval(200); //Set an advertising interval
  //Intervals of 0.625ms

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
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
  
    // Read value from piezoelectric sensors
    sensorValue = analogRead(piezo_pin); 
    Serial.println(sensorValue); //Print sensor value to serial monitor for debugging

    // Threshold to meet for registering fetal kick
    int condition = sensorValue > 400;
  
    if (condition){
      // Increment the fetal kick count variable
      fetalkick_count++;
  
      // Enable the LEDs
      digitalWrite(pLED_pin, HIGH);
          
      // Delay the program for X milliseconds, to see the LED
      delay(500);
      
      // Turn LEDs off 
      digitalWrite(pLED_pin, LOW);
    }
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

//void updateSensors() {
  
//}
  
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


//Function to record fetal kicks from the piezoelectric sensors 
/*
 
void fetalKick(){
 */ 
  /* PIEZO ELECTRIC SENSORS */

/*
  for (int i = 0; i < 6; i++){

    //Read value from piezoelectric sensors
    sensorValue[i] = analogRead(inputPiezo[i]); 
    
    //When sensor reaches a threashold
    if (sensorValue[i] > 50){
      for (int j = 0; j < 6; j ++){
        //Enable the LEDs
        digitalWrite(outputLED[j], HIGH);
        
        //Delay the program for X milliseconds, to see the LED
        delay(sensorValue[i]);
    
        //Turn LEDs off 
        digitalWrite(outputLED[j], LOW);
      }
    }
  }
}
*/

/*
//Function to enable Bluetooth Low Energy Connection
void bleConnection(){

  //While the button is not pressed again
  while(button_value == HIGH){
    
    BLE.advertise(); //Advertise the BLE device 

    //Print to serial that Bluetooth device is active and waiting for connections
    Serial.println("Bluetooth device active, waiting for connections...");
  
    //BLE CONNECTION
    BLEDevice central = BLE.central(); //Listen for BLE peripherals to connect 
    //BLE.setConnectable(true); //Make the device connectable when advertising 
  
    //Check the BLE device is connected to central
    if(central){
  
      //Stop advertising
      BLE.stopAdvertise();
      
      //Print to serial that BLE device is connected to central
      Serial.print("Connected to central: ");
      Serial.println(central.address()); 
      
      //Turn on on-board LED to idicate connection 
    
      while(central.connected()){
  
        //Send data to central
        Serial.println("Data send");
  
        //updateSensors();
      }
  
      //DISCONNECTION OF CENTRAL
      //Central is disconnected
      //Turn off on-board LED to indicate disconnection
  
      //Print to serial that BLE device has disconnected to central
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
      
      //Turn off BLE
      BLE.end(); 
    }  

    // prints title with ending line break
    Serial.println("BLE end");
  }
}
*/
