#include <ArduinoBLE.h> // Include the ArduinoBLE library
#include <SPI.h> //

int inputSensor = A0;    // select the input pin for the potentiometer
int outputLED = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

//Create a BLE service 
BLEService monitorService("1101"); //Temporary UUID

//Create the BLECharacteristics
BLECharacteristic pizeoSensor("2101", BLERead, 2); //Temprary UUID
BLECharacteristic ecgElectrode("3101", BLERead, 2); //Temporary UUID

void setup() {

  //Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  //Wait for serial port to connect
  while (!Serial){
    ; 
  }

  //Declare the pinModes of used pins 
  //Declare the ledPin as an OUTPUT
  pinMode(outputLED, OUTPUT);
  
  //Begin initilisation of BLE
  if (!BLE.begin()){
    Serial.println("Starting BLE failed!");
    while(1);
  }

  //BLE SET UP
  BLE.setDeviceName("Arduino Nano BLE"); //Set the device name
  BLE.setLocalName("Fetal Monitoring"); //Set advertised local name
  BLE.setAdvertisedService(monitorService); //Set the advertised service 
  monitorService.addCharacteristic(pizeoSensor); //Add pizeoelectric sensor charactersitic to monitorService
  monitorService.addCharacteristic(ecgElectrode); //Add ECG electrode charactersitic to monitorService
  BLE.addService(monitorService); //Add "monitorService" to set of services the BLE device provides
  BLE.setAdvertisingInterval(320); //Set an advertising interval
  //Advertising interval is currently TEMP: 200*0.625ms
  BLE.advertise(); //Advertise the BLE device 

  //Print to serial that Bluetooth device is active and waiting for connections
  Serial.println("Bluetooth device active, waiting for connections...");
  }
  
}

void loop() {

  /*BLE CONNECTION*/
  BLEDevice central = BLE.central(); //Listen for BLE peripherals to connect 
  BLE.setConnectable(true); //Make the device connectable when advertising 

  //Check the BLE device is connected to central
  if(central){
    BLE.stopAdvertise(); //Stop advertising
    
    //Print to serial that BLE device is connected to central
    Serial.print("Connected to central: ");
    Serial.println(central.address()); 
    //Turn on on-board LED to idicate connection 
    
  
    while(central.connected()){

      //Send data to central

      updateSensors();
    }

    //Central is disconnected
      //Tun off on-board LED to indicate disconnection
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    BLE.end(); //Turn off BLE
  }  
  
  /* PIEZO ELECTRIC SENSORS */
  //Read value from sensor
  sensorValue = analogRead(inputSensor); 

  //When sensor reaches a threashold
  if (sensorValue > 50){

    digitalWrite(ledPin, HIGH);
    
    //Delay the program for X milliseconds, to see the LED
    delay(sensorValue);

    //Turn LEDs (outputLED) off 
    digitalWrite(ledPin, LOW);

  }

  /* ECG ELECTRODES */
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

}

void bleConnection(){
  
}

void updateSensors() {
}
