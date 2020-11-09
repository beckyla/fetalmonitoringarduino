  /*INCLUDE LIBRARIES*/
#include <ArduinoBLE.h> //Include the ArduinoBLE library
#include <SPI.h> //Include the SPI library 
#include "ADAS1000.h" //include ADAS100 library 
#include "Filter.h"
#include <vector>
using std::vector;
#include <string>
using namespace std;
#include "TimeLib.h"

/*FETAL KICK VARIABLES*/
int pLED_pin[] = {4, 2, 3, 5, 6, 7};
int piezo_pin[] = {A5, A1, A2, A3, A4, A6};
float sensorValue[] = {0, 0, 0, 0, 0, 0};
float filteredSensorValue[] = {0, 0, 0, 0, 0, 0};
int fetalkick_count = 0;
long previousPiezoSensorMillis;
vector<int> sensor0;
vector<int> sensor1;
vector<int> sensor2;
vector<int> sensor3;
vector<int> sensor4;
vector<int> sensor5;
int piezoSend = 0;
int currentToggle[] = {0, 0, 0, 0, 0, 0};
int prevToggle[] = {0, 0, 0, 0, 0, 0};
char buff[20];
String sendStr = "";
unsigned long lastMillis;
vector<String> kickInfo;
String startTime[] = {"", "", "", "", "", ""};
String endTime[] = {"", "", "", "", "", ""};
String iString = "";
String recordStart;
String recordEnd;
String date;

/*BLUETOOTH VARIABLES*/
int LED_pin = A0;
int ONBOARD_LED = 13;
int button_pin = 10;
int button_value = 0;
int lastBtnPressMillis = 0;
int interval = 3000; //3 seconds
int bluetoothState = 0;

/*BLE CONNECTION VARIABLES*/
// Create a BLE service
BLEService monitoringService("19B10000-E8F2-537E-4F6C-D104768A1214");
// Create the BLECharacteristics
BLECharacteristic fetalKickChar("19B10000-E8F2-537E-4F6C-D104768A1216", BLEWriteWithoutResponse | BLENotify, 20);
//BLECharacteristic fetalECGChar("19B10000-E8F2-537E-4F6C-D104768A1218", BLERead, 2); //Temporary UUID
BLECharacteristic rawPiezoChar("19B10000-E8F2-537E-4F6C-D104768A1224", BLEWriteWithoutResponse | BLENotify, 20);
BLECharacteristic timeSyncChar("19B10000-E8F2-537E-4F6C-D104768A1226", BLEWriteWithoutResponse, 14);

/*PIEZO FILTERING VARIABLES*/
float AveValue[] = {0, 0, 0, 0, 0, 0};
int AveNum = 25;
const int RunningAveNum = 16;
float RunningAveBuffer0[RunningAveNum];
float RunningAveBuffer1[RunningAveNum];
float RunningAveBuffer2[RunningAveNum];
float RunningAveBuffer3[RunningAveNum];
float RunningAveBuffer4[RunningAveNum];
float RunningAveBuffer5[RunningAveNum];
int NextRunningAve0;
int NextRunningAve1;
int NextRunningAve2;
int NextRunningAve3;
int NextRunningAve4;
int NextRunningAve5;
float RunningAveValue[] = {0, 0, 0, 0, 0, 0};
float filter0, filter2, filter3, filter1;

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

  for (int i = 0; i < 6; i++) {
    pinMode(piezo_pin[i], INPUT);
    pinMode(pLED_pin[i], OUTPUT);
  }

  /*BLE CONNECTION SETUP*/
  //Begin initilisation of BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  /*BLE SET UP*/
  BLE.setDeviceName("Arduino Nano BLE"); //Set the device name
  BLE.setLocalName("Arduino Nano BLE"); //Set advertised local name
  BLE.setAdvertisedService(monitoringService); //Set the advertised service as monitoringService
  monitoringService.addCharacteristic(fetalKickChar); //Add pizeoelectric sensor charactersitic to monitorService
  monitoringService.addCharacteristic(rawPiezoChar);
  //monitoringService.addCharacteristic(fetalECGChar); //Add ECG electrode charactersitic to monitorService
  monitoringService.addCharacteristic(timeSyncChar);
  BLE.addService(monitoringService); //Add "monitorService" to set of services the BLE device provides
  BLE.setAdvertisingInterval(200); //Set an advertising interval
  //Intervals of 0.625ms

  //Assign event handlers for connection and disconnection to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  //Assign event handler for characteristic
  rawPiezoChar.setEventHandler(BLEWritten, updateRawPiezoWritten);
  timeSyncChar.setEventHandler(BLEWritten, syncTime);
  fetalKickChar.setEventHandler(BLEWritten, updateKickInfoWritten);

  if (timeStatus() == timeNotSet) {
    //setTime(15,30,33,24,7,2020);
    recordStart = String(millis() / 1000);
  } else {
    time_t sRecordT = now();
    date = String(day(sRecordT)) + "/" + String(month(sRecordT)) + "/" + String(year(sRecordT));
    recordStart = String(hour(sRecordT)) + ":" + String(minute(sRecordT)) + ":" + String(second(sRecordT));
  }
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

    if (!bluetoothState) {
      bluetoothState = ~bluetoothState;

      // Turn LED on to indicate bluetooth on
      digitalWrite(LED_pin, HIGH);
      delay(1000); // Delay between LED turning on for stability

      // Set advertising state
      BLE.advertise(); //Advertise the BLE device

      if (timeStatus() == timeNotSet) {
        recordEnd = String(millis() / 1000);
      } else {
        time_t eRecordT = now();
        recordEnd = String(hour(eRecordT)) + ":" + String(minute(eRecordT)) + ":" + String(second(eRecordT));
      }

      Serial.println("Bluetooth device active, waiting for connections...");
    } else {
      bluetoothState = ~bluetoothState;

      //Stop advertising
      BLE.stopAdvertise();

      //Turn LED off to indicate bluetooth off
      digitalWrite(LED_pin, LOW);

      delay(1000); // Delay between LED turning on for stability
      Serial.println("Turn bluetooth off...");
    }
  }

  if (!bluetoothState) {

    /* PIEZO ELECTRIC SENSORS */
    long currentPiezoSensorMillis = millis();

    // Check piezoelectric sensors every 50m
    if (currentPiezoSensorMillis - previousPiezoSensorMillis >= 50) {
      previousPiezoSensorMillis = currentPiezoSensorMillis;


      /*EXPONENTIAL FILTER*/
      /*
      ExponentialFilter<float> Filter0 (50, 0);
      ExponentialFilter<float> Filter1 (50, 0);
      ExponentialFilter<float> Filter2 (50, 0);
      ExponentialFilter<float> Filter3 (50, 0);
      ExponentialFilter<float> Filter4 (50, 0);
      ExponentialFilter<float> Filter5 (50, 0);

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

      Filter0.Filter(sensorValue[0]);
      Filter1.Filter(sensorValue[1]);
      Filter2.Filter(sensorValue[2]);
      Filter3.Filter(sensorValue[3]);
      Filter4.Filter(sensorValue[4]);
      Filter5.Filter(sensorValue[5]);

      filteredSensorValue[0] = Filter0.Current();
      filteredSensorValue[1] = Filter1.Current();
      filteredSensorValue[2] = Filter2.Current();
      filteredSensorValue[3] = Filter3.Current();
      filteredSensorValue[4] = Filter4.Current();
      filteredSensorValue[5] = Filter5.Current();

      //Print sensor value to serial monitor for debugging
      Serial.print(sensorValue[0]);
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

      //Print filtered sensor values to serial monitor for debugging
      Serial.print(filteredSensorValue[0]);
      Serial.print("\t");
      Serial.print(filteredSensorValue[1]);
      Serial.print("\t");
      Serial.print(filteredSensorValue[2]);
      Serial.print("\t");
      Serial.println(filteredSensorValue[3]);
      Serial.print("\t");
      Serial.print(filteredSensorValue[4]);
      Serial.print("\t");
      Serial.println(filteredSensorValue[5]);
      */

      /*//AVERAGE FILTER
      for (int i = 0; i < AveNum; ++i) {
        sensorValue[0] = analogRead(piezo_pin[0]);
        sensorValue[1] = analogRead(piezo_pin[1]);
        sensorValue[2] = analogRead(piezo_pin[2]);
        sensorValue[3] = analogRead(piezo_pin[3]);
        sensorValue[4] = analogRead(piezo_pin[4]);
        sensorValue[5] = analogRead(piezo_pin[5]);
        
        // Read value from piezoelectric sensors
        AveValue[0] += sensorValue[0];
        AveValue[1] += sensorValue[1];
        AveValue[2] += sensorValue[2];
        AveValue[3] += sensorValue[3];
        AveValue[4] += sensorValue[4];
        AveValue[5] += sensorValue[5];
        
        delay(1);
      }
  
      AveValue[0] /= AveNum;
      AveValue[1] /= AveNum;
      AveValue[2] /= AveNum;
      AveValue[3] /= AveNum;
      AveValue[4] /= AveNum;
      AveValue[5] /= AveNum;
      */

      /*//Print sensor value to serial monitor for debugging
      Serial.print(sensorValue[0]);
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
      */
      /*//Print filtered sensor values to serial monitor for debugging
      Serial.print(AveValue[0]);
      Serial.print("\t");
      Serial.print(AveValue[1]);
      Serial.print("\t");
      Serial.print(AveValue[2]);
      Serial.print("\t");
      Serial.print(AveValue[3]);
      Serial.print("\t");
      Serial.print(AveValue[4]);
      Serial.print("\t");
      Serial.println(AveValue[5]);

     */
      /*RUNNING AVERAGE FILTER*/
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

      //Print sensor value to serial monitor for debugging
      /*Serial.print(sensorValue[0]);
      Serial.print("\t");
      Serial.print(sensorValue[1]);
      Serial.print("\t");
      Serial.print(sensorValue[2]);
      Serial.print("\t");
      Serial.print(sensorValue[3]);
      Serial.print("\t");
      Serial.print(sensorValue[4]);
      Serial.print("\t");
      */
      Serial.print(sensorValue[5]);
      Serial.print("\t");
      
      RunningAveBuffer0[NextRunningAve0++] = sensorValue[0];
      RunningAveBuffer1[NextRunningAve1++] = sensorValue[1];
      RunningAveBuffer2[NextRunningAve2++] = sensorValue[2];
      RunningAveBuffer3[NextRunningAve3++] = sensorValue[3];
      RunningAveBuffer4[NextRunningAve4++] = sensorValue[4];
      RunningAveBuffer5[NextRunningAve5++] = sensorValue[5];

      if (NextRunningAve0 >= RunningAveNum){
        NextRunningAve0 = 0;
      }
      if (NextRunningAve1 >= RunningAveNum){
        NextRunningAve1 = 0;
      }
      if (NextRunningAve2 >= RunningAveNum){
        NextRunningAve2 = 0;
      }
      if (NextRunningAve3 >= RunningAveNum){
        NextRunningAve3 = 0;
      }
      if (NextRunningAve4 >= RunningAveNum){
        NextRunningAve4 = 0;
      }
      if (NextRunningAve5 >= RunningAveNum){
        NextRunningAve5 = 0;
      }

      RunningAveValue[0] = 0;
      RunningAveValue[1] = 0;
      RunningAveValue[2] = 0;
      RunningAveValue[3] = 0;
      RunningAveValue[4] = 0;
      RunningAveValue[5] = 0;

      for(int i=0; i < RunningAveNum; ++i){
        RunningAveValue[0] += RunningAveBuffer0[i];
        RunningAveValue[1] += RunningAveBuffer1[i];
        RunningAveValue[2] += RunningAveBuffer2[i];
        RunningAveValue[3] += RunningAveBuffer3[i];
        RunningAveValue[4] += RunningAveBuffer4[i];
        RunningAveValue[5] += RunningAveBuffer5[i];
      }

      RunningAveValue[0] /= RunningAveNum;
      RunningAveValue[1] /= RunningAveNum;
      RunningAveValue[2] /= RunningAveNum;
      RunningAveValue[3] /= RunningAveNum;
      RunningAveValue[4] /= RunningAveNum;
      RunningAveValue[5] /= RunningAveNum;  

      /*Serial.print(RunningAveValue[0]);
      Serial.print("\t");
      Serial.print(RunningAveValue[1]);
      Serial.print("\t");
      Serial.print(RunningAveValue[2]);
      Serial.print("\t");
      Serial.print(RunningAveValue[3]);
      Serial.print("\t");
      Serial.print(RunningAveValue[4]);
      Serial.print("\t");
      */
      Serial.println(RunningAveValue[5]);
       
      //Reset toggle
      for (int i = 0; i < 6; i++) {
        currentToggle[i] = 0;
      }

      //Run checkSensor function
      checkSensor();

      //Run checkToggle function
      checkToggle();

      //Set prevToggle as currentToggle
      for (int j = 0; j < 6; j++) {
        prevToggle[j] = currentToggle[j];
      }

      //Prints information stored about kicks to serial monitor for debugging
      //for (auto j = kickInfo.begin();j != kickInfo.end();++j){
      //  Serial.println(*j);
      //}
      //delay(500);
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

  //Prints title with ending line break
  Serial.println("BLE end");

  //Record start time for start of sensor recording
  if (timeStatus() == timeNotSet) {
    recordStart = String(millis() / 1000);
  } else {
    time_t sRecordT = now();\
    date = String(day(sRecordT)) + "/" + String(month(sRecordT)) + "/" + String(year(sRecordT));
    recordStart = String(hour(sRecordT)) + ":" + String(minute(sRecordT)) + ":" + String(second(sRecordT));
  }
}

void checkToggle() {
  if (prevToggle[0] & !currentToggle[0]) {

    //time_t endT = now();
    //Run createTimeString function to create end time string
    createTimeString(0, now(), 1);

    // Call storeInfo function
    storeInfo(0);

    // Turn LEDs off
    digitalWrite(pLED_pin[0], LOW);
  }

  if (prevToggle[1] & !currentToggle[1]) {
    //Run createTimeString function to create end time string
    createTimeString(1, now(), 1);

    // Call storeInfo function
    storeInfo(1);

    // Turn LEDs off
    digitalWrite(pLED_pin[1], LOW);
  }

  if (prevToggle[2] & !currentToggle[2]) {
    //Run createTimeString function to create end time string
    createTimeString(2, now(), 1);

    // Call storeInfo function
    storeInfo(2);

    // Turn LEDs off
    digitalWrite(pLED_pin[2], LOW);
  }

  if (prevToggle[3] & !currentToggle[3]) {
    //Run createTimeString function to create end time string
    createTimeString(3, now(), 1);

    // Call storeInfo function
    storeInfo(3);

    // Turn LEDs off
    digitalWrite(pLED_pin[3], LOW);
  }

  if (prevToggle[4] & !currentToggle[4]) {
    //Run createTimeString function to create end time string
    createTimeString(4, now(), 1);

    // Call storeInfo function
    storeInfo(4);

    // Turn LEDs off
    digitalWrite(pLED_pin[4], LOW);
  }

  if (prevToggle[5] & !currentToggle[5]) {
    //Run createTimeString function to create end time string
    createTimeString(5, now(), 1);

    // Call storeInfo function
    storeInfo(5);

    // Turn LEDs off
    digitalWrite(pLED_pin[5], LOW);
  }
}

void checkSensor() {
  int threshold0 = 270;
  int threshold1 = 260;
  int threshold2 = 289;
  int threshold3 = 259;
  int threshold4 = 337;
  int threshold5 = 319;
  
  if (RunningAveValue[0] > threshold0) {
    if (!prevToggle[0] & !currentToggle[0]) {
      //Turn on the LED
      digitalWrite(pLED_pin[0], HIGH);

      //time_t startT = now();
      //Run createTimeString function to create start time string
      createTimeString(0, now(), 0);

      //Increment fetalkick_count
      fetalkick_count++;

    }
    currentToggle[0] = 1;
  }

  if (RunningAveValue[1] > threshold1) {
    if (!prevToggle[1] & !currentToggle[1]) {
      //Turn on the LED
      digitalWrite(pLED_pin[1], HIGH);

      //Run createTimeString function to create start time string
      createTimeString(1, now(), 0);

      //Increment fetalkick_count
      fetalkick_count++;
    }
    currentToggle[1] = 1;
  }

  if (RunningAveValue[2] > threshold2) {
    if (!prevToggle[2] & !currentToggle[2]) {
      //Turn on the LED
      digitalWrite(pLED_pin[2], HIGH);

      //Run createTimeString function to create start time string
      createTimeString(2, now(), 0);

      //Increment fetalkick_count
      fetalkick_count++;
    }
    currentToggle[2] = 1;
  }

  if (RunningAveValue[3] > threshold3) {
    if (!prevToggle[3] & !currentToggle[3]) {
      //Turn on the LED
      digitalWrite(pLED_pin[3], HIGH);

      //Run createTimeString function to create start time string
      createTimeString(3, now(), 0);

      //Increment fetalkick_count
      fetalkick_count++;
    }
    currentToggle[3] = 1;
  }

  if (RunningAveValue[4] > threshold4) {
    if (!prevToggle[4] & !currentToggle[4]) {
      //Turn on the LED
      digitalWrite(pLED_pin[4], HIGH);

      //Run createTimeString function to create start time string
      createTimeString(4, now(), 0);

      //Increment fetalkick_count
      fetalkick_count++;
    }
    currentToggle[4] = 1;
  }

  if (RunningAveValue[5] > threshold5) {
    if (!prevToggle[5] & !currentToggle[5]) {
      //Turn on the LED
      digitalWrite(pLED_pin[5], HIGH);

      //Run createTimeString function to create start time string
      createTimeString(5, now(), 0);

      //Increment fetalkick_count
      fetalkick_count++;
    }
    currentToggle[5] = 1;
  }
}

void storeInfo(int num) {

  //Create String of information
  String infoString = String(num) + "," + startTime[num] + "," + endTime[num];

  //String in kickInfo Vector
  kickInfo.push_back(infoString);

  //Clear startTime and endTime
  startTime[num] = "";
  endTime[num] = "";
}

void createTimeString(int num, time_t t, int choose) {

  //If choose == 0, store in startTime; else store in endTime
  if (choose == 0) {

    if (timeStatus() == timeNotSet) {
      //Record the time in seconds since the Arduino board began running the current program
      startTime[num] = String(millis() / 1000);
    } else {
      startTime[num] = String(hour(t)) + ":" + String(minute(t)) + ":" + String(second(t));
    }

  } else {
    if (timeStatus() == timeNotSet) {
      //Record the time in seconds since the Arduino board began running the current program
      endTime[num] = String(millis() / 1000);
    } else {
      endTime[num] = String(hour(t)) + ":" + String(minute(t)) + ":" + String(second(t));
    }
  }
}

void updateRawPiezoWritten(BLEDevice central, BLECharacteristic characteristic) {

  delay(1500);
  vector<int> sensorSend;
  long sendInterval = 250;

  for (int num = 0; num < 6; num++) {

    //Case statement to choose sensor
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
    sendStr += "*";

    auto i = sensorSend.begin();

    int startSeq = 1;

    while (startSeq == 1) {
      //Read i value
      String charValue = String(*i);

      //check if larger than 18 bytes; if so then write to rawPiezoChar
      if (sendStr.length() + charValue.length() > 18 || i == sensorSend.end()) {

        while (millis() - lastMillis < sendInterval)
        {
          // Wait for appropriate interval before sending
          //Serial.print("waiting");
        }
        
        //Send data to central
        sendStr.toCharArray(buff, 20);
        rawPiezoChar.writeValue(buff, 20);

        //Print to Serial Monitor for Debugging
        Serial.print("Sensor Send: ");
        Serial.print(sendStr);
        Serial.print("\n");

        //Reset sendStr
        sendStr = "";
        lastMillis = millis();
        
        if (i == sensorSend.end()) {
          startSeq = 0;
        }

      } else {
        //Add value to string
        sendStr += charValue;
        sendStr += ",";
        i++;
      }
    }
  }

  //Print to Serial Monitor for Debugging
  Serial.print("Sensor Send: ");
  Serial.print(sendStr);
  Serial.print("\n");

  //Send data to central
  sendStr.toCharArray(buff, 20);
  rawPiezoChar.writeValue(buff, 20);

  delay(800);
  sendStr = "!";
  Serial.print("End: ");
  Serial.print(sendStr);

  sendStr.toCharArray(buff, 20);
  rawPiezoChar.writeValue(buff, 20);

  //Reset sendStr
  sendStr = "";
}

void syncTime(BLEDevice central, BLECharacteristic characteristic) {

  char buff[14];

  //Extract value sent by central
  int bytes = timeSyncChar.readValue(buff, 14);
  String strTime = (char*)buff;

  Serial.print("Central wrote to syncTime characteristic:");
  Serial.println(strTime);

  Serial.println("Bytes read: ");
  Serial.println(bytes);

  int mHour = strTime.substring(0, 2).toInt();
  int mMinute = strTime.substring(2, 4).toInt();
  int mSecond = strTime.substring(4, 6).toInt();
  int iDay = strTime.substring(6, 8).toInt();
  int iMonth = strTime.substring(8, 10).toInt();
  int iYear = strTime.substring(10).toInt();

  Serial.print("Hour: ");
  Serial.println(mHour);
  Serial.print("Minute: ");
  Serial.println(mMinute);
  Serial.print("Second: ");
  Serial.println(mSecond);
  Serial.print("Day: ");
  Serial.println(iDay);
  Serial.print("Month: ");
  Serial.println(iMonth);
  Serial.print("Year: ");
  Serial.println(iYear);

  //Add 4 seconds to account for delay time
  setTime(mHour, mMinute, mSecond + 4, iDay, iMonth, iYear);
}

void updateKickInfoWritten(BLEDevice central, BLECharacteristic characteristic) {

  delay(1500);

  //Serial.print("Fetal Kick Count Send: ");
  Serial.println(fetalkick_count);

  char cbuff[20];
  long sendInterval = 250;

  //Indicate start of sequence
  if (timeStatus() == timeNotSet) {
    iString = "*" + String(fetalkick_count) + "," + "0";

  } else {
    iString = "*" + String(fetalkick_count) + "," + date;
  }

  Serial.print("Sensor Send: ");
  Serial.println(iString);

  //Send data to central
  iString.toCharArray(cbuff, 20);
  fetalKickChar.writeValue(cbuff, 20);

  //Delay for 1 second
  delay(1000);

  //Reset sendStr
  iString = "";

  for (auto i = kickInfo.begin(); i != kickInfo.end(); ++i) {

    while (millis() - lastMillis < sendInterval)
    {
      // Wait for appropriate interval before sending
      //Serial.print("waiting");
    }

    iString = iString + *i;

    //Send data to central
    iString.toCharArray(cbuff, 20);
    fetalKickChar.writeValue(cbuff, 20);

    //Print to Serial Monitor for Debugging
    Serial.print("Sensor Send: ");
    Serial.println(iString);

    //Reset sendStr
    iString = "";
    lastMillis = millis();
  }

  //Indicate end of sequence
  iString = "!," + recordStart + "," + recordEnd;

  Serial.print("Sensor Send: ");
  Serial.println(iString);

  //Send data to central
  iString.toCharArray(cbuff, 20);
  fetalKickChar.writeValue(cbuff, 20);

  //Reset sendStr
  iString = "";
}

/*void expFilter(){
  ExponentialFilter<float> FilteredValue(50, 0);
  float sensorValue = analogRead(piezo_pin);
  FilteredValue.Filter(sensorValue);
  float SmoothValue = FilteredValue.Current();
  }
*/



/*void averageFilter(sensorValue) {
  for (int i = 0; i < AveNum; ++i){

      // Read value from piezoelectric sensors
      AveValue += sensorValue;
      delay(1);
  }

    AveValue /= AveNum;

    //Serial.print(sensorValue); //Print sensor value to serial monitor for debugging
    //Serial.print("\t");
    Serial.println(AveValue); //Print ave sensor value
  }
*/

/*void runningAveFilter(sensorValue) {
  RunningAveBuffer[NextRunningAve++] = sensorValue;

  if (NextRunningAve >= RunningAveNum){
    NextRunningAve = 0;
  }
  float RunningAveValue = 0;

  for(int i=0; i< RunningAveNum; ++i){
    RunningAveValue += RunningAveBuffer[i];
  }

  RunningAveValue /= RunningAveNum;

  //Serial.print(sensorValue); //Print sensor value to serial monitor for debugging
  Serial.println(RunningAveValue); //Print ave sensor value
  Serial.print("\t");
  }
*/
