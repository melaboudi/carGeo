//inits
  #include "LowPower.h"
  #include "PinChangeInterrupt.h"
  #define intPin 8
  volatile bool flag=false;
  bool received=false;
  // uint32_t debugWriteCounter=20000;
  #include "Adafruit_FRAM_I2C.h"
  Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
  uint16_t          framAddr = 0;
  unsigned long framWritePosition = 0;
  unsigned long framWritePositionDebug = 32000;
  int insertCounter = 0;
  String fixStatus = " ";
  String gpsTime = " ";
  String latitude = " ";
  String longitude = " ";
  String course = " ";
  String pdop = " ";
  uint32_t payLoadSize = 0;
  String used_satellites = " ";
  String viewed_satellites = " ";
  String lastUnixTime = " ";
  char Buffer[10] = {0};
  String speed = " ";
  #include "timestamp32bits.h"
  int fixState = 0;
  int gnsState = 0;
  uint16_t gnsFailCounter = 0;
  unsigned long previousMillisGps = 0;
  const long intervalGps = 200000;
  unsigned long currentMillis = 0;
  uint16_t gpsFailCounter = 0;
  bool started = true;
  bool restarted = false;
  uint16_t httpActionFail = 0;
  uint16_t FirstStartCounter = 0;
  uint16_t ReStartCounter=0;
  bool onOff = true;
  unsigned long t1 = 0; //le temps du dernier point inséré
  unsigned long t2 = 0; //le temps du dernier point capté
  uint16_t ti = 6; //le temps entre chaque insertion
  unsigned long t3 = 0; //le temps du dernier envoie
  unsigned long unixTimeInt = 0;
  uint16_t SizeRec = 49;//49
  uint16_t sizeSend=163;//163
  bool wakeUp=true;
  bool OkToSend = true;
  uint16_t maxTime = 0;
  uint16_t Size = 0;
  String batLev=" ";
  volatile uint16_t wakeUpCounter = 0;
  char newC[3]={0};
  //6  45----7points 9secondsSend  ok
  //6  41----7points 10secondsSend
  //6  39----6points 9secondsSend
  //6  50----8points 10secondsSend
  //6  36----6points 9secondsSend
  //6  34----6points 8secondsSend
  //6  29----5points 8.5secondsSent
  //6  25----4Points 8secondsSend
  int badCharCounter=0;
  // uint16_t httpTimeout=8000;  //voiture 07 06/05/2020 from 9am to 10am ¬OK
  uint16_t httpTimeout=14000;  //voiture 07
  uint64_t lastSend =0;
  uint16_t reps=0;
  char* one="1";
  char* zero="0";
  bool ping=true;
  bool httpPostCustom(char custom);
  void badCharChecker(String data);
  void IntRoutine(void);
  void httpPostMaster();
  void httpPing();
  bool httpPostFromTo(uint16_t p1, uint16_t p2);
  void getWriteFromFramFromZero(uint16_t p1, uint16_t p2);
  void decrementCounter(uint16_t value);
  bool turnOnGns();
  bool getGnsStat();
  bool getGpsData();
  void sendAtCom(char *AtCom);
  uint8_t getGsmStat();
  String batteryLevel();
  String rssiLevel();
  bool gprsOn();
  void gprsOff();
  bool getGprsState();
  void flushSim();
  void writeDataFram(char* dataFram);
  void writeDataFramDebug(char* dataFram, long p1);
  void powerUp();
  void powerDown();
  void blinkLED(int k);
  void blinkLEDFast(int k);
  void simHardReset();
  void clearMemory(uint16_t size);
  void clearMemoryDiff(uint16_t size, uint16_t size1);
  void clearMemoryDebug(uint16_t size);
  void insertMem();
  void incrementCounter();
  String complete(String s, int t);
  int getCounter();
  int getValue(uint16_t position, uint8_t largeur);
  void incrementValue(uint16_t position, uint8_t largeur);
  bool sendAtFram(long timeout, uint16_t pos1, uint16_t pos2, char* Rep, char* Error, int nbRep);
  bool fireHttpAction(long timeout, char* Commande, char* Rep, char* Error);
  void trace(unsigned long unixTime, uint8_t type);
  void clearValue();
  bool insertGpsData();
  void resetSS();
  void cfunReset();
  void hardResetSS();
  int getBatchCounter(uint16_t i);
  bool gps();
  void sendFromFram(uint16_t start,uint16_t length);
  int limitToSend =12;
  unsigned long te = 60; //le temps entre les envoies
  String previousUnixTime="";
  uint16_t iterations=8; //sleeping time = iterations X 8 Seconds
  void setup() {
    delay(100);
    fram.begin();
      pinMode(A2, OUTPUT);//VIO
      pinMode(1, OUTPUT);//SS TX
      pinMode(A0, OUTPUT);//sim Reset
      pinMode(A3, INPUT);//sim Power Status
      pinMode(0, INPUT);//SS RX
      digitalWrite(A2, HIGH);
      digitalWrite(A0, HIGH);
    powerDown();
    powerUp();
    Serial.begin(4800);
    turnOnGns();
    while (getGsmStat() != 1) {
      delay(500);
    }
    gprsOn();httpPing();gps();
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin), IntRoutine, RISING);
}

void loop() {
  if(getCounter()>600){clearMemory(30999);clearMemoryDebug(32003);resetSS();} 
  enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
  if (!digitalRead(8)) {           //if the engine is powered on
    gps();                         //get a new gps point all the time
    if((t2 - t3) >= (te-8)){t3=t2; //wait until it's 8 seconds before it's time to send
      httpPing();gps();            //Ping the server and get a new gps point
      if(!ping){
        if ((getCounter()%limitToSend)!=0) //if we have collected any new points since the last send, send them
        {
          uint16_t batchCounter=getCounter()/limitToSend;
          uint16_t startingPoint=batchCounter*limitToSend;
          if(httpPostFromTo(startingPoint,getCounter())){
          clearMemoryDiff(startingPoint*SizeRec,getCounter()*SizeRec); 
          decrementCounter(getCounter()%limitToSend);
          }
        }else{                     //if we have collected a complete batch, send it
          uint16_t batchCounter=getCounter()/limitToSend-1;
          uint16_t startingPoint=batchCounter*limitToSend;
          if(getBatchCounter(batchCounter)==1){
            if(httpPostFromTo((batchCounter)*limitToSend,((batchCounter+1)*limitToSend))){
              clearMemoryDiff(startingPoint*SizeRec,getCounter()*SizeRec); 
              decrementCounter(limitToSend);
            }
          }
        }
      }
    }
  }else {                          //if the engine is powered off
    httpTimeout=25000;
    while ((getCounter()!=0)&&(!digitalRead(8))){httpPostMaster();}
    httpTimeout=14000;
    httpPostCustom('0');
    powerDown();
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin), IntRoutine, RISING);
    Serial.flush();
    while (wakeUpCounter <= iterations) {
      LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
      if (digitalRead(8)){wakeUpCounter= iterations;}
      wakeUpCounter++;
    }
    if (wakeUpCounter != (iterations+1)) {                  //Vehicule ignition wakeup
      powerUp();turnOnGns(); gprsOn();
      wakeUpCounter = 0;  
      httpPostCustom('1');
    }else {                                                 //WD timer wakeups
      powerUp();turnOnGns();gprsOn();
      wakeUpCounter = 0;
      httpPostCustom('1');
      gps();
    }
  }
}
void httpPostMaster(){
  for (uint16_t i = 0; i<(getCounter()/limitToSend); i++){
    if(getBatchCounter(i)==1){//found=true;
      if(httpPostFromTo((i)*limitToSend,((i+1)*limitToSend))){
        writeDataFramDebug("0",(32080+i));
        clearMemoryDiff((i)*limitToSend*SizeRec,((i+1)*limitToSend*SizeRec));
      }
      // else{
      //   uint8_t j=0;while (ping&&(j<3)){httpPing();j++;}if (j==3){resetSS();}
      //   httpPostFromTo((i)*limitToSend,((i+1)*limitToSend));
      //   writeDataFramDebug("0",(32080+i));
      //   clearMemoryDiff((i)*limitToSend*SizeRec,((i+1)*limitToSend*SizeRec));
      // }
    }
  }
  if((getCounter()%limitToSend)!=0){
    uint16_t reps= (getCounter()/limitToSend);         
    if(httpPostFromTo((reps*limitToSend),getCounter())){
      clearMemoryDiff(reps*limitToSend*SizeRec,getCounter()*SizeRec); 
      clearMemoryDebug(32003);
    }
    // else{
    //   uint8_t j=0;while (ping&&(j<3)){httpPing();j++;}if (j==3){resetSS();}
    //   httpPostFromTo(reps*limitToSend,getCounter());
    //   clearMemoryDiff(reps*limitToSend*SizeRec,getCounter()*SizeRec); 
    //   clearMemoryDebug(32003);
    // }
  }else{clearMemoryDebug(32003);}
}


bool gps(){
  if (!getGpsData()) {
      if (!getGnsStat()) {if (gnsFailCounter == 2) {resetSS();} else {turnOnGns();delay(1000);gnsFailCounter++;}}
        if(restarted){if (ReStartCounter == 10) {resetSS();}else {delay(2000);ReStartCounter++;}
        }else if (started){if (FirstStartCounter == 1) {resetSS();}else{delay(60000);FirstStartCounter++;}
        }else if((!restarted)&&(!started)){if (gpsFailCounter == 10) {resetSS();}else {delay(1000);gpsFailCounter++;}}
        return false;
    }else{return true;}   
}
bool httpPostFromTo(uint16_t p1, uint16_t p2) {
  if(!ping){
    bool OkToSend = true;
    if (sendAtFram(3000, 31254, 11, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
      if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
        if (sendAtFram(5000, 31286, 67, "OK", "ERROR", 5)) { //URL GEORED
          Serial.setTimeout(10000);
          flushSim();
          Serial.print("AT+HTTPDATA=");
          delay(100);
          // uint16_t Size = ((p2-p1) * (sizeRec)) + 40 + 140;
          uint16_t Size = ((p2-p1) * (sizeSend)) + 40 + 140;
          Serial.print(Size);
          Serial.print(",");
          uint32_t maxTime = 30000;
          Serial.println(maxTime);
          Serial.findUntil("DOWNLOAD", "ERROR");
/////////////////////////////////////////////////////////////////////
          for(int i=0;i<140;i++){
            uint8_t test = fram.read8(i+31405);
            char Buffer[2] = {0};
            sprintf(Buffer, "%c", test);
            Serial.write(Buffer);
            delay(1);
          }  
/////////////////////////////////////////////////////////////////////          
          for (uint16_t i = p1; i < p2 ; i++)
          {
            sendFromFram(31041,13);           //"<Track Imei=\""                           //13
            sendFromFram(31474,15);           //imei                                       //15
            sendFromFram(31054,26);           //"\" Fc=\"WGS84\" FixPosition=\""           //26
            sendFromFram(SizeRec*(i),1);      //fix                                        //1
            sendFromFram(31080,7);            //"\" Lat=\""                                //7
            sendFromFram(1+SizeRec*(i),10);   //Latitude                                   //10
            sendFromFram(31087,7);            //"\"Lon=\""                                 //7
            sendFromFram(11+SizeRec*(i),11);  //Longitude                                  //11
            sendFromFram(31094,7);            //"\" Vit=\""                                //7
            sendFromFram(22+SizeRec*(i),6);   //Speed                                      //6
            sendFromFram(31101,7);            //"\" Sat=\""                                //7
            sendFromFram(28+SizeRec*(i),2);   //Sat                                        //2
            sendFromFram(31108,7);            //"\" Cap=\""                                //7
            sendFromFram(30+SizeRec*(i),6);   //Course                                     //6
            sendFromFram(31115,16);           //"\" BatteryLevel=\""                       //16
            sendFromFram(36+SizeRec*(i),3);   //BatLev                                     //3
            sendFromFram(31131,6);            //"\" Dh=\""                                 //6
            sendFromFram(39+SizeRec*(i),10);  //Dh                                         //10
            sendFromFram(31137,3);            //"\"/>";                                    //3  
          }
/////////////////////////////////////////////////////////////////////          
          for(int i=0;i<40;i++){
            uint8_t test = fram.read8(i+31545);
            char Buffer[2] = {0};
            sprintf(Buffer, "%c", test);
            Serial.write(Buffer);
            delay(1);
          }
/////////////////////////////////////////////////////////////////////
          Serial.findUntil("OK", "OK");
        } else OkToSend = false;
      } else OkToSend = false;
    } else OkToSend = false;
    if (OkToSend) {
      if (fireHttpAction(httpTimeout, "AT+HTTPACTION=", ",200,", "ERROR")) 
      {sendAtFram(5000, 31241, 11, "OK", "ERROR", 5);return true;} 
        else 
        {sendAtFram(5000, 31241, 11, "OK", "ERROR", 5);return false;}
    }
  }else{return false;}  
}

void httpPing() {
  bool OkToSend = true;
  if (sendAtFram(2500, 31254, 11, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
    if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
      if (sendAtFram(5000, 31286, 67, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
        Serial.setTimeout(10000);
        flushSim();
        Serial.print("AT+HTTPDATA=");
        delay(100);
        uint8_t Size = 2;
        Serial.print(Size);
        Serial.print(",");
        uint16_t maxTime = 5500;
        Serial.println(maxTime);
        Serial.findUntil("DOWNLOAD", "ERROR");
        Serial.print("vv");
        Serial.findUntil("OK", "OK");
      } else OkToSend = false;
    } else OkToSend = false;
  } else OkToSend = false;
  if (OkToSend) {
    fireHttpAction(3500, "AT+HTTPACTION=", ",200,", "ERROR");
    sendAtFram(5000, 31241, 11, "OK", "ERROR", 5); //httpterm
  }
}
bool httpPostCustom(char custom) {
  if(!ping){
    bool OkToSend = true;
    if (sendAtFram(3000, 31254, 11, "OK", "ERROR", 5)) { //"AT+HTTPINIT"
      if (sendAtFram(3000, 31267, 19, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"CID\",1"
        if (sendAtFram(5000, 31286, 67, "OK", "ERROR", 5)) { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
          Serial.setTimeout(10000);
          flushSim();
          Serial.print("AT+HTTPDATA=");
          delay(100);
          uint16_t Size = 26;
          Serial.print(Size);
          Serial.print(",");
          uint32_t maxTime = 30000;
          Serial.println(maxTime);
          Serial.findUntil("DOWNLOAD", "ERROR");
          Serial.print("[{\"S\":\"");
          sendFromFram(31474, 15);   //imei
          Serial.print(custom);
          Serial.print("\"}]");
          Serial.findUntil("OK", "OK");
        } else OkToSend = false;
      } else OkToSend = false;
    } else OkToSend = false;
    if (OkToSend) {
      if (fireHttpAction(2000, "AT+HTTPACTION=", ",200,", "ERROR")) {sendAtFram(5000, 31241, 11, "OK", "ERROR", 5); return true;} else {sendAtFram(5000, 31241, 11, "OK", "ERROR", 5); return false;}
    }else{return false;}
  }else{return false;}
}
void getWriteFromFramFromZero(uint16_t p1, uint16_t p2) {
  framWritePosition = 0;
  for (uint16_t a = p1; a < p1 + p2; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
    sprintf(Buffer, "%c", test);
    writeDataFram(Buffer);
  }
}
void IntRoutine() {
   wakeUpCounter = iterations;
  Serial.flush();
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(intPin));
}
void decrementCounter(uint16_t value) {
  int countVal = getCounter();
  countVal -= value;
  writeDataFramDebug(complete(String(countVal), 3).c_str(), 32000);
}
bool turnOnGns() {
  while (!getGnsStat()) {
    sendAtFram(3000, 31000, 12, "OK", "ERROR", 5); //"AT+CGNSPWR=1"
    delay(100);
  } return true;
}
bool getGnsStat() {
  flushSim();
  Serial.setTimeout(100);
  Serial.println("AT+CGNSPWR?");//"AT+CGNSPWR?"
  if (Serial.findUntil("1", "0")) {
    return true;
  } else return false;
}
void badCharChecker(String data){
      for (int i = 0; i < strlen(data.c_str()); i++){   
          if(!isAlphaNumeric(data.c_str()[i])&&(int(data.c_str()[i])!=45)&&(int(data.c_str()[i])!=46)){badCharCounter++;}
        }
}
bool getGpsData() {
  fixStatus = gpsTime = latitude = longitude = used_satellites = viewed_satellites = speed = " ";
  Serial.setTimeout(2000);
  flushSim();
  char gpsData[120] = {0};
  Serial.println("AT+CGNSINF");
  Serial.readBytesUntil('O', gpsData, 119);
    String gpsdatastr = String(gpsData);

    uint8_t ind1 = gpsdatastr.indexOf(',');
    //  String mode = gpsdatastr.substring(0, ind1);

    uint8_t ind2 = gpsdatastr.indexOf(',', ind1 + 1);
    fixStatus = gpsdatastr.substring(ind1 + 1, ind2);
    fixStatus = fixStatus.substring(0, 1);
    
    uint8_t ind3 = gpsdatastr.indexOf(',', ind2 + 1);
    String utctime = gpsdatastr.substring(ind2 + 1, ind3);
    timestamp32bits stamp = timestamp32bits();

    unixTimeInt = stamp.timestamp(
                    (utctime.substring(2, 4)).toInt(),
                    (utctime.substring(4, 6)).toInt(),
                    (utctime.substring(6, 8)).toInt(),
                    (utctime.substring(8, 10)).toInt(),
                    (utctime.substring(10, 12)).toInt(),
                    (utctime.substring(12, 14)).toInt());
    lastUnixTime = String(unixTimeInt);
    lastUnixTime = lastUnixTime.substring(0, 10);
    // badCharChecker(lastUnixTime);

    unsigned long gpsTimeInt = unixTimeInt - 315961182 ; //315964782 - 3600
    gpsTime = String(gpsTimeInt);
    t2 = gpsTimeInt;

    uint8_t ind4 = gpsdatastr.indexOf(',', ind3 + 1);
    latitude = gpsdatastr.substring(ind3 + 1, ind4);
    //latitude = latitude.substring(0, 9);
    while (strlen(latitude.c_str()) < 10) {
      latitude += '0';
    }

    uint8_t ind5 = gpsdatastr.indexOf(',', ind4 + 1);
    longitude = gpsdatastr.substring(ind4 + 1, ind5);
    while (strlen(longitude.c_str()) < 11) {
      longitude += '0';
    }
    uint8_t ind6 = gpsdatastr.indexOf(',', ind5 + 1);
    //  String altitude = gpsdatastr.substring(ind5 + 1, ind6);

    uint8_t ind7 = gpsdatastr.indexOf(',', ind6 + 1);
    speed = gpsdatastr.substring(ind6 + 1, ind7);
    speed = speed.substring(0, 6);
    while (strlen(speed.c_str()) < 6) {
      speed = '0' + speed;
    }

    uint8_t ind8 = gpsdatastr.indexOf(',', ind7 + 1);
    course = gpsdatastr.substring(ind7 + 1, ind8);
    course = course.substring(0, 6);
    while (strlen(course.c_str()) < 6) {
      course = '0' + course;
    }

    uint8_t ind9 = gpsdatastr.indexOf(',', ind8 + 1);
    //  String fixmode = gpsdatastr.substring(ind8 + 1, ind9);
    uint8_t ind10 = gpsdatastr.indexOf(',', ind9 + 1);
    //  String reserved1 = gpsdatastr.substring(ind9 + 1, ind10);
    uint8_t ind11 = gpsdatastr.indexOf(',', ind10 + 1);
    //  String HDOP = gpsdatastr.substring(ind10 + 1, ind11);

    uint8_t ind12 = gpsdatastr.indexOf(',', ind11 + 1);
    pdop = gpsdatastr.substring(ind11 + 1, ind12);
    pdop = pdop.substring(0, 4);
    while (strlen(pdop.c_str()) < 4) {
      pdop = '0' + pdop;
    }

    uint8_t ind13 = gpsdatastr.indexOf(',', ind12 + 1);
    //  String VDOP = gpsdatastr.substring(ind12 + 1, ind13);
    uint8_t ind14 = gpsdatastr.indexOf(',', ind13 + 1);
    //  String reserved2 = gpsdatastr.substring(ind13 + 1, ind14);
    uint8_t ind15 = gpsdatastr.indexOf(',', ind14 + 1);
    String viewed_satellites = gpsdatastr.substring(ind14 + 1, ind15);
    while (strlen(viewed_satellites.c_str()) < 2) {
      viewed_satellites = '0' + viewed_satellites;
    }
    uint8_t ind16 = gpsdatastr.indexOf(',', ind15 + 1);

    used_satellites = gpsdatastr.substring(ind15 + 1, ind16);
    used_satellites = used_satellites.substring(0, 2);
    while (strlen(used_satellites.c_str()) < 2) {
      used_satellites = '0' + used_satellites;
    }

    //  uint8_t ind17 = gpsdatastr.indexOf(',', ind16 + 1);
    ////  String reserved3 = gpsdatastr.substring(ind16 + 1, ind17);
    //  uint8_t ind18 = gpsdatastr.indexOf(',', ind17 + 1);
    ////  String N0max = gpsdatastr.substring(ind17 + 1, ind18);
    //  uint8_t ind19 = gpsdatastr.indexOf(',', ind18 + 1);
    ////  String HPA = gpsdatastr.substring(ind18 + 1, ind19);
    //  uint8_t ind20 = gpsdatastr.indexOf(',', ind19 + 1);
    ////  String VPA = gpsdatastr.substring(ind19);
    //////////////////////////////////////////////////////////////////
    // badCharChecker(imei);
    // imeiChecker(imei);
    badCharChecker(fixStatus);
    badCharChecker(latitude);
    badCharChecker(longitude);
    badCharChecker(speed);
    badCharChecker(used_satellites);
    badCharChecker(course);
    badCharChecker(batteryLevel());
    badCharChecker(lastUnixTime);
 
    if (onOff) {
      onOff = false;}
    if ((fixStatus.toInt() == 1) && (latitude.toInt() != 0) && (longitude.toInt() != 0)&&(badCharCounter==0)&&(lastUnixTime!=previousUnixTime)) {
      previousUnixTime=lastUnixTime;
      started = false;
      restarted=false;
      insertMem();
      return true;
    } else {return false;badCharCounter=0;}  
}
void sendAtCom(char *AtCom) {
  flushSim();
  Serial.println(AtCom);
  String tempGSM = Serial.readString();
}
uint8_t getGsmStat() {
  flushSim();
  Serial.println("AT+CREG?");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(',');
  String gsmStat = tempGSM.substring(ind1 + 1, ind1 + 2);
  return gsmStat.toInt();
}
String batteryLevel() {
  flushSim();
  Serial.println("AT+CBC");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(',');
  int ind2 = tempGSM.indexOf(',', ind1 + 1);
  //String chargeState = tempGSM.substring( 1, ind1 + 1);
  String batteryLevel = tempGSM.substring(ind1 + 1, ind2);

  uint16_t vbat = batteryLevel.toInt();

  if (vbat >= 100) {
    sprintf(batteryLevel.c_str(), "%d", vbat);
  }
  else if ((vbat < 100) && (vbat > 9)) {
    sprintf(batteryLevel.c_str(), "0%d", vbat);
  } else if (vbat < 10) {
    char charbat[3] = {0};
    itoa(vbat, charbat, 10);
    sprintf(batteryLevel.c_str(), "00%d", vbat);
  } else {
    strcpy(batteryLevel.c_str(), "000");
  }

  return batteryLevel;
}
String rssiLevel() {
  flushSim();
  Serial.println("AT+CSQ");
  String tempGSM = Serial.readString();
  int ind1 = tempGSM.indexOf(':');
  int ind2 = tempGSM.indexOf(',');
  String rssiLevel = tempGSM.substring(ind1 + 1, ind2);
  uint16_t n = rssiLevel.toInt();
  int r;
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  int p = abs(r);
  char charRSSI[8] = {0};
  if (p >= 100) {
    sprintf(charRSSI, "-%d", p);
  } else sprintf(charRSSI, "-0%d", p);
  return String(charRSSI);
}
bool gprsOn() {
  sendAtFram(5000, 31140, 9, "OK", "ERROR", 5); //"AT+CFUN=1"
  sendAtFram(5000, 31184, 29, "OK", "OK", 5); //"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""
  if (sendAtFram(5000, 31213, 12, "OK", "OK", 5)) { //"AT+SAPBR=1,1"
    if (sendAtFram(5000, 31225, 8, "OK", "ERROR", 5)) { //"AT+CIICR"
      if (sendAtFram(5000, 31233, 8, ">", "ERROR", 5)) { //"AT+CIFSR"
        return true;
      } else return false;
    } else return false;
  } else return false;
}
void gprsOff() {
  //if(getGprsState())
  { sendAtFram(5000, 31599, 10, "OK", "ERROR", 5); //"AT+CGATT=0"
  }
}
bool getGprsState() {//very slow!!
  Serial.setTimeout(10000);
  flushSim();
  Serial.write("AT+CGATT?");
  //Serial.println("");
  if (Serial.findUntil("1", "OK")) {
    return true;
  } else return false;
}
void flushSim() {
  uint16_t timeoutlp = 0;
  while (timeoutlp++ < 40) {
    while (Serial.available()) {
      Serial.read();
      timeoutlp = 0;  // If char was received reset the timer
    }
    delay(1);
  }
}
void writeDataFram(char* dataFram) {
  uint8_t dataFramSize = strlen(dataFram);
  for (unsigned long i = framWritePosition; i <= (dataFramSize + framWritePosition); i++)
  {
    fram.write8(i, dataFram[(i - framWritePosition)]);
  } framWritePosition += (dataFramSize) ;
}
void writeDataFramDebug(char* dataFram, long p1) {
  //for (unsigned long i = p1; i <= (p1 + strlen(dataFram)); i++)
  for (unsigned long i = p1; i < (p1 + strlen(dataFram)); i++)
  {
    fram.write8(i, dataFram[(i - p1)]);
  }
}
void powerUp() {
  while (analogRead(A3) < 200) {
    pinMode(5, OUTPUT);//PWR KEY
    digitalWrite(5, LOW);
    delay(2000);
    pinMode(5, INPUT_PULLUP);
    delay(100);
  }
}
void powerDown() {
  while (analogRead(A3) > 200) {
    pinMode(5, OUTPUT);//PWR KEY
    digitalWrite(5, LOW);
    delay(2000);
    pinMode(5, INPUT_PULLUP);
    delay(100);
  }
}
void blinkLED(int k) {
  for (int i = 0; i < k; i++) {
    digitalWrite(A0, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(A0, LOW);    // turn the LED off by making the voltage LOW
    delay(100);
  }
}
void blinkLEDFast(int k) {
  for (int i = 0; i < k; i++) {
    digitalWrite(A0, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(20);                       // wait for a second
    digitalWrite(A0, LOW);    // turn the LED off by making the voltage LOW
    delay(20);
  }
}
void simHardReset() {
  digitalWrite(6, HIGH);
  delay(10);
  digitalWrite(6, LOW);
  delay(200);
  digitalWrite(6, HIGH);
  powerUp();
  Serial.begin(4800);
}
void clearMemory(uint16_t size) {
  for (uint16_t a = 0; a < size; a++) {
    fram.write8(a, 0);
  }
  framWritePosition = 0;
}
void clearMemoryDiff(uint16_t size, uint16_t size1) {
  for (uint16_t a = size; a < size1; a++) {
    fram.write8(a, 0);
  }
  // if(size1==getCounter()){framWritePosition = 0;}
}
void clearMemoryDebug(uint16_t size) {
  for (uint16_t a = 32000; a < size; a++) {
    fram.write8(a, "0");
  }
}
void getWriteFromFram(uint16_t p1, uint16_t p2){
  for (uint16_t a = p1; a < p1+p2; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
    sprintf(Buffer, "%c", test);
  writeDataFram(Buffer);
  }
}
void insertMem() {
  framWritePosition = getCounter() * SizeRec;
  writeDataFram(fixStatus.c_str());                     //1
  writeDataFram(latitude.c_str());                      //10
  writeDataFram(longitude.c_str());                     //11
  writeDataFram(speed.c_str());                         //6
  writeDataFram(used_satellites.c_str());               //2
  writeDataFram(course.c_str());                        //6
  writeDataFram(batteryLevel().c_str());                //3
  writeDataFram(gpsTime.c_str());                  //10
  incrementCounter();
  if((getCounter()%limitToSend)==0){writeDataFramDebug("1",(32079+(getCounter()/limitToSend)));}
}

void incrementCounter() {
  int countVal=getCounter();
  countVal++;
  writeDataFramDebug(complete(String(countVal), 3).c_str(), 32000);
}
String complete(String s, int t) {
  while (strlen(s.c_str()) < t) {
    s = '0' + s;
  }
  return s;
}
int getCounter() {
  String retour = " ";
  for (long i = 32000; i <= 32002; i++) {
    char Buffer[5] = {0};
    uint8_t test = fram.read8(i);
    sprintf(Buffer, "%c", test);
    retour += Buffer;
  }
  return retour.toInt();
}
int getBatchCounter(uint16_t j) {
  String retour = " ";
  char Buffer[5] = {0};
  uint8_t test = fram.read8(32080+j);
  sprintf(Buffer, "%c", test);
  retour = Buffer;
  return retour.toInt();
}
int getValue(uint16_t position, uint8_t largeur) {
  String retour = " ";
  for (long i = position; i < position + largeur; i++) {
    char Buffer[2] = {0};
    uint8_t test = fram.read8(i);
    sprintf(Buffer, "%c", test);
    retour += Buffer;
  }
  return retour.toInt();
}
void incrementValue(uint16_t position, uint8_t largeur) {
  int countVal = getValue(position, largeur);
  countVal++;
  writeDataFramDebug(complete(String(countVal), largeur).c_str(), position);
}
void resetSS() {
  cfunReset();
  turnOnGns();
  while (getGsmStat() != 1) {
    delay(500);
  }
  gprsOn();
  restarted=true;
  gnsFailCounter = 0;
  gpsFailCounter = 0;
  httpActionFail = 0;
  FirstStartCounter = 0;
  ReStartCounter=0;
}
void hardResetSS() {
  // pinMode(5, OUTPUT);//PWR KEY
  // digitalWrite(5, LOW);
  // delay(2000);
  // pinMode(5, INPUT_PULLUP);
  // delay(100);
  // powerUp();
  // Serial.begin(4800);
  sendAtFram(6000, 31730, 11, "OK", "ERROR", 1);  //CFUN=1,1
  Serial.begin(4800);
  turnOnGns();
  while (getGsmStat() != 1) {delay(500);}
  gprsOn();
  restarted=true;
  gnsFailCounter = 0;
  gpsFailCounter = 0;
  httpActionFail = 0;
  FirstStartCounter = 0;
  ReStartCounter=0;
}
void cfunReset(){
  sendAtFram(6000, 31741, 9, "OK", "ERROR", 1);  //CFUN=0
  sendAtFram(6000, 31140, 9, "OK", "ERROR", 1);  //CFUN=1
}
bool sendAtFram(long timeout, uint16_t pos1, uint16_t pos2, char* Rep, char* Error, int nbRep) {
  flushSim();
  Serial.setTimeout(timeout);
  for (uint16_t a = pos1; a < pos1 + pos2; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
    sprintf(Buffer, "%c", test);
    Serial.print(Buffer);
  } Serial.println("");

  int compteur = 0;
  while ((!Serial.findUntil(Rep, Error)) && (compteur < nbRep)) {
    flushSim();
    for (uint16_t a = pos1; a < pos1 + pos2; a++)
    {
      uint8_t test = fram.read8(a);
      char Buffer[2] = {0};
      sprintf(Buffer, "%c", test);
      Serial.print(Buffer);
    } Serial.println("");
    compteur++;
    delay(50);
  }
  if (compteur <= nbRep)
  {
    return true;
  } else
  {
    return false;
  }
  Serial.setTimeout(1000);
}
void sendFromFram(uint16_t start,uint16_t length){
  for (uint16_t a = start; a < start + length; a++)
  {
    uint8_t test = fram.read8(a);
    char Buffer[2] = {0};
    sprintf(Buffer, "%c", test);
    Serial.print(Buffer);
    // writeDataFramDebug(Buffer,debugWriteCounter);
    // debugWriteCounter++;
  }

}
bool fireHttpAction(long timeout, char* Commande, char* Rep, char* Error) {
  flushSim();
  Serial.setTimeout(timeout);
  Serial.print(Commande);
  Serial.println(1, DEC);
  if(Serial.findUntil(Rep, Error)){
    // sendAtFram(2000, 31241, 11, "OK", "ERROR", 5); // httpterm
    ping =false;
    return true;
  } else{
    ping = true;
    return false;
    }
  Serial.setTimeout(1000);
}
void trace(unsigned long unixTime, uint8_t type) {
  uint8_t jour = (int)(((unixTime / 86400) + 4) % 7) + 1; //1 dimanche
  uint16_t positionEcriture = 32000 + jour * 10;
  if (jour == 2) {
    clearValue();
  }
  switch (type) {
    case 1:
      incrementValue(positionEcriture, 2);
      break;
    case 2:
      incrementValue(positionEcriture + 3, 3);
      break;
    case 3:
      incrementValue(positionEcriture + 6, 3);
      break;
  }
}
void clearValue() {
  if (getValue(32010, 2) > 0 ) {
    clearMemoryDiff(32010, 32080);
  }
}
bool insertGpsData() {
  if (getGpsData()) {
    gpsFailCounter = 0;
    insertMem();
    t1 = t2;
    return true;
  } else return false;
}

