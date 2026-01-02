#include "esp_task_wdt.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <time.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX   "6e400002-b5a3-f393-e0a9-e50e24dcca9e" 
#define CHARACTERISTIC_RX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e" 

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
int sendData = 0;

const int SAMPLES_PER_NOTIFICATION = 10;
const int SAMPLE_SIZE = 12;
const int BUFFER_SIZE = SAMPLES_PER_NOTIFICATION * SAMPLE_SIZE;

struct Sample {
  uint64_t timestamp;
  float ecgData;
};

uint8_t sendBuffer[BUFFER_SIZE];
int sampleCount = 0;

const int led1 = 16;
const int led2 = 34;
const int led3 = 4;

const int XBTN = 33;
const int VBAT_SEN = 35;
const int LED1 = 4;
const int LED2 = 16;
const int LED3 = 34;
const int ADS_WCT = 32;
const int ADS_ALA = 26; 
const int ADS_DRDY = 25;
const int ADS_SS = 15;
const int ADS_MISO = 12;
const int ADS_MOSI = 13;
const int ADS_SCK = 14;
const int SD_DETECT = 17;
const int SD_SS = 5;
const int SD_MISO = 19;
const int SD_MOSI = 23;
const int SD_SCK = 18;
const int PIN_SDA = 21;
const int PIN_SCL = 22;

#define SDSPEED 27000000

SPIClass * ads = NULL;
SPIClass sdc(VSPI);

#define bufferECG_size 250
#define raw_mv_B 5000.00
#define offsetECG_B 4.00

volatile uint32_t bufferECG[bufferECG_size][4]; // [0,1,2] untuk saluran EKG, [3] untuk timestamp (millis)
volatile int bufferECG_p=bufferECG_size-1;
volatile int bufferECG_p2=bufferECG_size-1;
volatile boolean bufferFull=false;
volatile int bufferECG_i=0;
volatile uint32_t ctECG[3];
volatile uint32_t pctECG[3];

String fName;
byte XBTN_STAT=0;
boolean cardOK=true;
byte ctr;
volatile byte sx;
boolean btSend=false;
String stringBT;
String stringSD;
String timeStamp;
char btr;

#define CS_PIN 5
#define BUFFER_QUEUE_LENGTH 512 
#define BUFFER_LINE_LENGTH 128

QueueHandle_t sdQueue;
TaskHandle_t sdTaskHandle;

volatile uint64_t lastTimeStamp;
volatile uint64_t prevTime = 0;
volatile uint64_t timeperSampling;
volatile uint64_t timereadECG;

unsigned long lastLedBlink = 0; 
bool ledBlinkState = false; 
unsigned long lastLedBlinkDisconnected = 0;
bool ledBlinkDisconnected = false;

String formatUnixMillis(uint64_t unixMillis, int timezoneOffsetHours) {
  time_t seconds = (unixMillis / 1000) + timezoneOffsetHours * 3600;
  int millisPart = unixMillis % 1000;

  struct tm *tmInfo = gmtime(&seconds);  // Tetap pakai gmtime karena sudah di-offset
  char buffer[30];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
           tmInfo->tm_year + 1900,
           tmInfo->tm_mon + 1,
           tmInfo->tm_mday,
           tmInfo->tm_hour,
           tmInfo->tm_min,
           tmInfo->tm_sec,
           millisPart);

  return String(buffer);
}

void sdLoggingTask(void *parameter) {
  char fileName[fName.length()]; 
  fName.toCharArray(fileName, fName.length()+1);
  File logger=SD.open(fileName,FILE_APPEND);
  if (!logger) {
    Serial.println("Failed to open file for writing!");
    vTaskDelete(NULL); // Matikan task ini
    return;
  }

  char *lineBuffer;

  while (1) {
    // Tunggu data dari queue, dengan timeout
    if (xQueueReceive(sdQueue, &lineBuffer, pdMS_TO_TICKS(100)) == pdPASS) {
      logger.println(lineBuffer);  // Bisa juga pakai println jika pakai \n
      free(lineBuffer);
    }

    // Flush secara periodik
    static unsigned long lastFlush = millis();
    if (millis() - lastFlush > 5000) {
      logger.flush();  // flush tiap 5 detik
      lastFlush = millis();
    }

    // Tutup file setelah selesai record
    if(btSend==2){
      logger.flush();
      logger.close();      
      Serial.println("File closed, data saved.");
      vTaskDelete(NULL);
      btSend = 0;
      return;
    }
  }
}

void bufferToSD(String data) {
  if (uxQueueSpacesAvailable(sdQueue) > 0) {
    char *copy = (char *)malloc(BUFFER_LINE_LENGTH);
    if (copy != nullptr) {
      strncpy(copy, data.c_str(), BUFFER_LINE_LENGTH - 1);
      copy[BUFFER_LINE_LENGTH - 1] = '\0'; // Null-terminate
      if (xQueueSend(sdQueue, &copy, 0) != pdPASS) {
        free(copy); // Hindari memory leak
        Serial.println("Queue full! Data lost.");
      }
    }
  } else {
    Serial.println("Queue full! Data lost.");
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    sendData = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    Serial.println(rxValue);

    if (rxValue.length() > 0) {
      char command = rxValue[0];
      timeStamp = rxValue.substring(2);
      Serial.println(timeStamp);
      lastTimeStamp = strtoull(timeStamp.c_str(), NULL, 10);
      Serial.println(lastTimeStamp);
      if (command == 'S') {
        sendData = 1;
        Serial.println("Received command: START (S)");
      } else if (command == 'T') {
        sendData = 2;
        Serial.println("Received command: STOP (T)");
      }
    }
  }
};

void setup() {
  esp_task_wdt_deinit();
  pinMode(XBTN,INPUT_PULLUP);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  
  Serial.begin(115200);

  Serial.println("Initializing ECG");
  ads = new SPIClass(HSPI);
  ads->begin();
  ads->setBitOrder(MSBFIRST);
  ads->setDataMode(SPI_MODE0);
  pinMode(ADS_SS, OUTPUT);
  pinMode(ADS_DRDY, INPUT_PULLUP);
  ads_wreg(0x01,0x11); //CH1: INP to IN2 | INN to IN1
  ads_wreg(0x02,0x19); //CH1: INP to IN3 | INN to IN1
  ads_wreg(0x03,0x2E); //CH1: INP to IN5 | INN to IN6
  ads_wreg(0x0A,0x07); //enable common-mode detector on IN1,IN2, and IN3
  //ads_wreg(0x0B,0x07);
  ads_wreg(0x0C,0x04); //connect RLD amp output to IN4
  ads_wreg(0x0D,0x01); //1st buffer of Wilson ref to IN1
  ads_wreg(0x0E,0x02); //2nd buffer of Wilson ref to IN2
  ads_wreg(0x0F,0x03); //3rd buffer of Wilson ref to IN3
  ads_wreg(0x10,0x01); //Wilson ref out to IN6
  ads_wreg(0x12,0x04); //uses external crystal, feed out internal osc to digital
  //ads_wreg(0x13,0x34);
  //R2=8 R3=8 267Hz ODR
  ads_wreg(0x21,0x08);  //R2 dec rate 8 (all channel)
  ads_wreg(0x22,0x08);  //R3 dec rate 12 (ch1)
  ads_wreg(0x23,0x08);  //R3 dec rate 12 (ch2)
  ads_wreg(0x24,0x08);  //R3 dec rate 12 (ch3)
  ads_wreg(0x27,0x08);  //DRDYB src to ECG ch1/fastest ch
  ads_wreg(0x2F,0x70);  //Enable ECG ch1,2,3 loop read back
  ads_wreg(0x00,0x01);  //start data conversion
    
  Serial.println("Validating ECG registers");
  Serial.println("0x01 -> "+String(ads_rreg(0x01),HEX));
  Serial.println("0x02 -> "+String(ads_rreg(0x02),HEX));
  Serial.println("0x03 -> "+String(ads_rreg(0x03),HEX));
  Serial.println("0x0A -> "+String(ads_rreg(0x0A),HEX));
  Serial.println("0x0C -> "+String(ads_rreg(0x0C),HEX));
  Serial.println("0x0D -> "+String(ads_rreg(0x0D),HEX));
  Serial.println("0x0E -> "+String(ads_rreg(0x0E),HEX));
  Serial.println("0x0F -> "+String(ads_rreg(0x0F),HEX));
  Serial.println("0x10 -> "+String(ads_rreg(0x10),HEX));
  Serial.println("0x12 -> "+String(ads_rreg(0x12),HEX));
  Serial.println("0x21 -> "+String(ads_rreg(0x21),HEX));
  Serial.println("0x22 -> "+String(ads_rreg(0x22),HEX));
  Serial.println("0x23 -> "+String(ads_rreg(0x23),HEX));
  Serial.println("0x24 -> "+String(ads_rreg(0x24),HEX));
  Serial.println("0x27 -> "+String(ads_rreg(0x27),HEX));
  Serial.println("0x2F -> "+String(ads_rreg(0x2F),HEX));
  Serial.println("0x00 -> "+String(ads_rreg(0x00),HEX));

  Serial.println("done ECG Initialization");
  delay(500);

  sdc.begin(SD_SCK, SD_MISO, SD_MOSI, SD_SS); //SCLK, MISO, MOSI, SS
  Serial.println("Initializing SD Card");
  pinMode(SD_DETECT,INPUT_PULLUP);
  delay(1);
  if(digitalRead(SD_DETECT)) {
    Serial.println("No Card");
  }
  if(!SD.begin(SD_SS,sdc,SDSPEED)) {
    Serial.println("Card Error");
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No Card");
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC) {
    Serial.println("MMC");
  }
  else if(cardType == CARD_SD) {
    Serial.println("SDSC");
  }
  else if(cardType == CARD_SDHC) {
    Serial.println("SDHC");
  }
  else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

  File root = SD.open("/");
  if(!root) {
    Serial.println("Failed to open directory");
  // break;
  }
  if(!root.isDirectory()) {
    Serial.println("Not a directory");
  // break;
  }
  
  File file = root.openNextFile();
  int fNum=0; int fNmax=0;
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
    }
    else {
      fName = file.name();
      Serial.println(fName);
      byte fLen = fName.length();
      String fExt = fName;
      fExt.remove(0, fLen - 4);
      if (fExt == ".csv" || fExt == ".CSV") {
        Serial.println("csv detected");
      }
      byte fcp = 0;
      int cNum = 0;
      boolean validf = true;
      while (fcp < fName.length() - 4 && validf) {
        byte cN = fName.charAt(fcp);
        Serial.print(String(char(cN)));
        if (cN > 47 && cN < 58) {
          cNum *= 10;
          cNum += (cN - 48);
        } 
        else {
          validf = false;
        }
        fcp++;
      }
      if (validf) {
        Serial.print("valid file format existed: ");
        Serial.print(cNum);
        Serial.println(".csv");
        fNum = cNum;
      }
      if (fNmax < fNum) fNmax = fNum;
    }
    file = root.openNextFile();
  }
    
  Serial.println("last file number was "+String(fNmax)+".");
  
  fName="/"+String(fNmax+1)+".csv";
  Serial.println("Creating "+fName);
    
  char fileName[fName.length()];
  fName.toCharArray(fileName, fName.length()+1);
    
  File logger=SD.open(fileName,FILE_WRITE);
  if(!logger) {
    Serial.println("Failed to open file for writing");
  // break;
  }
  logger.println("YYYY-MM-DDTHH:MM:SS.SSS,YYYY-MM-DDTHH:MM:SS.SSS");
  if(logger.println("Time,ECG")) {
    Serial.println("File written");
  } 
  else {
    cardOK=false;
    Serial.println("Write failed");
  }
  logger.flush();

  // Inisialisasi Queue
  sdQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(char *));

  // Buat task logging di Core 0
  xTaskCreatePinnedToCore(
    sdLoggingTask,      // Fungsi task
    "SD Logging Task",  // Nama task
    4096,               // Stack size
    NULL,               // Parameter
    1,                  // Prioritas
    &sdTaskHandle,      // Handle task
    0                   // Core 0
  );
    
  Serial.println("SD Card Initialized...");

  // Inisialisasi BLE
  BLEDevice::init("EKG-ETA1-A");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_RX,
    BLECharacteristic::PROPERTY_WRITE
  );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  Serial.println("BLE ECG server started");
    
  Serial.print("Initializing ECG data stream listener...");
  attachInterrupt(ADS_DRDY, streamECG, FALLING);
  Serial.print("Done!");

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, HIGH);
  for(int i=1; i<=7; i++){
    if(i%2==0){
      digitalWrite(led3, HIGH);
    }
    else{
      digitalWrite(led3, LOW);
    }
    delay(200);
  }
  digitalWrite(led3, LOW);
}

void streamECG(){
  digitalWrite(ADS_SS, LOW);
  ads->transfer(0xD0);
  bufferECG_p++;
  if(bufferECG_p==bufferECG_size) bufferECG_p=0;
  for(byte chan=0;chan<3;chan++){
    if(bufferFull){
      ctECG[chan]-=bufferECG[bufferECG_p][chan]/bufferECG_size;
    }
    bufferECG[bufferECG_p][chan]=  ads->transfer(0)<<16;
    bufferECG[bufferECG_p][chan]|= ads->transfer(0)<<8;
    bufferECG[bufferECG_p][chan]|= ads->transfer(0);
    if(bufferFull){
      ctECG[chan]+=bufferECG[bufferECG_p][chan]/bufferECG_size;
    }
    else{
      bufferECG_i++;
      pctECG[chan]+=bufferECG[bufferECG_p][chan]/bufferECG_size;
      ctECG[chan]=pctECG[chan]/bufferECG_i;
      ctECG[chan]*=bufferECG_size;
      if(bufferECG_i==bufferECG_size){
        bufferFull=true;
      }
    }
  }
  digitalWrite(ADS_SS, HIGH);

  timereadECG = millis();
}


void ads_wreg(int reg, int val){
  digitalWrite(ADS_SS, LOW);
  ads->transfer(reg);
  ads->transfer(val);
  delayMicroseconds(1);
  digitalWrite(ADS_SS, HIGH);
}

int ads_rreg(int reg){
  int out = 0;
  digitalWrite(ADS_SS, LOW);
  ads->transfer(reg|1<<7);
  out = ads->transfer(0);
  delayMicroseconds(1);
  digitalWrite(ADS_SS, HIGH);
  return(out);
}


void loop() {
  if (deviceConnected && sendData == 1) {
    unsigned long currentTime = millis();
    if (ledBlinkState && currentTime - lastLedBlink >= 80) {
      digitalWrite(led3, LOW);
      ledBlinkState = false;
      lastLedBlink = currentTime;
    } else if (!ledBlinkState && currentTime - lastLedBlink >= 4000) {
      digitalWrite(led3, HIGH);
      ledBlinkState = true;
      lastLedBlink = currentTime;
    }
  } else if (!deviceConnected) {
    // Kedip cepat saat belum terhubung BLE
    unsigned long currentTime = millis();
    if (ledBlinkDisconnected && currentTime - lastLedBlinkDisconnected >= 80) {
      digitalWrite(led3, LOW);
      ledBlinkDisconnected = false;
      lastLedBlinkDisconnected = currentTime;
    } else if (!ledBlinkDisconnected && currentTime - lastLedBlinkDisconnected >= 200) {
      digitalWrite(led3, HIGH);
      ledBlinkDisconnected = true;
      lastLedBlinkDisconnected = currentTime;
    }
  } else {
    digitalWrite(led3, LOW);
    ledBlinkState = false;
  }
  
  if(bufferECG_p2 != bufferECG_p){
    ctr++;
    if(ctr==200)ctr=0;
    bufferECG_p2++;
    if (bufferECG_p2==bufferECG_size) bufferECG_p2=0;

    float ecgB=float(bufferECG[bufferECG_p2][1])-float(ctECG[1]);
    ecgB/=raw_mv_B;
    ecgB+=offsetECG_B;
    if(ecgB>8)ecgB=0;
    if(ecgB<-8)ecgB=0;

    timeperSampling = timereadECG - prevTime;
    lastTimeStamp += timeperSampling;

    if(sendData==1){
      Sample sample;
      sample.timestamp = lastTimeStamp;
      sample.ecgData = ecgB;

      // Copy timestamp and ECG data to buffer
      int offset = sampleCount * SAMPLE_SIZE;
      memcpy(&sendBuffer[offset], &sample.timestamp, 8);     // Copy 8 bytes of timestamp
      memcpy(&sendBuffer[offset + 8], &sample.ecgData, 4);  // Copy 4 bytes of ECG data

      sampleCount++;

      // Check if buffer is full
      if (sampleCount >= SAMPLES_PER_NOTIFICATION) {
        pTxCharacteristic->setValue(sendBuffer, BUFFER_SIZE);
        pTxCharacteristic->notify();
        sampleCount = 0;
      }

      stringSD = (formatUnixMillis(lastTimeStamp, 7)+","+String(ecgB,2));
      bufferToSD(stringSD);
    }

    Serial.println(ecgB);
    
    prevTime = timereadECG;   
  }
}
