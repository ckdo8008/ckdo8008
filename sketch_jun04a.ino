#include "M5Atom.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
  
#include "base64.hpp"


//#include <BLE2902.h>

#define RX_PIN      32
#define TX_PIN      26

//BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float seaLevel = 1013.3;
double t0, t1;
double advt0, advt1;
double advloopt0, advloopt1;

BLEServer *pServer;
byte cid = 1;
//BLEAdvertisementData custadvdata;

struct CallData {
  byte head1;
  byte head2;
  byte id;
  byte size;
  byte chksum;
  byte mode;
} callData = {0xff, 0xfe, 0x00, 0x02, 0x00, 0xa1};

byte recvbuff[20];

byte manbuff[4][20];
double recvmanbuff[4];
double termrecvman;

byte incomingbyte;
byte idx = 0;

// Advert Data select
byte currman = 0;
byte iocontrol[20];

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#pragma pack(push, 1)
struct ADUData {
  byte cid: 4;     
  byte val1_low: 8;
  byte val1_hi: 2;
  byte val2_low: 8;
  byte val2_hi: 2;
  byte val3_low: 8;
  byte val3_hi: 2;
  byte val4_low: 8;
  byte val4_hi: 2;
  byte val5_low: 8;
  byte val5_hi: 2; 
  byte dummy: 2;
};
#pragma pack(pop)

int man_code = 0x02E5;
int set_code = 0x03E5;

void setManData(String c, int c_size, BLEAdvertisementData &adv, int m_code) {
  String s;
  char b2 = (char)(m_code >> 8);
  m_code <<= 8;
  char b1 = (char)(m_code >> 8);
  s.concat(b1);
  s.concat(b2);
  s.concat(c);
  adv.setManufacturerData(s.c_str());
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("deviceConnected");
    }
};

// 명령 요청 처리 로직
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      //uint8_t* rxValue = pCharacteristic->getData();
      std::string value = pCharacteristic->getValue();

      unsigned int binary_length = decode_base64((unsigned char*)value.c_str(), iocontrol);
      Serial2.flush();
      Serial2.write(iocontrol, binary_length);
      delay(1);
    }
};

void setup() {
  M5.begin(true, false, true);
  Serial.begin(115200);
  Serial2.begin(250000, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);
  M5.dis.drawpix(0, 0xf00000);
  Serial.println(F("init succeeded."));

  BLEDevice::init("COAI");
  esp_ble_tx_power_set( ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N14 );
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
//  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
//  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
}

//void buttons_test() {
//  if (M5.Btn.isPressed()) {
//    delay(200);
//  }
//}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

// 데이터 전송
void advertising() {
    
    BLEAdvertisementData scan_response;
    scan_response.setName("COAI");
    
    advt0 = recvmanbuff[currman];
    advt1 = millis() - advt0;
    if (advt1 > 500) {
      // 만약 오래된 데이터라면
      String a = "";
      setManData(a, a.length() , scan_response, man_code);
    }
    else {
      ADUData bleadu;
      bleadu.cid = manbuff[currman][2];
      bleadu.val1_low = manbuff[currman][6];
      bleadu.val1_hi = manbuff[currman][7];
      bleadu.val2_low = manbuff[currman][9];
      bleadu.val2_hi = manbuff[currman][10];
      bleadu.val3_low = manbuff[currman][12];
      bleadu.val3_hi = manbuff[currman][13];
      bleadu.val4_low = manbuff[currman][15];
      bleadu.val4_hi = manbuff[currman][16];
      bleadu.val5_low = manbuff[currman][18];
      bleadu.val5_hi = manbuff[currman][19];
      bleadu.dummy = 0;
      
      int inputLen = sizeof(bleadu);
      unsigned char encoded[10];
      unsigned int base64_length = encode_base64((unsigned char*)&bleadu, inputLen, encoded);
      setManData((const char*)encoded, base64_length, scan_response, man_code);     
    }
    
    pServer->getAdvertising()->setScanResponseData(scan_response);
}

void readSerial() {
  if (Serial2.available() > 0) {
    incomingbyte = Serial2.read();

    if (idx == 0 && incomingbyte != 0xff){
      // stx가 확인 안되면 무시
      return;
    }
  
    if (idx == 0 && incomingbyte == 0xff) {
      idx = 0;
      recvbuff[idx++] = 0xff;
      return;
    }
    
    if (idx >= 1 && recvbuff[idx - 1] == 0xff && incomingbyte == 0xfe) {
      idx = 0;
      recvbuff[idx++] = 0xff;
      recvbuff[idx++] = incomingbyte;
      return;
    }

    if (idx > 1){
      recvbuff[idx++] = incomingbyte;
    }

    if (idx > 19) {
      // 정상 수신 시 데이터 수정 및 수신 시간 변경
      int sums = 0;
      for(unsigned int i = 2; i < 20; i++){
        sums = sums + recvbuff[i];
      }
      sums = sums % 256;
      
      if (sums == 255){
        memcpy(manbuff[recvbuff[2] - 1], recvbuff, 20);
        recvmanbuff[recvbuff[2] - 1] = millis();
//        Serial2.println(String(recvbuff[2] - 1));
      }

      idx = 0;
    }
  } else idx = 0;
}

void call(int id) {
  callData.id = (byte)id;
  callData.chksum = ~(byte)(callData.id + callData.size + callData.mode);  
  Serial2.flush();
  Serial2.write((byte*)&callData, sizeof(callData));
//  Serial2.write("hello");
}

void loop() {
  t1 = millis() - t0;

  // 주기별 전송
  // 150ms당 각 id 호출
  if (t1 > 150) {
    t0 = millis();
      
    call(cid++);
    if (cid == 5){
      cid = 1;
      //  ble 자동 연결 차단  
//      if(deviceConnected)
//        pServer->disconnectClient();
    }
    delay(1);
  }

  readSerial();
  advloopt1 = millis() - advloopt0;
  if (advloopt1 > 100){
    advloopt0 = millis();
    advertising();
    
    if (currman++ > 3)
      currman = 0;
  }
  
  M5.update(); 
}
