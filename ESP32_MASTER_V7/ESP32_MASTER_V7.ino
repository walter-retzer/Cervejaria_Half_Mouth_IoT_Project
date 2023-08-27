/**
   NANO SMART IOT

   Controlador: ESP32 - WROOM
   Date: 27/08/2023
   Autor: Walter Dawid Retzer
   Tipo: ESPNOW MASTER
   Firmware: V1.0.7
   
   Note: Configuração inicial de leitura dos termistores;
         Exibição da Tela de Logo da Cervejaria;
         Exibição da temperatura no display em formato de medidor analógico.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "Free_Fonts.h"
#include "FS.h"
#include "SD.h"
#include <JPEGDecoder.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>

//========================================================================================================
// Variaveis de Configuração do Display TFT:
TFT_eSPI tft = TFT_eSPI();
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define ORANGE2RED 6
#define SCALE0 0x528A
#define SCALE1 0x3151
#define TEXT_COLOR 0xFFFF
//========================================================================================================
// Variaveis de Configuração dos Dados Enviados em formato JSON ao ESP32 SLAVE com ESPNOW:
String jsonData;
StaticJsonDocument<200> doc;
uint8_t data = 0;
//========================================================================================================
// Variaveis de Configuração da Comunicação ESPNOW do ESP32:
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
//========================================================================================================
// Variaveis de Configuração do Buzzer:
#define BUZZER_VCC 26
#define BUZZER_GND 27
//========================================================================================================
// Variaveis de Configuração da Medição de Temperatura com Termistor:
esp_adc_cal_characteristics_t adc_cal;  //Estrutura que contem as informacoes para calibracao
bool esp32 = true;
int thermistorPin;
double adcMax, vccThermistor;
String TempReal;
double R1 = 10000.0;    // resitor 10k
double Beta = 3950.0;   // Beta value
double To = 298.15;     // temperatura em Kelvin
double Ro = 10000.0;    // resitor 10k em Paralelo
double tempVaso = 0.0;  // valor de temp do vaso 001
//========================================================================================================


/****************************************SETUP***********************************************************/
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_VCC, OUTPUT);      // Declara BUZZER como saída - PINO +VCC
  pinMode(BUZZER_GND, OUTPUT);      // Declara BUZZER como saída - PINO GND
  digitalWrite(BUZZER_GND, false);  // Reseta o BUZZER
  digitalWrite(BUZZER_VCC, true);   // Seta o BUZZER

  delay(250);
  digitalWrite(BUZZER_VCC, false);  // Reseta o BUZZER
  delay(250);
  digitalWrite(BUZZER_VCC, true);  // Seta o BUZZER
  delay(250);
  digitalWrite(BUZZER_VCC, false);  // Reseta o BUZZER
  delay(250);
  digitalWrite(BUZZER_VCC, true);  // Seta o BUZZER
  delay(250);
  digitalWrite(BUZZER_VCC, false);  // Reseta o BUZZER

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);                                                              //pin 34 esp32 devkit v1
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);  //Inicializa a estrutura de calibracao

  thermistorPin = 34;   // Pino de Entrada
  adcMax = 4095.0;      // ADC 12-bit (0-4095)
  vccThermistor = 3.3;  // Alimentação VCC Termistor

  WiFi.mode(WIFI_STA);                                   // Inicia WiFi para ler o Endereço MAC Address
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);  // Inicia Config do Canal de comunicação do ESPNOW
  Serial.println("ESPNOW CONFIG IN MASTER");
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL ");
  Serial.println(WiFi.channel());

  InitESPNow();                          // Inicia Config para Ativar Comunicação ESPNOW
  esp_now_register_send_cb(OnDataSent);  // Callback de Envio de dados pelo ESPNOW

  tft.begin();

  if (!SD.begin(5)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  delay(1000);
  displayLogoHalfMouth();
  delay(5000);
}
/**************************************END_SETUP*********************************************************/
//========================================================================================================


//========================================================================================================
/****************************************LOOP************************************************************/
void loop() {
  tft.fillScreen(TFT_BLACK);

  uint32_t AD = 0;
  for (int i = 0; i < 100; i++) {
    AD += adc1_get_raw(ADC1_CHANNEL_6);  //Obtem o valor RAW do ADC
    ets_delay_us(30);
  }
  AD /= 100;

  double vOut, Rt = 0;
  double tempKelvin, tempCelsius = 0;
  double adc = 0;

  adc = analogRead(thermistorPin);
  adc = AD;
  vOut = adc * vccThermistor / adcMax;
  Rt = R1 * vOut / (vccThermistor - vOut);
  tempKelvin = 1 / (1 / To + log(Rt / Ro) / Beta);  //  Kelvin
  tempCelsius = tempKelvin - 273.15;                // Celsius
  data = int(round(tempCelsius)) - 1;
  Serial.print(tempCelsius);
  Serial.println(" Celsius");

  scanForSlave();
  if (slave.channel == CHANNEL) {  // check if slave channel is defined
    bool isPaired = manageSlave();
    if (isPaired) {
      sendData();
    } else {
      Serial.println("Slave pair failed!");
    }
  } else {
    // No slave found to process
  }

  tft.setCursor(70, 25);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("TEMPERATURA"));

  tft.setCursor(105, 50);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("VASO-001"));

  int xpos = 20, ypos = 120, radius = 140;                                // Posição do medidor gauge analógico no display
  ringMeter(-1.50, -25.00, 40, xpos, ypos, radius, "TI-001", GREEN2RED);  // Função que exibe o medidor analógico no display
  delay(5000);
  displayLogoHalfMouth();
  delay(5000);
}
/**************************************END_LOOP**********************************************************/
//========================================================================================================


//========================================================================================================
/************************************INIT_ESP_NOW********************************************************/
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}
/**********************************END_INIT_ESP_NOW******************************************************/


/**********************************SCAN_SLAVE_DEVICES******************************************************/
void scanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);  // Scan only on one channel
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            slave.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        slave.channel = CHANNEL;
        slave.encrypt = 0;
        slaveFound = 1;
        break;
      }
    }
  }
  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }
  WiFi.scanDelete();
}
/*******************************END_SCAN_SLAVES_DEVICES****************************************************/


/*******************************MENAGE_SLAVES_DEVICES****************************************************/
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }
    Serial.print("Slave Status: ");
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if (exists) {
      Serial.println("Already Paired");
      return true;
    } else {
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    Serial.println("No Slave found to process");
    return false;
  }
}
/***************************END_MENAGE_SLAVES_DEVICES****************************************************/


/******************************DELETE_PEER_DEVICE********************************************************/
void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}
/*******************************END_DELETE_PEER_DEVICE***************************************************/


/******************************SEND_DATA_TO_SLAVE_DEVICE**************************************************/
void sendData() {
  data++;
  jsonData = "";
  doc["CHILLER-001"] = "ON";
  doc["TI-001"] = data;
  doc["TI-002"] = data * -1;
  doc["TI-003"] = data * -2;
  doc["TI-004"] = data * -3;

  size_t length = sizeof(jsonData) * 5;
  serializeJson(doc, jsonData);

  const uint8_t *peer_addr = slave.peer_addr;
  Serial.print("Sending: ");
  Serial.println(jsonData);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)jsonData.c_str(), length + 2);

  Serial.println(sizeof(jsonData));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}
/****************************END_SEND_DATA_TO_SLAVE_DEVICE************************************************/


/**************************CALLBACK_SEND_DATA_TO_SLAVE_DEVICE*********************************************/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
/************************END_CALLBACK_SEND_DATA_TO_SLAVE_DEVICE*******************************************/


/**************************************DRAW_SD_IMAGE_JPEG*************************************************/
void drawSdJpeg(const char *filename, int xpos, int ypos) {
  File jpegFile = SD.open(filename, FILE_READ);  // or, file handle reference for SD library

  if (!jpegFile) {
    Serial.print("ERROR: File \"");
    Serial.print(filename);
    Serial.println("\" not found!");
    return;
  }

  Serial.println("===========================");
  Serial.print("Drawing file: ");
  Serial.println(filename);
  Serial.println("===========================");

  bool decoded = JpegDec.decodeSdFile(jpegFile);  // Pass the SD file handle to the decoder,

  if (decoded) {
    jpegRender(xpos, ypos);
  } else {
    Serial.println("Jpeg file format not supported!");
  }
}
/************************************END_DRAW_SD_IMAGE_JPEG***********************************************/


/************************************JPEG_RENDER_IN_DISPLAY***********************************************/
void jpegRender(int xpos, int ypos) {
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);
  uint32_t min_w = jpg_min(mcu_w, max_x % mcu_w);
  uint32_t min_h = jpg_min(mcu_h, max_y % mcu_h);
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;
  uint32_t drawTime = millis();

  max_x += xpos;
  max_y += ypos;
  while (JpegDec.read()) {  // While there is more data in the file
    pImg = JpegDec.pImage;  // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;
    if (win_w != mcu_w) {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++) {
        p += mcu_w;
        for (int w = 0; w < win_w; w++) {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }
    uint32_t mcu_pixels = win_w * win_h;
    if ((mcu_x + win_w) <= tft.width() && (mcu_y + win_h) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ((mcu_y + win_h) >= tft.height())
      JpegDec.abort();  // Image has run off bottom of screen so abort decoding
  }
  tft.setSwapBytes(swapBytes);
}
/**********************************END_JPEG_RENDER_IN_DISPLAY*********************************************/


/**************************************MEDIDOR_ANALÓGICO**************************************************/
int ringMeter(float value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme) {
  x += r;
  y += r;                                         // calculate coordinates of center of ring
  int w = r / 3;                                  // width of outer ring is 1/4 of radius
  int angle = 150;                                // half the sweep angle of the meter (300 degrees)
  int v = map(value, vmin, vmax, -angle, angle);  // map the value to an angle v
  byte seg = 3;                                   // segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6;                                   // draw segments every 3 degrees, increase to 6 for segmented ring
  int colour = TFT_GREEN;                         // variable to save "value" text color from scheme and set default

  for (int i = -angle + inc / 2; i < angle - inc / 2; i += inc)  // draw color blocks every increment degrees
  {
    float sx = cos((i - 90) * 0.0174532925);  // calculate pair of coordinates for segment start
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    float sx2 = cos((i + seg - 90) * 0.0174532925);  // salculate pair of coordinates for segment end
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) {  // fill in coloured segments with 2 triangles
      switch (scheme) {
        case 0: colour = TFT_RED; break;                                  // fixed color
        case 1: colour = TFT_GREEN; break;                                // fixed color
        case 2: colour = TFT_BLUE; break;                                 // fixed colour
        case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break;   // full spectrum blue to red
        case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break;  // green to red (high temperature etc)
        case 5:
          colour = rainbow(map(i, -angle, angle, 127, 63));
          break;                             // red to green (low battery etc)
        default: colour = TFT_GREEN; break;  // fixed color
      }
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
    } else  // fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, SCALE1);  // color of unoccupied ring scale segment triangle pointing outward
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, SCALE0);  // color of unoccupied ring scale segment traiangle pointing inward
    }
  }

  char buf[10];  // convert value to a string
  byte len = 2;
  dtostrf(value, len, 0, buf);

  if (value >= 0) {
    tft.setTextColor(colour, TFT_BLACK);
    tft.setCursor(x - 55, y + 25);
    tft.setFreeFont(FMB24);
    tft.setTextSize(2);
    tft.print(buf);
    tft.setTextSize(1);
  }
  if (value < 0) {
    tft.setTextColor(colour, TFT_BLACK);
    tft.setCursor(x - 75, y + 25);
    tft.setFreeFont(FMB24);
    tft.setTextSize(2);
    tft.print(buf);
    tft.setTextSize(1);
  }
  if (units == "TI-001") {
    tft.setCursor(130, 360);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFreeFont(FF22);
    tft.print(F("O"));
    tft.setCursor(150, 380);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFreeFont(FF20);
    tft.print(F("C"));
    tft.setCursor(130, 405);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFreeFont(FF17);
    tft.print(F("TI-001"));
  }
  return x + r;  // calculate and return right hand side x coordinate
}
/************************************END_MEDIDOR_ANALÓGICO************************************************/


/************************************GAUGE_MEDIDOR_ANALÓGICO************************************************/
unsigned int rainbow(byte value) {
  byte red = 0;
  byte green = 0;
  byte blue = 0;
  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}
/**********************************END_GAUGE_MEDIDOR_ANALÓGICO**********************************************/


/**********************************FASE_GAUGE_MEDIDOR_ANALÓGICO**********************************************/
float sineWave(int phase) {
  return sin(phase * 0.0174532925);
}
/*******************************END_FASE_GAUGE_MEDIDOR_ANALÓGICO*********************************************/


/***************************************LOGO_CERVEJARIA******************************************************/
void displayLogoHalfMouth() {
  tft.fillScreen(TFT_WHITE);
  tft.setRotation(0);
  drawSdJpeg("/logoV3.jpg", 70, 150);
  tft.setFreeFont(FF44);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(45, 380);
  tft.print(F("HalfMouth"));
  tft.setFreeFont(FF34);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(64, 410);
  tft.print(F("Cervejas Artesanais"));
}
/*************************************END_LOGO_CERVEJARIA****************************************************/
