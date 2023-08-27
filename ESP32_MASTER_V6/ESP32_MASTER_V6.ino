/**
   NANO SMART IOT

   Controlador: ESP32 - WROOM
   Date: 27/08/2023
   Autor: Walter Dawid Retzer
   Tipo: ESPNOW MASTER
   Firmware: V1.0.6
   
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

TFT_eSPI tft = TFT_eSPI();

// ========================================================================================================
// DEFINIÇÃO DAS CORES NO DISPLAY TFT:

// #define BLACK 0x0000
// #define BLUE 0x001F
// #define RED 0xF800
// #define GREEN 0x07E0
// #define CYAN 0x07FF
// #define MAGENTA 0xF81F
// #define YELLOW 0xFFE0
// #define WHITE 0xFFFF
// #define GREY 0x8410
// #define ORANGE2 0xE880
// #define ORANGE1 0xE800
// #define ORANGE 0xFD20
// #define GREENYELLOW 0xAFE5
// #define LIGHTGREY 0xC618
// #define DARKGREY 0x7BEF
// #define OLIVE 0x7BE0
// #define DARKGREEN 0x03E0
// #define DARKCYAN 0x03EF
// #define MARROM 0x7800
// #define PURPLE 0x780F

// ========================================================================================================
// DEFINIÇÃO DE VARIAVEIS USADOS NA FUNÇAÕ GAUGE NO DISPLAY TFT:

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
int d = 0;

String jsonData;
StaticJsonDocument<200> doc;

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

#define BUZZER_VCC 26
#define BUZZER_GND 27

esp_adc_cal_characteristics_t adc_cal;  //Estrutura que contem as informacoes para calibracao

//temperatura
bool esp32 = true;  // mude para falso ao usar o Arduino

int ThermistorPin;
double adcMax, Vs;

String TempReal;
double R1 = 10000.0;   // resitor 10k
double Beta = 3950.0;  // Beta value
double To = 298.15;    // temperatura em kelvin Kelvin
double Ro = 10000.0;   // resitor 10k em C

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);  // Scan only on one channel
  // reset on each scan
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
      // Print SSID and RSSI for each device found
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

        slave.channel = CHANNEL;  // pick a channel
        slave.encrypt = 0;        // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if (exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
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
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

uint8_t data = 0;
// send data
void sendData() {
  data++;

  jsonData = "";
  doc["CHILLER-001"] = "ON";
  doc["TI-001"] = data;
  doc["TI-002"] = data * -2;
  doc["TI-003"] = data * -4;
  doc["TI-004"] = data * -1;

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
    // How did we get so far!!
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

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  // pinMode(BUZZER_VCC, OUTPUT);  // Declara BUZZER como saída - PINO +VCC
  // pinMode(BUZZER_GND, OUTPUT);  // Declara BUZZER como saída - PINO GND

  // digitalWrite(BUZZER_GND, false);  // Reseta o BUZZER
  // digitalWrite(BUZZER_VCC, true);   // Seta o BUZZER
  // delay(250);
  // digitalWrite(BUZZER_VCC, false);  // Reseta o BUZZER
  // delay(250);
  // digitalWrite(BUZZER_VCC, true);  // Seta o BUZZER
  // delay(250);
  // digitalWrite(BUZZER_VCC, false);  // Reseta o BUZZER
  // delay(250);
  // digitalWrite(BUZZER_VCC, true);  // Seta o BUZZER
  // delay(250);
  // digitalWrite(BUZZER_VCC, false);  // Reseta o BUZZER

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);                                                              //pin 34 esp32 devkit v1
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);  //Inicializa a estrutura de calibracao

  ThermistorPin = 34;
  adcMax = 4095.0;  // ADC 12-bit (0-4095)
  Vs = 3.3;         // voltagem

  uint32_t AD = 0;
  for (int i = 0; i < 100; i++) {
    AD += adc1_get_raw(ADC1_CHANNEL_6);  //Obtem o valor RAW do ADC
    ets_delay_us(30);
  }
  AD /= 100;

  double Vout, Rt = 0;
  double T, Tc, Tf = 0;

  double adc = 0;

  adc = analogRead(ThermistorPin);
  adc = AD;

  Vout = adc * Vs / adcMax;
  Rt = R1 * Vout / (Vs - Vout);

  T = 1 / (1 / To + log(Rt / Ro) / Beta);  //  Kelvin
  Tc = T - 273.15;                         // Celsius
  Serial.print(Tc);
  Serial.println(" Celsius");
  delay(2000);

  Serial.println("initialisation done.");

  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL ");
  Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

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

void loop() {

  tft.fillScreen(TFT_BLACK);  // portrait
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) {  // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
      sendData();
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  } else {
    // No slave found to process
  }
  // tft.fillScreen(TFT_BLUE);
  // delay(2000);

  uint32_t AD = 0;
  for (int i = 0; i < 100; i++) {
    AD += adc1_get_raw(ADC1_CHANNEL_6);  //Obtem o valor RAW do ADC
    ets_delay_us(30);
  }
  AD /= 100;

  double Vout, Rt = 0;
  double T, Tc, Tf = 0;

  double adc = 0;

  adc = analogRead(ThermistorPin);
  adc = AD;

  Vout = adc * Vs / adcMax;
  Rt = R1 * Vout / (Vs - Vout);

  T = 1 / (1 / To + log(Rt / Ro) / Beta);  //  Kelvin
  Tc = T - 273.15;                         // Celsius
  Serial.print(Tc);
  Serial.println(" Celsius");

  tft.setCursor(70, 25);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("TEMPERATURA"));

  tft.setCursor(105, 50);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("VASO-001"));

  int xpos = 20, ypos = 120, radius = 140;                                 // position of upper ring and proportion
  ringMeter(-1.50, -25.00, 40, xpos, ypos, radius, "TI-001", GREEN2RED);  // draw analog mete
  delay(5000);
  displayLogoHalfMouth();
  delay(5000);
}

//####################################################################################################
// Draw a JPEG on the TFT pulled from SD Card
//####################################################################################################
// xpos, ypos is top left corner of plotted image
void drawSdJpeg(const char *filename, int xpos, int ypos) {

  // Open the named file (the Jpeg decoder library will close it)
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

  // Use one of the following methods to initialise the decoder:
  bool decoded = JpegDec.decodeSdFile(jpegFile);  // Pass the SD file handle to the decoder,
  //bool decoded = JpegDec.decodeSdFile(filename);  // or pass the filename (String or character array)

  if (decoded) {
    // print information about the image to the serial port
    jpegInfo();
    // render the image onto the screen at given coordinates
    jpegRender(xpos, ypos);
  } else {
    Serial.println("Jpeg file format not supported!");
  }
}

//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void jpegRender(int xpos, int ypos) {

  //jpegInfo(); // Print information from the JPEG file (could comment this line out)

  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = jpg_min(mcu_w, max_x % mcu_w);
  uint32_t min_h = jpg_min(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // Fetch data from the file, decode and display
  while (JpegDec.read()) {  // While there is more data in the file
    pImg = JpegDec.pImage;  // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
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

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    // draw image MCU block only if it will fit on the screen
    if ((mcu_x + win_w) <= tft.width() && (mcu_y + win_h) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ((mcu_y + win_h) >= tft.height())
      JpegDec.abort();  // Image has run off bottom of screen so abort decoding
  }

  tft.setSwapBytes(swapBytes);
}

//####################################################################################################
// Print image information to the serial port (optional)
//####################################################################################################
// JpegDec.decodeFile(...) or JpegDec.decodeArray(...) must be called before this info is available!
void jpegInfo() {

  // Print information extracted from the JPEG file
  Serial.println("JPEG image info");
  Serial.println("===============");
  Serial.print("Width      :");
  Serial.println(JpegDec.width);
  Serial.print("Height     :");
  Serial.println(JpegDec.height);
  Serial.print("Components :");
  Serial.println(JpegDec.comps);
  Serial.print("MCU / row  :");
  Serial.println(JpegDec.MCUSPerRow);
  Serial.print("MCU / col  :");
  Serial.println(JpegDec.MCUSPerCol);
  Serial.print("Scan type  :");
  Serial.println(JpegDec.scanType);
  Serial.print("MCU width  :");
  Serial.println(JpegDec.MCUWidth);
  Serial.print("MCU height :");
  Serial.println(JpegDec.MCUHeight);
  Serial.println("===============");
  Serial.println("");
}

// ========================================================================================================
// FUNÇÃO GRÁFICA PARA EXIBIÇÃO DE GAUGE ANALÓGICO NO DISPLAY:

int ringMeter(float value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme) {

  x += r;
  y += r;                                         // calculate coordinates of center of ring
  int w = r / 3;                                  // width of outer ring is 1/4 of radius
  int angle = 150;                                // half the sweep angle of the meter (300 degrees)
  int v = map(value, vmin, vmax, -angle, angle);  // map the value to an angle v
  byte seg = 3;                                   // segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6;                                   // draw segments every 3 degrees, increase to 6 for segmented ring
  //int colour = BLUE;                                                        // variable to save "value" text color from scheme and set default
  int colour = TFT_GREEN;  // variable to save "value" text color from scheme and set default

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
        case 0: colour = TFT_RED; break;                                      // fixed color
        case 1: colour = TFT_GREEN; break;                                    // fixed color
        case 2: colour = TFT_BLUE; break;                                     // fixed colour
        case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break;   // full spectrum blue to red
        case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break;  // green to red (high temperature etc)
        case 5:
          colour = rainbow(map(i, -angle, angle, 127, 63));
          break;  // red to green (low battery etc)
        //default: colour = BLUE; break;                                   // fixed color
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

// ========================================================================================================
// FUNÇÃO GRÁFICA PARA EXIBIÇÃO DE GAUGE ANALÓGICO NO DISPLAY: COR DA PALHETA RAINBOW

unsigned int rainbow(byte value)  // value is expected to be in range 0-127
{                                 // value is converted to a spectrum color from 0 = blue through to 127 = red
  byte red = 0;                   // red is the top 5 bits of a 16 bit colour value
  byte green = 0;                 // green is the middle 6 bits
  byte blue = 0;                  // blue is the bottom 5 bits
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


// ========================================================================================================
// FUNÇÃO GRÁFICA PARA EXIBIÇÃO DE GAUGE ANALÓGICO NO DISPLAY: FAZE DO ANGULO EM GRAUS

float sineWave(int phase) {
  return sin(phase * 0.0174532925);
}

void displayLogoHalfMouth() {
  tft.fillScreen(TFT_WHITE);
  tft.setRotation(0);                  // portrait
  drawSdJpeg("/logoV3.jpg", 70, 150);  // This draws a jpeg pulled off the SD Card
  tft.setFreeFont(FF44);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(45, 380);
  tft.print(F("HalfMouth"));
  tft.setFreeFont(FF34);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(64, 410);
  tft.print(F("Cervejas Artesanais"));
}
