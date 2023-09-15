/**
   NANO SMART IOT

   Controlador: ESP32 - WROOM
   Date: 15/09/2023
   Autor: Walter Dawid Retzer
   Comunicação: ESPNOW MASTER
   Firmware: V1.0.1
   
   Note: Calibração dos sensores do tipo NTC e ajuste dos offset de leitura dos sensores dos Fermentadores FM-001 e FM-002;
         Inclusão da verificação de estado de funcionamento da bomba de recirculação;
         Ajuste do envio das leituras das temperaturas em formato double durante a comunicação ESP-NOW;
         Correção do bug dos valores max e min do TI-002;
         Ajuste da orientação da tela do display TFT.
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
#include <Wire.h>
#include "RTClib.h"

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
int32_t data1 = 0;
int32_t data2 = 0;
int32_t data3 = 0;
int32_t data4 = 0;
int bombState;
int camaraState;
#define STATUS_CHILLER 16
#define GND_CHILLER 17
#define STATUS_CAMARA 25
#define GND_CAMARA 25
//========================================================================================================
// Variaveis de Configuração da Comunicação ESPNOW do ESP32:
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
String macAddress;
String SSID;
String BSSIDstr;
int32_t RSSI = 0;
int sendDataOk;
int sendDataNotOk;
bool isModeEspNowOk = false;
unsigned long auxTimeSendValue;
unsigned long auxTimeLoop1;
unsigned long auxTimeLoop2;
unsigned long auxTimeLoop3;
unsigned long auxTimeLoop4;
unsigned long auxTimeLoop5;
unsigned long auxTimeLoop6;
unsigned long intervalo = 300000;
unsigned long intervaloLoop1 = 10000;
unsigned long intervaloLoop2 = 20000;
unsigned long intervaloLoop3 = 30000;
unsigned long intervaloLoop4 = 40000;
unsigned long intervaloLoop5 = 50000;
unsigned long intervaloLoop6 = 60000;
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
double R1 = 10000.0;        // resitor 10k
double Beta = 3950.0;       // Beta value
double To = 298.15;         // temperatura em Kelvin
double Ro = 10000.0;        // resitor 10k em Paralelo
double offsetTemp1 = 3.40;  // valor de offset da temperatura 1
double offsetTemp2 = 2.55;  // valor de offset da temperatura 2
double offsetTemp3 = 0;     // valor de offset da temperatura 3
double offsetTemp4 = 4.50;  // valor de offset da temperatura 4
double tempCelsius = 0;
double tempCelsius1 = 0;
double tempCelsiusMax1 = 0;
double tempCelsiusMin1 = 0;
double tempCelsius2 = 0;
double tempCelsiusMax2 = 0;
double tempCelsiusMin2 = 0;
double tempCelsius3 = 0;
double tempCelsiusMax3 = 0;
double tempCelsiusMin3 = 0;
double tempCelsius4 = 0;
double tempCelsiusMax4 = 0;
double tempCelsiusMin4 = 0;
//========================================================================================================
// Variaveis de Configuração do Módulo DS3231:
RTC_DS3231 rtc;
char days[7][12] = { "Domingo", "Segunda", "Terca", "Quarta", "Quinta", "Sexta", "Sabado" };
//========================================================================================================
// Variaveis Aux de contagem de gravação no SD CARD:
int saveSDCardOk;
int saveSDCardNotOk;
//========================================================================================================



/****************************************SETUP***********************************************************/
void setup() {
  Serial.begin(115200);
  delay(2000);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);                                                              //pin 34 esp32 devkit v1
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);  //Inicializa a estrutura de calibracao

  adcMax = 4095.0;      // ADC 12-bit (0-4095)
  vccThermistor = 3.3;  // Alimentação VCC Termistor
  readTermistor(ADC1_CHANNEL_6, 34);
  tempCelsiusMax1 = -100.00;
  tempCelsiusMin1 = 100.00;
  readTermistor(ADC1_CHANNEL_7, 35);
  tempCelsiusMax2 = -100.00;
  tempCelsiusMin2 = 100.00;
  readTermistor(ADC1_CHANNEL_4, 32);
  tempCelsiusMax3 = -100.00;
  tempCelsiusMin3 = 100.00;
  readTermistor(ADC1_CHANNEL_5, 33);
  tempCelsiusMax4 = -100.00;
  tempCelsiusMin4 = 100.00;
  sendDataOk = 0;
  sendDataNotOk = 0;
  saveSDCardOk = 0;
  saveSDCardNotOk = 0;

  WiFi.mode(WIFI_STA);                                   // Inicia WiFi para ler o Endereço MAC Address
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);  // Inicia Config do Canal de comunicação do ESPNOW
  Serial.println(F("ESPNOW CONFIG IN MODE: MASTER"));
  Serial.print(F("STA MAC: "));
  Serial.println(WiFi.macAddress());
  macAddress = WiFi.macAddress();
  Serial.print(F("STA CHANNEL: "));
  Serial.println(WiFi.channel());

  InitESPNow();                          // Inicia Config para Ativar Comunicação ESPNOW
  esp_now_register_send_cb(OnDataSent);  // Callback de Envio de dados pelo ESPNOW

  scanForSlave();
  delay(1000);

  if (!rtc.begin()) {
    Serial.println(F("Could not find RTC! Check circuit."));
  }
  //rtc.adjust(DateTime(__DATE__, __TIME__));

  tft.begin();
  delay(1000);

  pinMode(BUZZER_VCC, OUTPUT);  // Declara BUZZER como saída - PINO +VCC
  pinMode(BUZZER_GND, OUTPUT);  // Declara BUZZER como saída - PINO GND

  pinMode(GND_CHILLER, OUTPUT);
  pinMode(GND_CAMARA, OUTPUT);
  digitalWrite(GND_CHILLER, LOW);
  digitalWrite(GND_CAMARA, LOW);

  pinMode(STATUS_CHILLER, INPUT_PULLUP);  // Declara Pino do Status do CHILLER como entrada
  // pinMode(STATUS_CAMARA, INPUT_PULLUP);   // Declara Pino do Status do CAMARA FRIA como entrada

  startDisplayScreen();
  displaySetup();
  writeFile(SD, "/setup.txt", "NANO SMART - SETUP REALIZADO;");
  delay(15000);
  auxTimeSendValue = millis();
  auxTimeLoop1 = millis();
  auxTimeLoop2 = millis();
  auxTimeLoop3 = millis();
  auxTimeLoop4 = millis();
  auxTimeLoop5 = millis();
  auxTimeLoop6 = millis();
  // tft.drawLine(160, 0, 160, 480, TFT_BLUE);
  // tft.drawLine(0, 160, 320, 160, TFT_BLUE);
}
/**************************************END_SETUP*********************************************************/
//========================================================================================================


//========================================================================================================
/****************************************LOOP************************************************************/
void loop() {
  bombState = digitalRead(STATUS_CHILLER);
  // camaraState = digitalRead(STATUS_CAMARA);
  camaraState = false;

  if (millis() - auxTimeLoop1 > intervaloLoop1) {
    displayLogoHalfMouth();
    reseteMinMaxValues();
    auxTimeLoop1 = millis();
  }

  if (millis() - auxTimeLoop2 > intervaloLoop2) {
    showTempThermistor1();
    reseteMinMaxValues();
    auxTimeLoop1 = millis();
    auxTimeLoop2 = millis();
  }

  if (millis() - auxTimeLoop3 > intervaloLoop3) {
    showTempThermistor2();
    reseteMinMaxValues();
    auxTimeLoop1 = millis();
    auxTimeLoop2 = millis();
    auxTimeLoop3 = millis();
  }

  if (millis() - auxTimeLoop4 > intervaloLoop4) {
    showTempThermistor1();
    reseteMinMaxValues();
    auxTimeLoop1 = millis();
    auxTimeLoop2 = millis();
    auxTimeLoop3 = millis();
    auxTimeLoop4 = millis();
  }

  if (millis() - auxTimeLoop5 > intervaloLoop5) {
    showTempThermistor2();
    reseteMinMaxValues();
    auxTimeLoop1 = millis();
    auxTimeLoop2 = millis();
    auxTimeLoop3 = millis();
    auxTimeLoop4 = millis();
    auxTimeLoop5 = millis();
  }

  if (millis() - auxTimeLoop6 > intervaloLoop6) {
    displayReport();
    reseteMinMaxValues();
    auxTimeLoop1 = millis();
    auxTimeLoop2 = millis();
    auxTimeLoop3 = millis();
    auxTimeLoop4 = millis();
    auxTimeLoop5 = millis();
    auxTimeLoop6 = millis();
  }

  if (millis() - auxTimeSendValue > intervalo) {
    scanForSlave();
    if (slave.channel == CHANNEL) {  // check if slave channel is defined
      bool isPaired = manageSlave();
      if (isPaired) {
        sendData();
      } else {
        Serial.println(F("Slave pair failed!"));
      }
    } else {
      // No slave found to process
      jsonData = "";
      sendDataNotOk++;
    }
    writeFileJson(SD, "/send_value.txt", "NANO SMART - SEND VALUE TO SIM800L;");
    auxTimeSendValue = millis();
  }
}
/**************************************END_LOOP**********************************************************/
//========================================================================================================



//========================================================================================================
/************************************RESETE_MIN_AND_MAX***************************************************/
void reseteMinMaxValues() {
  DateTime now = rtc.now();
  String hour = String(now.hour());
  String minute = String(now.minute());
  if (hour == "0" && minute == "0") {
    Serial.println(F("Valores MIN e MAX resetados!"));
    readTermistor(ADC1_CHANNEL_6, 34);
    tempCelsiusMax1 = -100.00;
    tempCelsiusMin1 = 100.00;
    readTermistor(ADC1_CHANNEL_7, 35);
    tempCelsiusMax2 = -100.00;
    tempCelsiusMin2 = 100.00;
    readTermistor(ADC1_CHANNEL_4, 32);
    tempCelsiusMax3 = -100.00;
    tempCelsiusMin3 = 100.00;
    readTermistor(ADC1_CHANNEL_5, 33);
    tempCelsiusMax4 = -100.00;
    tempCelsiusMin4 = 100.00;
  }
}
//========================================================================================================
/**********************************END_RESETE_MIN_AND_MAX*************************************************/



//========================================================================================================
/************************************SHOW_THERMISTOR_01***************************************************/
void showTempThermistor1() {
  DateTime now = rtc.now();
  tft.fillScreen(TFT_BLACK);

  tft.setCursor(5, 20);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFreeFont(FF18);
  tft.print(now.hour(), DEC);
  tft.print(F(":"));
  String minute = String(now.minute());
  int min = minute.toInt();
  if (min < 10) {
    tft.print(F("0"));
    tft.print(now.minute());
  } else {
    tft.print(now.minute());
  }
  tft.print(F("h "));

  tft.print(days[now.dayOfTheWeek()]);

  tft.print(F("  "));
  tft.print(now.day(), DEC);
  tft.print(F("/"));
  tft.print(now.month(), DEC);
  tft.print(F("/"));
  tft.print(now.year(), DEC);

  tft.setCursor(70, 60);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("TEMPERATURA"));

  tft.setCursor(105, 85);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F(" FM-001 "));

  readTermistor(ADC1_CHANNEL_6, 34);

  int xpos = 20, ypos = 120, radius = 140;                                       // Posição do medidor gauge analógico no display
  ringMeter(tempCelsius1, -25.00, 25, xpos, ypos, radius, "TI-001", GREEN2RED);  // Função que exibe o medidor analógico no display

  tft.setCursor(10, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MIN: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMin1);

  tft.setCursor(170, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MAX: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMax1);
}
//========================================================================================================
/**********************************END_SHOW_THERMISTOR_01*************************************************/

//========================================================================================================
/************************************SHOW_THERMISTOR_02***************************************************/
void showTempThermistor2() {
  DateTime now = rtc.now();
  tft.fillScreen(TFT_BLACK);

  tft.setCursor(5, 20);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFreeFont(FF18);
  tft.print(now.hour(), DEC);
  tft.print(F(":"));
  String minute = String(now.minute());
  int min = minute.toInt();
  if (min < 10) {
    tft.print(F("0"));
    tft.print(now.minute());
  } else {
    tft.print(now.minute());
  }
  tft.print(F("h "));

  tft.print(days[now.dayOfTheWeek()]);

  tft.print(F("  "));
  tft.print(now.day(), DEC);
  tft.print(F("/"));
  tft.print(now.month(), DEC);
  tft.print(F("/"));
  tft.print(now.year(), DEC);

  tft.setCursor(70, 60);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("TEMPERATURA"));

  tft.setCursor(105, 85);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F(" FM-002 "));

  readTermistor(ADC1_CHANNEL_7, 35);

  int xpos = 20, ypos = 120, radius = 140;                                       // Posição do medidor gauge analógico no display
  ringMeter(tempCelsius2, -25.00, 25, xpos, ypos, radius, "TI-002", GREEN2RED);  // Função que exibe o medidor analógico no display

  tft.setCursor(10, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MIN: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMin2);

  tft.setCursor(170, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MAX: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMax2);
}
//========================================================================================================
/**********************************END_SHOW_THERMISTOR_02*************************************************/


//========================================================================================================
/************************************SHOW_THERMISTOR_03***************************************************/
void showTempThermistor3() {
  DateTime now = rtc.now();
  tft.fillScreen(TFT_BLACK);

  tft.setCursor(5, 20);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFreeFont(FF18);
  tft.print(now.hour(), DEC);
  tft.print(F(":"));
  String minute = String(now.minute());
  int min = minute.toInt();
  if (min < 10) {
    tft.print(F("0"));
    tft.print(now.minute());
  } else {
    tft.print(now.minute());
  }
  tft.print(F("h "));

  tft.print(days[now.dayOfTheWeek()]);

  tft.print(F("  "));
  tft.print(now.day(), DEC);
  tft.print(F("/"));
  tft.print(now.month(), DEC);
  tft.print(F("/"));
  tft.print(now.year(), DEC);

  tft.setCursor(70, 60);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("TEMPERATURA"));

  tft.setCursor(105, 85);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("VASO-002"));

  readTermistor(ADC1_CHANNEL_4, 32);

  int xpos = 20, ypos = 120, radius = 140;                                       // Posição do medidor gauge analógico no display
  ringMeter(tempCelsius3, -25.00, 25, xpos, ypos, radius, "TI-003", GREEN2RED);  // Função que exibe o medidor analógico no display

  tft.setCursor(10, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MIN: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMin3);

  tft.setCursor(170, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MAX: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMax3);
}
//========================================================================================================
/**********************************END_SHOW_THERMISTOR_03*************************************************/


//========================================================================================================
/************************************SHOW_THERMISTOR_04***************************************************/
void showTempThermistor4() {
  DateTime now = rtc.now();
  tft.fillScreen(TFT_BLACK);

  tft.setCursor(5, 20);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFreeFont(FF18);
  tft.print(now.hour(), DEC);
  tft.print(F(":"));
  String minute = String(now.minute());
  int min = minute.toInt();
  if (min < 10) {
    tft.print(F("0"));
    tft.print(now.minute());
  } else {
    tft.print(now.minute());
  }
  tft.print(F("h "));

  tft.print(days[now.dayOfTheWeek()]);

  tft.print(F("  "));
  tft.print(now.day(), DEC);
  tft.print(F("/"));
  tft.print(now.month(), DEC);
  tft.print(F("/"));
  tft.print(now.year(), DEC);

  tft.setCursor(70, 60);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("TEMPERATURA"));

  tft.setCursor(105, 85);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("VASO-004"));

  readTermistor(ADC1_CHANNEL_5, 33);

  int xpos = 20, ypos = 120, radius = 140;                                       // Posição do medidor gauge analógico no display
  ringMeter(tempCelsius4, -25.00, 25, xpos, ypos, radius, "TI-004", GREEN2RED);  // Função que exibe o medidor analógico no display

  tft.setCursor(10, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MIN: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMin4);

  tft.setCursor(170, 470);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("MAX: "));
  tft.setTextColor(TFT_WHITE);
  tft.print(tempCelsiusMax4);
}
//========================================================================================================
/**********************************END_SHOW_THERMISTOR_03*************************************************/

//========================================================================================================
/************************************SAVE_SETUP_SD_CARD***************************************************/
void writeFile(fs::FS &fs, const char *path, const char *message) {
  DateTime now = rtc.now();
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println(F("Failed to open file for writing"));
    saveSDCardNotOk++;
    return;
  }
  if (file.print(message)) {
    file.print(F(" DATA: "));
    file.print(now.day(), DEC);
    file.print(F("/"));
    file.print(now.month(), DEC);
    file.print(F("/"));
    file.print(now.year(), DEC);
    file.print(F(";"));
    file.print(F(" HORARIO: "));
    file.print(now.hour(), DEC);
    file.print(F(":"));
    file.print(now.minute(), DEC);
    file.print(F(":"));
    file.print(now.second(), DEC);
    file.print(F(";"));
    file.println(F(" "));
    Serial.println(F("File written"));
    saveSDCardOk++;
  } else {
    Serial.println(F("Write failed"));
    saveSDCardNotOk++;
  }
  file.close();
}
/**********************************END_SAVE_SETUP_SD_CARD*************************************************/



//========================================================================================================
/***************************************SAVE_JSON********************************************************/
void writeFileJson(fs::FS &fs, const char *path, const char *message) {
  DateTime now = rtc.now();
  Serial.printf("Writing file: %s\n", path);

  if (!SD.begin(5)) {
    Serial.print(F("SD CARD em Falha!"));
  } else {
    Serial.print(F("SD CARD Ok!"));
  }

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println(F("Failed to open file for writing"));
    saveSDCardNotOk++;
    return;
  }
  if (file.print(message)) {
    file.print(F(" DATA: "));
    file.print(now.day(), DEC);
    file.print(F("/"));
    file.print(now.month(), DEC);
    file.print(F("/"));
    file.print(now.year(), DEC);
    file.print(F(";"));
    file.print(F(" HORARIO: "));
    file.print(now.hour(), DEC);
    file.print(F(":"));
    file.print(now.minute(), DEC);
    file.print(F(":"));
    file.print(now.second(), DEC);
    file.print(F(";"));
    file.print(F(" JSON: "));
    file.print(jsonData);
    file.println(F(" "));
    Serial.println(F("File written"));
    saveSDCardOk++;
  } else {
    Serial.println(F("Write failed"));
    saveSDCardNotOk++;
  }
  file.close();
}
/******************************************END_SAVE_JSON*************************************************/


//========================================================================================================
/************************************READ_TERMISTOR*******************************************************/
void readTermistor(adc1_channel_t channel, int thermistorPin) {
  uint32_t AD = 0;
  for (int i = 0; i < 100; i++) {
    AD += adc1_get_raw(channel);  //Obtem o valor RAW do ADC
    ets_delay_us(30);
  }
  AD /= 100;

  double vOut, Rt = 0;
  double tempKelvin = 0;
  double adc = 0;

  adc = analogRead(thermistorPin);
  adc = AD;
  vOut = adc * vccThermistor / adcMax;
  Rt = R1 * vOut / (vccThermistor - vOut);
  tempKelvin = 1 / (1 / To + log(Rt / Ro) / Beta);  //  Kelvin
  tempCelsius = tempKelvin - 273.15;                // Celsius

  if (channel == 4) {
    tempCelsius3 = tempCelsius - offsetTemp3;
    data3 = int32_t(round(tempCelsius3));
    Serial.print(F("Temperatura Celsius TI-003: "));
    Serial.println(tempCelsius3);
    Serial.print(F("Temperatura Celsius TI-003 Round: "));
    Serial.println(data3);
    if (tempCelsius3 < tempCelsiusMin3) {
      tempCelsiusMin3 = tempCelsius3;
    }
    if (tempCelsius3 > tempCelsiusMax3) {
      tempCelsiusMax3 = tempCelsius3;
    }
  }

  if (channel == 5) {
    tempCelsius4 = tempCelsius + offsetTemp4;
    data4 = int32_t(round(tempCelsius4));
    Serial.print(F("Temperatura Celsius TI-004: "));
    Serial.println(tempCelsius4);
    Serial.print(F("Temperatura Celsius TI-004 Round: "));
    Serial.println(data3);
    if (tempCelsius4 < tempCelsiusMin4) {
      tempCelsiusMin4 = tempCelsius4;
    }
    if (tempCelsius4 > tempCelsiusMax4) {
      tempCelsiusMax4 = tempCelsius4;
    }
  }

  if (channel == 6) {
    tempCelsius1 = tempCelsius + offsetTemp1;
    data1 = int32_t(round(tempCelsius1));
    Serial.print(F("Temperatura Celsius TI-001: "));
    Serial.println(tempCelsius1);
    Serial.print(F("Temperatura Celsius TI-001 Round: "));
    Serial.println(data1);
    if (tempCelsius1 < tempCelsiusMin1) {
      tempCelsiusMin1 = tempCelsius1;
    }
    if (tempCelsius1 > tempCelsiusMax1) {
      tempCelsiusMax1 = tempCelsius1;
    }
  }

  if (channel == 7) {
    tempCelsius2 = tempCelsius + offsetTemp2;
    data2 = int32_t(round(tempCelsius2));
    Serial.print(F("Temperatura Celsius TI-002: "));
    Serial.println(tempCelsius2);
    Serial.print(F("Temperatura Celsius TI-002 Round: "));
    Serial.println(data2);
    if (tempCelsius2 < tempCelsiusMin2) {
      tempCelsiusMin2 = tempCelsius2;
    }
    if (tempCelsius2 > tempCelsiusMax2) {
      tempCelsiusMax2 = tempCelsius2;
    }
  }
}
/**********************************END_READ_TERMISTOR*****************************************************/


//========================================================================================================
/********************************CONFIG_DISPLAY_SETUP*****************************************************/
void startDisplayScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(2);
  for (int i = 0; i < 10000; i++) {
    tft.drawPixel(random(320), random(480), random(0xFFFF));
  }
  delay(2000);
  tft.fillRoundRect(60, 60, 200, 200, 15, TFT_YELLOW);
  delay(1000);
  tft.setCursor(90, 145);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.setFreeFont(FF24);
  tft.print(F("NANO"));
  delay(1000);
  tft.setCursor(75, 205);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.setFreeFont(FF24);
  tft.print(F("SMART"));
  delay(1000);

  tft.setCursor(80, 475);
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(FF21);
  tft.print(F("Firmware: v1.0.1"));

  tft.setCursor(5, 396);
  tft.setTextColor(TFT_CYAN);
  tft.setFreeFont(FF22);
  tft.print(F("Iniciando configuracao:"));
  delay(250);
  digitalWrite(BUZZER_GND, false);  // Reseta o BUZZER
  digitalWrite(BUZZER_VCC, true);   // Seta o BUZZER
  tft.fillRect(0, 405, 20, 6, TFT_YELLOW);
  delay(250);
  tft.fillRect(0, 405, 40, 6, TFT_YELLOW);
  digitalWrite(BUZZER_VCC, false);  // Seta o BUZZER
  delay(250);

  tft.fillRect(0, 405, 60, 6, TFT_YELLOW);
  digitalWrite(BUZZER_VCC, true);  // Reseta o BUZZER
  delay(250);

  tft.fillRect(0, 405, 80, 6, TFT_YELLOW);
  digitalWrite(BUZZER_VCC, false);  // Seta o BUZZER
  delay(250);

  tft.fillRect(0, 405, 120, 6, TFT_YELLOW);
  digitalWrite(BUZZER_VCC, true);  // Reseta o BUZZER
  delay(250);

  tft.fillRect(0, 405, 160, 6, TFT_YELLOW);
  digitalWrite(BUZZER_VCC, false);  // Seta o BUZZER

  tft.fillRect(0, 405, 200, 6, TFT_YELLOW);
  delay(500);

  tft.fillRect(0, 405, 240, 6, TFT_YELLOW);
  delay(500);

  tft.fillRect(0, 405, 280, 6, TFT_YELLOW);
  delay(750);

  tft.fillRect(0, 405, 320, 6, TFT_YELLOW);
  delay(1000);
}
/*******************************END_CONFIG_DISPLAY_SETUP**************************************************/


//========================================================================================================
/************************************CONFIG_SETUP********************************************************/
void displaySetup() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRoundRect(0, 0, 320, 60, 15, TFT_YELLOW);
  delay(500);
  tft.setCursor(82, 45);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.setFreeFont(FF24);
  tft.print(F("SETUP"));
  delay(1000);

  tft.drawLine(0, 70, 320, 70, TFT_LIGHTGREY);

  tft.setCursor(5, 95);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("SD Card: "));
  if (!SD.begin(5)) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setFreeFont(FF18);
    tft.print(F("Falha"));
  } else {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setFreeFont(FF18);
    tft.print(F("Ok"));
  }

  tft.drawLine(0, 100, 320, 100, TFT_LIGHTGREY);

  uint8_t cardType = SD.cardType();
  tft.setCursor(5, 125);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("SD Card Type: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  if (cardType == CARD_NONE) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print(F("Nao Inserido"));
  } else {
    if (cardType == CARD_MMC) {
      tft.print(F("MMC"));
    } else if (cardType == CARD_SD) {
      tft.print(F("SDSC"));
    } else if (cardType == CARD_SDHC) {
      tft.print(F("SDHC"));
    } else {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.print(F("UNKNOWN"));
    }
  }

  tft.drawLine(0, 130, 320, 130, TFT_LIGHTGREY);

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  tft.setCursor(5, 155);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("SD Card Size: "));
  if (cardSize == 0) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
  }
  tft.setFreeFont(FF18);
  tft.print(cardSize);
  tft.print(F("MB"));

  tft.drawLine(0, 160, 320, 160, TFT_LIGHTGREY);

  uint64_t cardUsed = (SD.usedBytes() / (1024 * 1024));
  tft.setCursor(5, 185);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("SD Card Usado: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(cardUsed);
  tft.print(F("MB"));

  tft.drawLine(0, 190, 320, 190, TFT_LIGHTGREY);

  tft.setCursor(5, 215);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("MAC1: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(macAddress);

  tft.drawLine(0, 220, 320, 220, TFT_LIGHTGREY);

  tft.setCursor(5, 245);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("WIFI Sinal: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(RSSI);
  tft.print(F("dB"));
  if (RSSI >= -30) {
    tft.print(F(" (Excelente)"));
  } else if (RSSI < -30 && RSSI >= -55) {
    tft.print(F(" (Otimo)"));
  } else if (RSSI < -55 && RSSI >= -67) {
    tft.print(F(" (Muito Bom)"));
  } else if (RSSI < -67 && RSSI >= -70) {
    tft.print(F(" (Bom)"));
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print(F(" (Fraco)"));
  }

  tft.drawLine(0, 250, 320, 250, TFT_LIGHTGREY);

  tft.setCursor(5, 275);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("MAC2: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(BSSIDstr);

  tft.drawLine(0, 280, 320, 280, TFT_LIGHTGREY);

  tft.setCursor(5, 305);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("TTCALL Mode: "));
  tft.setFreeFont(FF18);
  if (SSID == "Slave_1") {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.print(SSID);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print(F("Inoperante"));
  }

  tft.drawLine(0, 310, 320, 310, TFT_LIGHTGREY);

  tft.setCursor(5, 335);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("ESP NOW Mode: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  if (isModeEspNowOk == true) {
    tft.print(F(" Ativado"));
  }

  tft.drawLine(0, 340, 320, 340, TFT_LIGHTGREY);
}
/**********************************END_CONFIG_SETUP******************************************************/


//========================================================================================================
/************************************INIT_ESP_NOW********************************************************/
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println(F("ESPNow Init Success"));
    isModeEspNowOk = true;
  } else {
    Serial.println(F("ESPNow Init Failed"));
    ESP.restart();
  }
}
/**********************************END_INIT_ESP_NOW******************************************************/


/**********************************SCAN_SLAVE_DEVICES******************************************************/
void scanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);  // Scan only on one channel
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println(F(""));
  if (scanResults == 0) {
    Serial.println(F("No WiFi devices in AP Mode found"));
  } else {
    Serial.print(F("Found "));
    Serial.print(scanResults);
    Serial.println(F(" devices "));
    for (int i = 0; i < scanResults; ++i) {
      SSID = WiFi.SSID(i);
      RSSI = WiFi.RSSI(i);
      BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(SSID);
        Serial.print(F(" ("));
        Serial.print(RSSI);
        Serial.print(F(")"));
        Serial.println(F(""));
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println(F("Found a Slave."));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(SSID);
        Serial.print(F(" ["));
        Serial.print(BSSIDstr);
        Serial.print(F("]"));
        Serial.print(F(" ("));
        Serial.print(RSSI);
        Serial.print(F(")"));
        Serial.println(F(""));
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
    Serial.println(F("Slave Found, processing.."));
  } else {
    Serial.println(F("Slave Not Found, trying again."));
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
    Serial.print(F("Slave Status: "));
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if (exists) {
      Serial.println(F("Already Paired"));
      return true;
    } else {
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        Serial.println(F("Pair success"));
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println(F("ESPNOW Not Init"));
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println(F("Invalid Argument"));
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println(F("Peer list full"));
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println(F("Out of memory"));
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println(F("Peer Exists"));
        return true;
      } else {
        Serial.println(F("Not sure what happened"));
        return false;
      }
    }
  } else {
    Serial.println(F("No Slave found to process"));
    return false;
  }
}
/***************************END_MENAGE_SLAVES_DEVICES****************************************************/


/******************************DELETE_PEER_DEVICE********************************************************/
void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print(F("Slave Delete Status: "));
  if (delStatus == ESP_OK) {
    Serial.println(F("Success"));
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println(F("ESPNOW Not Init"));
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println(F("Invalid Argument"));
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println(F("Peer not found."));
  } else {
    Serial.println(F("Not sure what happened"));
  }
}
/*******************************END_DELETE_PEER_DEVICE***************************************************/


/******************************SEND_DATA_TO_SLAVE_DEVICE**************************************************/
void sendData() {
  jsonData = "";
  doc["BOMB-001"] = !bombState;     //status de chiller ligado quando receber GND no pin16
  doc["CAMARA-001"] = camaraState;  //status de camara fria ligado quando receber GND no pin17
  doc["TI-001"] = tempCelsius1;
  doc["TI-002"] = tempCelsius2;
  doc["TI-003"] = tempCelsius3;
  doc["TI-004"] = tempCelsius4;

  size_t length = sizeof(jsonData) * 10;
  serializeJson(doc, jsonData);

  const uint8_t *peer_addr = slave.peer_addr;
  Serial.print("Sending: ");
  Serial.println(jsonData);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)jsonData.c_str(), length + 2);

  Serial.println(sizeof(jsonData));
  Serial.print(F("Send Status: "));
  if (result == ESP_OK) {
    Serial.println(F("Success"));
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println(F("ESPNOW not Init."));
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println(F("Invalid Argument"));
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println(F("Internal Error"));
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println(F("ESP_ERR_ESPNOW_NO_MEM"));
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println(F("Peer not found."));
  } else {
    Serial.println(F("Not sure what happened"));
  }
}
/****************************END_SEND_DATA_TO_SLAVE_DEVICE************************************************/


/**************************CALLBACK_SEND_DATA_TO_SLAVE_DEVICE*********************************************/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(F("Last Packet Sent to: "));
  Serial.println(macStr);
  Serial.print(F("Last Packet Send Status: "));
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS) {
    sendDataOk++;
  } else {
    sendDataNotOk++;
  }
}
/************************END_CALLBACK_SEND_DATA_TO_SLAVE_DEVICE*******************************************/


/**************************************DRAW_SD_IMAGE_JPEG*************************************************/
void drawSdJpeg(const char *filename, int xpos, int ypos) {
  if (!SD.begin(5)) {
    Serial.println(F("SD CARD em Falha!"));
  } else {
    Serial.println(F("SD CARD Ok!"));
  }
  File jpegFile = SD.open(filename, FILE_READ);  // or, file handle reference for SD library

  if (!jpegFile) {
    Serial.print(F("ERROR: File \""));
    Serial.print(filename);
    Serial.println(F("\" not found!"));
    return;
  }

  Serial.println(F("==========================="));
  Serial.print(F("Drawing file: "));
  Serial.println(filename);
  Serial.println(F("==========================="));

  bool decoded = JpegDec.decodeSdFile(jpegFile);  // Pass the SD file handle to the decoder,

  if (decoded) {
    jpegRender(xpos, ypos);
  } else {
    Serial.println(F("Jpeg file format not supported!"));
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

  if (value >= 10) {
    tft.setTextColor(colour, TFT_BLACK);
    tft.setCursor(x - 55, y + 25);
    tft.setFreeFont(FMB24);
    tft.setTextSize(2);
    tft.print(buf);
    tft.setTextSize(1);
  }
  if (value > 0 && value < 10) {
    tft.setTextColor(colour, TFT_BLACK);
    tft.setCursor(x - 82, y + 25);
    tft.setFreeFont(FMB24);
    tft.setTextSize(2);
    tft.print(buf);
    tft.setTextSize(1);
  }
  if (value < 0) {
    if (value < 0 && value > -1) {
      char buf2[10];  // convert value to a string
      dtostrf(0.00, 2, 0, buf2);
      tft.setTextColor(colour, TFT_BLACK);
      tft.setCursor(x - 82, y + 25);
      tft.setFreeFont(FMB24);
      tft.setTextSize(2);
      tft.print(buf2);
      tft.setTextSize(1);
    } else {
      tft.setTextColor(colour, TFT_BLACK);
      tft.setCursor(x - 75, y + 25);
      tft.setFreeFont(FMB24);
      tft.setTextSize(2);
      tft.print(buf);
      tft.setTextSize(1);
    }
  }

  tft.setCursor(130, 360);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF22);
  tft.print(F("O"));
  tft.setCursor(150, 380);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF20);
  tft.print(F("C"));

  tft.setCursor(80, 405);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF17);
  if (units == "TI-001") {
    tft.print(F("TI-001: -40C a +40C"));
  }
  if (units == "TI-002") {
    tft.print(F("TI-002: -40C a +40C"));
  }
  if (units == "TI-003") {
    tft.print(F("TI-003: -40C a +40C"));
  }
  if (units == "TI-004") {
    tft.print(F("TI-004: -40C a +40C"));
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
  tft.setRotation(2);
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


//========================================================================================================
/************************************TELA_DIAGNOSTICO*****************************************************/
void displayReport() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRoundRect(0, 0, 320, 60, 15, TFT_YELLOW);
  tft.setCursor(20, 45);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.setFreeFont(FF24);
  tft.print(F("RELATORIO"));

  tft.drawLine(0, 70, 320, 70, TFT_LIGHTGREY);

  tft.setCursor(5, 95);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("SD Card: "));
  if (!SD.begin(5)) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setFreeFont(FF18);
    tft.print(F("Falha"));
  } else {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setFreeFont(FF18);
    tft.print(F("Ok"));
  }

  tft.drawLine(0, 100, 320, 100, TFT_LIGHTGREY);

  tft.setCursor(5, 125);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("Dados SD Card OK: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(saveSDCardOk);

  tft.drawLine(0, 130, 320, 130, TFT_LIGHTGREY);

  tft.setCursor(5, 155);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("Dados SD Card Nao OK: "));
  if (saveSDCardNotOk == 0) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  tft.setFreeFont(FF18);
  tft.print(saveSDCardNotOk);

  tft.drawLine(0, 160, 320, 160, TFT_LIGHTGREY);

  uint64_t cardUsed = (SD.usedBytes() / (1024 * 1024));
  tft.setCursor(5, 185);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("SD Card Usado: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(cardUsed);
  tft.print(F("MB"));

  tft.drawLine(0, 190, 320, 190, TFT_LIGHTGREY);

  tft.setCursor(5, 215);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("MAC1: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(macAddress);

  tft.drawLine(0, 220, 320, 220, TFT_LIGHTGREY);

  tft.setCursor(5, 245);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("WIFI Sinal: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(RSSI);
  tft.print(F("dB"));
  if (RSSI >= -30) {
    tft.print(F(" (Excelente)"));
  } else if (RSSI < -30 && RSSI >= -55) {
    tft.print(F(" (Otimo)"));
  } else if (RSSI < -55 && RSSI >= -67) {
    tft.print(F(" (Muito Bom)"));
  } else if (RSSI < -67 && RSSI >= -70) {
    tft.print(F(" (Bom)"));
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print(F(" (Fraco)"));
  }

  tft.drawLine(0, 250, 320, 250, TFT_LIGHTGREY);

  tft.setCursor(5, 275);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("MAC2: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(BSSIDstr);

  tft.drawLine(0, 280, 320, 280, TFT_LIGHTGREY);

  tft.setCursor(5, 305);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("TTCALL Mode: "));
  tft.setFreeFont(FF18);
  if (SSID == "Slave_1") {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.print(SSID);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print(F("Inoperante"));
  }

  tft.drawLine(0, 310, 320, 310, TFT_LIGHTGREY);

  tft.setCursor(5, 335);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("ESP NOW Mode: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  if (isModeEspNowOk == true) {
    tft.print(F(" Ativado"));
  }

  tft.drawLine(0, 340, 320, 340, TFT_LIGHTGREY);

  tft.setCursor(5, 365);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("Send Dados OK: "));
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(sendDataOk);

  tft.drawLine(0, 370, 320, 370, TFT_LIGHTGREY);

  tft.setCursor(5, 395);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("Send Dados Nao OK: "));
  if (sendDataNotOk == 0) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  tft.setFreeFont(FF18);
  tft.print(sendDataNotOk);

  tft.drawLine(0, 400, 320, 400, TFT_LIGHTGREY);

  bombState = digitalRead(STATUS_CHILLER);
  tft.setCursor(5, 425);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("BOMBA: "));
  tft.setFreeFont(FF18);
  if (!bombState) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.print(F("Ligado"));
  } else {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.print(F("Desligado"));
  }

  tft.drawLine(0, 430, 320, 430, TFT_LIGHTGREY);

  camaraState = digitalRead(STATUS_CAMARA);
  tft.setCursor(5, 455);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF18);
  tft.print(F("CAMERA FRIA: "));
  tft.setFreeFont(FF18);
  if (camaraState) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.print(F("Ligado"));
  } else {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.print(F("Desligado"));
  }
  tft.drawLine(0, 460, 320, 460, TFT_LIGHTGREY);
}
/**********************************END_DIAGNOSTICO******************************************************/
