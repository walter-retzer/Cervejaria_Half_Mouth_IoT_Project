/**
   NANO SMART IOT

   Controlador: TTCALL ESP32 - WIFI COMUNICATIONS
   Date: 22/09/2023
   Autor: Walter Dawid Retzer
   Tipo: ESPNOW SLAVE
   Firmware: V0.0.1
   
   Note: Ajuste do código para utilizar o WIFI para envio dos dados ao ThingSpeak;
*/

#include <String.h>                  // Biblioteca para uso de Strings
#include <time.h>                    // Biblioteca para uso de variaveis de Tempo
#include <TimeLib.h>                 // Biblioteca para uso de variaveis de Tempo
#include <WiFi.h>                    // Biblioteca para uso do WIFI
#include "ThingSpeak.h"              // Biblioteca para uso do ThingSpeak
#include "soc/timer_group_struct.h"  // Biblioteca para uso do tempo para desabilitar o watchdog
#include "soc/timer_group_reg.h"     // Biblioteca para uso do tempo para desabilitar o watchdog
#include <esp_adc_cal.h>             // Biblioteca para calibração das temperaturas

// ========================================================================================================
// Variaveis de Configuração de acesso ao WiFi:
const char *ssid = "XXXXXX";          // your network SSID (name)
const char *password = "YYYYYYYYYY";  // your network password
WiFiClient client;
// ========================================================================================================
// Variaveis de Configuração de acesso ao Server THINGSPEAK:
const char *myWriteAPIKey = "ZZZZZZZZZZZZ";
long myChannelNumber = 9999999;
// ========================================================================================================
// Variaveis auxiliares do loop:
unsigned long intervalo = 250;
unsigned long auxTimeLed;
bool ledState = true;
unsigned long auxTimeLoop1;
unsigned long intervaloLoop1 = 360000;
unsigned long lastTime = 0;
unsigned long timerDelay = 300000;
// ========================================================================================================
// Variaveis auxiliares de status do chiller:
#define STATUS_CHILLER 2
#define GND_CHILLER 0
#define LED_GPIO 13
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
double offsetTemp4 = 10.5;  // valor de offset da temperatura 4
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
float tempTest = 25.2;
//========================================================================================================



/****************************************SETUP***********************************************************/
void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println(" ");
  Serial.println("**************************************************************************************************************************************** ");
  Serial.println("NANO SMART: ESP-TCALL");
  Serial.println("Firmware Version: 0.0.1");

  adc1_config_width(ADC_WIDTH_BIT_12);
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);  //Inicializa a estrutura de calibracao

  adcMax = 4095.0;      // ADC 12-bit (0-4095)
  vccThermistor = 3.25;  // Alimentação VCC Termistor
  readTermistor(ADC1_CHANNEL_5, 33);
  tempCelsiusMax4 = -100.00;
  tempCelsiusMin4 = 100.00;

  pinMode(GND_CHILLER, OUTPUT);
  digitalWrite(GND_CHILLER, LOW);

  setupWiFi();
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  pinMode(STATUS_CHILLER, INPUT_PULLUP);  // Declara Pino do Status do CHILLER como entrada
  pinMode(LED_GPIO, OUTPUT);              // Led placa
  digitalWrite(LED_GPIO, true);
  delay(5000);
  auxTimeLed = millis();
  auxTimeLoop1 = millis();
}
/**************************************END_SETUP*********************************************************/



/****************************************LOOP************************************************************/
void loop() {
  if (millis() - auxTimeLed > intervalo) {
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    ledState = !ledState;
    auxTimeLed = millis();
  }

  if ((millis() - lastTime) > timerDelay) {

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(" ");
      Serial.println("**************************************************************************************************************************************** ");
      Serial.print("[WiFi] - Reconectando WiFi");
      Serial.println(" ");
      Serial.println("**************************************************************************************************************************************** ");
      setupWiFi();
    }

    readTermistor(ADC1_CHANNEL_5, 33);
    Serial.print("Temperatura Fluido Recirculação TI-005: ");
    Serial.println(tempCelsius4);

    int status = digitalRead(STATUS_CHILLER);
    Serial.print("Status Chiller: ");
    Serial.println(!status);

    float field5 = tempCelsius4;
    float field8 = !status;

    ThingSpeak.setField(5, field5);
    ThingSpeak.setField(8, field8);

    int send = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    if (send == 200) {
      Serial.println(" ");
      Serial.println("**************************************************************************************************************************************** ");
      Serial.println("Mensagem JSON[SEND]:  Valores Publicados no Server ThingSpeak!!");
      Serial.println("**************************************************************************************************************************************** ");
      Serial.println(" ");
    } else {
      Serial.println(" ");
      Serial.println("**************************************************************************************************************************************** ");
      Serial.println("Mensagem JSON[SEND]: Falha ao Enviar ao Server ThingSpeak!!");
      Serial.println("Problema ao enviar os dados. HTTP error code: " + String(send));
      Serial.println("**************************************************************************************************************************************** ");
      Serial.println(" ");
      delay(20000);
      int resend = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if (resend == 200) {
        Serial.println(" ");
        Serial.println("**************************************************************************************************************************************** ");
        Serial.println("Mensagem JSON: resend Valores Publicados no Server ThingSpeak!!");
        Serial.println("**************************************************************************************************************************************** ");
        Serial.println(" ");
      } else {
        Serial.println(" ");
        Serial.println("**************************************************************************************************************************************** ");
        Serial.println("Mensagem JSON:resend Valores Não Publicados no Server ThingSpeak!!");
        Serial.println("Problema ao enviar od dados. HTTP error code: " + String(resend));
        Serial.println("**************************************************************************************************************************************** ");
        Serial.println(" ");
      }
    }
    lastTime = millis();
  }
}
/**************************************END_LOOP**********************************************************/



/************************************READ_TERMISTOR*******************************************************/
void readTermistor(adc1_channel_t channel, int thermistorPin) {
  uint32_t AD = 0;
  for (int i = 0; i < 1000; i++) {
    AD += adc1_get_raw(channel);  //Obtem o valor RAW do ADC
    ets_delay_us(30);
  }
  AD /= 1000;

  double vOut, Rt = 0;
  double tempKelvin = 0;
  double adc = 0;

  adc = analogRead(thermistorPin);
  adc = AD;
  vOut = adc * vccThermistor / adcMax;
  Rt = R1 * vOut / (vccThermistor - vOut);
  tempKelvin = 1 / (1 / To + log(Rt / Ro) / Beta);  //  Kelvin
  tempCelsius = tempKelvin - 273.15;                // Celsius

  Serial.println(" ");
  Serial.println("**************************************************************************************************************************************** ");

  if (channel == 4) {
    tempCelsius3 = tempCelsius - offsetTemp3;
    Serial.print(F("Temperatura Celsius TI-003: "));
    Serial.println(tempCelsius3);
    if (tempCelsius3 < tempCelsiusMin3) {
      tempCelsiusMin3 = tempCelsius3;
    }
    if (tempCelsius3 > tempCelsiusMax3) {
      tempCelsiusMax3 = tempCelsius3;
    }
  }

  if (channel == 5) {
    tempCelsius4 = tempCelsius + offsetTemp4;
    Serial.print(F("Temperatura Celsius TI-004: "));
    Serial.println(tempCelsius4);
    if (tempCelsius4 < tempCelsiusMin4) {
      tempCelsiusMin4 = tempCelsius4;
    }
    if (tempCelsius4 > tempCelsiusMax4) {
      tempCelsiusMax4 = tempCelsius4;
    }
  }

  if (channel == 6) {
    tempCelsius1 = tempCelsius + offsetTemp1;
    Serial.print(F("Temperatura Celsius TI-001: "));
    Serial.println(tempCelsius1);
    if (tempCelsius1 < tempCelsiusMin1) {
      tempCelsiusMin1 = tempCelsius1;
    }
    if (tempCelsius1 > tempCelsiusMax1) {
      tempCelsiusMax1 = tempCelsius1;
    }
  }

  if (channel == 7) {
    tempCelsius2 = tempCelsius + offsetTemp2;
    Serial.print(F("Temperatura Celsius TI-002: "));
    Serial.println(tempCelsius2);
    if (tempCelsius2 < tempCelsiusMin2) {
      tempCelsiusMin2 = tempCelsius2;
    }
    if (tempCelsius2 > tempCelsiusMax2) {
      tempCelsiusMax2 = tempCelsius2;
    }
  }
  Serial.println("**************************************************************************************************************************************** ");
  Serial.println(" ");
}
/**********************************END_READ_TERMISTOR*****************************************************/


/***************************************SETUP_WIFI********************************************************/
void setupWiFi() {
  Serial.println();
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  int tryDelay = 1000;
  int numberOfTries = 20;

  // Wait for the WiFi event
  while (true) {

    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:
        Serial.println("[WiFi] SSID not found");
        break;
      case WL_CONNECT_FAILED:
        Serial.print("[WiFi] Failed - WiFi not connected! Reason: ");
        return;
        break;
      case WL_CONNECTION_LOST:
        Serial.println("[WiFi] Connection was lost");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("[WiFi] Scan is completed");
        break;
      case WL_DISCONNECTED:
        Serial.println("[WiFi] WiFi is disconnected");
        break;
      case WL_CONNECTED:
        Serial.println("[WiFi] WiFi is connected!");
        Serial.print("[WiFi] IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("[WiFi] Sinal: ");
        Serial.println(WiFi.RSSI());
        return;
        break;
      default:
        Serial.print("[WiFi] WiFi Status: ");
        Serial.println(WiFi.status());
        break;
    }
    delay(tryDelay);

    if (numberOfTries <= 0) {
      Serial.print("[WiFi] Failed to connect to WiFi!");
      WiFi.disconnect();
      ESP.restart();
      return;
    } else {
      numberOfTries--;
    }
  }
}
/***************************************END_SETUP_WIFI********************************************************/
