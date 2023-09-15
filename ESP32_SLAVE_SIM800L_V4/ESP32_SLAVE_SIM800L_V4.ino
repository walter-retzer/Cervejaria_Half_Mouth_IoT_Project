/**
   NANO SMART IOT

   Controlador: TTCALL ESP32 - SIM800L
   Date: 15/09/2023
   Autor: Walter Dawid Retzer
   Tipo: ESPNOW SLAVE
   Firmware: V1.0.1
   
   Note: Inclusão do monitoramento da temperatura do fluído de resfriamento;
         Ajuste das variaveis enviadas ao ThingSpeak;
*/

#define SIM800L_IP5306_VERSION_20200811
#include "utilities.h"         // Biblioteca de configuração inicial do módulo SIM800L
#define TINY_GSM_MODEM_SIM800  // Select your modem: SIM800L

#include <SoftwareSerial.h>          // Biblioteca para comunicação Serial entre o ESP32 e SIM800L
#include <TinyGsmClient.h>           // Biblioteca com funções do SIM800L
#include <PubSubClient.h>            // Biblioteca para auxiliar na comunicação com o Server ThingsPeak
#include <String.h>                  // Biblioteca para uso de Strings
#include <time.h>                    // Biblioteca para uso de variaveis de Tempo
#include <TimeLib.h>                 // Biblioteca para uso de variaveis de Tempo
#include <esp_now.h>                 // Biblioteca para uso do Esp Now comunications
#include <WiFi.h>                    // Biblioteca para uso do WIFI
#include <ArduinoJson.h>             // Biblioteca para uso do formato Json
#include "soc/timer_group_struct.h"  // Biblioteca para uso do tempo para desabilitar o watchdog
#include "soc/timer_group_reg.h"     // Biblioteca para uso do tempo para desabilitar o watchdog
#include <esp_adc_cal.h>             // Biblioteca para realizar a leitura dos valores de temperatura dos sensores NTC


//========================================================================================================
// Variaveis de Configuração do Modem Sim800L:
#define SerialMon Serial          // Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialAT Serial1          // Set serial for AT commands (to the module)
#define TINY_GSM_DEBUG SerialMon  // Define the serial console for debug prints, if needed
#define TINY_GSM_USE_GPRS true    // Define how you're planning to connect to the internet
#define TINY_GSM_USE_WIFI false   // Define how you're planning to connect to the internet
#define GSM_PIN ""                // set GSM PIN, if any
#define TINY_GSM_RX_BUFFER 4096   // Set RX buffer to 1Kb
//========================================================================================================
// Variavel de Configuração do Modo Slave do ESP-NOW:
#define CHANNEL 1
//========================================================================================================
// Variaveis de Configuração de acesso a conexão GPRS do chip sim utilizado no modem SIM800L:
const char apn[] = "zap.vivo.com.br";
const char gprsUser[] = "";
const char gprsPass[] = "";
// ========================================================================================================
// Variaveis de Configuração de acesso ao Server THINGSPEAK:
char clientIdMQTT[] = "XXXXXXXXXXXXXXXXXXXXXXX";
char mqttUserName[] = "YYYYYYYYYYYYYYYYYYYYYYY";
char mqttPassword[] = "ZZZZZZZZZZZZZZZZZZZZZZZ";
char server_T1[] = "mqtt3.thingspeak.com";
long channelID = 9999999;
char typeThingspeak[] = "Thingspeak";
int statusServer;
// ========================================================================================================
// Variaveis de dados recebidos via ESP-NOW do ESP32 em modo Master:
String jsonData;
StaticJsonDocument<200> doc;
int chiller = 2;
int bomb = 2;
int camara = 2;
double temp = 0;
double temp2 = 0;
double temp3 = 0;
double temp4 = 0;
// ========================================================================================================
// Variaveis auxiliares do loop:
unsigned long intervalo = 250;
unsigned long auxTimeLed;
bool ledState = true;
// ========================================================================================================
// Variaveis auxiliares e funções de comunicação com o modem SIM800L:
String sendATCommand(String command);
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
// ========================================================================================================
// Variaveis auxiliares de comunicação do modem SIM800L com o Server ThingSpeak:
void sendDataToServerIoT();
TinyGsmClient client(modem);
PubSubClient mqtt(client);
// ========================================================================================================
// Variaveis auxiliares de status do chiller:
#define STATUS_CHILLER 2
#define GND_CHILLER 0
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
double offsetTemp4 = 9.0;   // valor de offset da temperatura 4
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



/****************************************SETUP***********************************************************/
void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println(" ");
  Serial.println("**************************************************************************************************************************************** ");
  Serial.println("NANO SMART: ESP-NOW CONFIG IN SLAVE");
  Serial.println("Firmware Version: 1.0.0");

  WiFi.mode(WIFI_AP);
  configSlaveDevice();
  Serial.print("Slave Address MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println("**************************************************************************************************************************************** ");
  Serial.println(" ");


  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);                                                              //pin 34 esp32 devkit v1
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);  //Inicializa a estrutura de calibracao

  adcMax = 4095.0;      // ADC 12-bit (0-4095)
  vccThermistor = 3.3;  // Alimentação VCC Termistor
  readTermistor(ADC1_CHANNEL_5, 33);
  tempCelsiusMax4 = -100.00;
  tempCelsiusMin4 = 100.00;

  pinMode(GND_CHILLER, OUTPUT);
  digitalWrite(GND_CHILLER, LOW);

  setupSimModem();

  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(STATUS_CHILLER, INPUT_PULLUP);  // Declara Pino do Status do CHILLER como entrada
  auxTimeLed = millis();
}
/**************************************END_SETUP*********************************************************/



/****************************************LOOP************************************************************/
void loop() {
  if (millis() - auxTimeLed > intervalo) {
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    ledState = !ledState;
    digitalWrite(LED_GPIO, ledState);
    auxTimeLed = millis();
  }
}
/**************************************END_LOOP**********************************************************/



/************************************INIT_ESP_NOW********************************************************/
void InitESPNow() {
  WiFi.disconnect();
  Serial.println(" ");
  Serial.println("**************************************************************************************************************************************** ");
  if (esp_now_init() == ESP_OK) {
    Serial.println("Comunicação ESP-NOW: Ativada com Sucesso");
  } else {
    Serial.println("Comunicação ESP-NOW: Em Falha!!");
    ESP.restart();
  }
  Serial.println("**************************************************************************************************************************************** ");
  Serial.println(" ");
}
/**********************************END_INIT_ESP_NOW******************************************************/



/*******************************CONFIG_SLAVE_DEVICE******************************************************/
void configSlaveDevice() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("Config Slave Device Failed!");
  } else {
    Serial.println("ESP-NOW Configuração: Modo Slave Device Configurada com Sucesso!");
    Serial.println("ESP-NOW Broadcasting ID Address: " + String(SSID));
    Serial.print("ESP-NOW Slave Device Channel: ");
    Serial.println(WiFi.channel());
  }
}
/********************************END_CONFIG_DEVICE_AP****************************************************/


/***********************************ON_DATA_RECEV********************************************************/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  char *buff = (char *)data;
  jsonData = String(buff);
  DeserializationError error = deserializeJson(doc, jsonData);

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.println(" ");
  Serial.println("**************************************************************************************************************************************** ");
  Serial.println("ESP-NOW: Comunicação Recebida com Sucesso!");
  Serial.print("ESP-NOW: Envio de Dados do ESP-32 MAC Address=");
  Serial.println(macStr);
  Serial.print("ESP-NOW: Dados Recebidos=");
  Serial.println(jsonData);

  if (!error) {
    bomb = doc["BOMB-001"];
    camara = doc["CAMARA-001"];
    temp = doc["TI-001"];
    temp2 = doc["TI-002"];
    temp3 = doc["TI-003"];
    temp4 = doc["TI-004"];

    Serial.print("Status Chiller: ");
    Serial.println(chiller);

    Serial.print("Temp1: ");
    Serial.println(temp);

    Serial.print("Temp2: ");
    Serial.println(temp2);

    Serial.print("Temp3: ");
    Serial.println(temp3);

    Serial.print("Temp4: ");
    Serial.println(temp4);

    Serial.println("**************************************************************************************************************************************** ");
    SerialMon.println(" ");
  }

  else {
    Serial.println("Falha na Deserialização!!");
    Serial.println(error.f_str());
    Serial.println("**************************************************************************************************************************************** ");
    SerialMon.println(" ");
    return;
  }

  sendDataToServerIoT(server_T1, typeThingspeak, channelID, clientIdMQTT, mqttUserName, mqttPassword, "");

  bool isSimGprsConnected = modem.isGprsConnected();
  if (!isSimGprsConnected) {

    SerialMon.println(" ");
    Serial.println("**************************************************************************************************************************************** ");
    SerialMon.print("Habilitando conexão GPRS: ");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println("Sem conexão GPRS!!");
      delay(10000);
      Serial.println("**************************************************************************************************************************************** ");
      SerialMon.println(" ");
    } else {
      SerialMon.println("Conexão GPRS iniciada!");
      Serial.println("**************************************************************************************************************************************** ");
      SerialMon.println(" ");
    }
  }
}
/********************************END_ON_DATA_RECEV*******************************************************/



/*****************************SEND_DATA_TO_SERVER_IOT****************************************************/
void sendDataToServerIoT(char *serverIoT, char *serverType, long channelID, char *clienId, char *userName, char *password, char *topic) {
  SerialMon.println(" ");
  Serial.println("****************************************************************************************************************************************");
  SerialMon.print("Connectando ao Server: ");
  SerialMon.println(serverIoT);

  client.connect(serverIoT, 1883);

  if (!client.connected()) {
    SerialMon.println("Status Servidor: Sem Conexão!");
  } else {
    SerialMon.println("Status Servidor: Conectado!");
  }

  if (!mqtt.connected()) {
    SerialMon.print("Conexão MQTT: ");
    if (!!!mqtt.connect(clienId, userName, password)) {

      statusServer = mqtt.state();
      SerialMon.println(statusServer);

      SerialMon.println("ERROR MQTT SERVER: ");
      if (statusServer == -4) {
        SerialMon.println("CONNECTION_TIMEOUT");
      }
      if (statusServer == -3) {
        SerialMon.println("CONNECTION_LOST");
      }
      if (statusServer == -2) {
        SerialMon.println("CONNECTION_FAILED");
      }
      if (statusServer == -1) {
        SerialMon.println("MQTT_DISCONNECTED");
      }
      if (statusServer == 1) {
        SerialMon.println("BAD_PROTOCOL");
      }
      if (statusServer == 2) {
        SerialMon.println("BAD_CLIENT_ID");
      }
      if (statusServer == 3) {
        SerialMon.println("CONNECT_UNAVAILABLE");
      }
      if (statusServer == 4) {
        SerialMon.println("CONNECT_BAD_CREDENTIALS");
      }
      if (statusServer == 5) {
        SerialMon.println("CONNECT_UNAUTHORIZED");
      }
    }
  }

  if (statusServer != 0) {
    Serial.println("Falha em Conectar com o Server ThingSpeak!!");
    ESP.restart();
  }

  if (statusServer == 0) {
    SerialMon.println("CONNECTION AUTHORIZED");
    int sinal = modem.getSignalQuality();
    int status = digitalRead(STATUS_CHILLER);
    SerialMon.print("Status Chiller: ");
    SerialMon.println(!status);
    float field1 = temp;
    float field2 = temp2;
    float field3 = temp3;
    float field4 = temp4;
    float field5 = tempCelsius4;
    float field6 = camara;
    float field7 = bomb;
    float field8 = !status;

    if (serverType == typeThingspeak) {
      String data = String("field1=") + String(field1, 2) + "&field2=" + String(field2, 2) + "&field3=" + String(field3, 2) + "&field4=" + String(field4, 2) + "&field5=" + String(field5, 2) + "&field6=" + String(field6, 0) + "&field7=" + String(field7, 0) + "&field8=" + String(field8, 0);
      int length = data.length();
      const char *msgBuffer;
      msgBuffer = data.c_str();

      SerialMon.print("String JSON Enviada: ");
      SerialMon.println(data);

      String topicString = "channels/" + String(channelID) + "/publish";
      length = topicString.length();
      const char *topicBuffer;
      topicBuffer = topicString.c_str();

      bool status_publict1 = mqtt.publish(topicBuffer, msgBuffer);

      if (status_publict1) {
        SerialMon.println("Mensagem JSON: Valores Publicados no Server ThingSpeak!");
        Serial.println("**************************************************************************************************************************************** ");
        SerialMon.println(" ");
        sendATCommand("AT+CIPCLOSE=0,1");
        mqtt.disconnect();
      } else {
        SerialMon.println("Mensagem JSON: Valores Não Publicados no Server ThingSpeak!!");
        Serial.println("**************************************************************************************************************************************** ");
        SerialMon.println(" ");
        sendATCommand("AT+CIPCLOSE=0,1");
        mqtt.disconnect();
      }
      readTermistor(ADC1_CHANNEL_5, 33);
    }
  }
}
/***************************END_SEND_DATA_TO_SERVER_IOT**************************************************/



/*********************************SEND_AT_COMMANDS*******************************************************/
String sendATCommand(String command) {

  String response = "Fail";
  SerialMon.println(" ");
  Serial.println("**************************************************************************************************************************************** ");
  SerialMon.print("SIM800L Send Command: ");
  SerialMon.println(command);
  SerialAT.println(command);

  delay(500);

  if (SerialAT.available()) {
    response = SerialAT.readString();
    response.trim();
    SerialMon.print("SIM800L Response: ");
    SerialMon.print(response);
    SerialMon.println(" ");
    Serial.println("**************************************************************************************************************************************** ");
    SerialMon.println(" ");
  }

  return response;
}
/*******************************END_SEND_AT_COMMANDS*****************************************************/



/*********************************SETUP_SIM_MODEM*******************************************************/
void setupSimModem() {
  setupModem();

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  Serial.println(" ");
  Serial.println("**************************************************************************************************************************************** ");
  SerialMon.println("Iniciando SIM800L...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("SIM800L Info: ");
  SerialMon.println(modemInfo);
  delay(1000);

// Unlock your SIM card with a PIN if needed
#if TINY_GSM_USE_GPRS
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }
#endif

  SerialMon.print("Conectando com a Rede GSM: ");
  if (!modem.waitForNetwork()) {
    SerialMon.println("Chip não Conectado!!");
    delay(10000);
    return;
  } else {
    SerialMon.println("Chip conectado na rede GSM da VIVO!");
  }
  delay(1000);

  SerialMon.print("Habilitando conexão com a internet: ");
  if (modem.isNetworkConnected()) {
    SerialMon.println("Internet Habilitada!");
  } else {
    SerialMon.println("Sem acesso à Internet!");
  }
  delay(1000);

  SerialMon.print("Ativando conexão GPRS: ");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("Sem conexão GPRS!!");
    delay(10000);
    return;
  } else {
    SerialMon.println("Conexão GPRS Ativada!");
  }
  delay(1000);

  SerialMon.print("Status Conexão GPRS: ");
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS Connected!");
  } else {
    SerialMon.println("GPRS Not Connected!");
  }
  delay(1000);

  int sinal = modem.getSignalQuality();
  SerialMon.print("Sinal GSM Quality: ");
  SerialMon.print(sinal);
  SerialMon.println("dB");
  delay(1000);

  IPAddress ip = modem.localIP();
  SerialMon.print("IP: ");
  SerialMon.println(ip);
  Serial.println("**************************************************************************************************************************************** ");
  SerialMon.println(" ");
  delay(1000);

  sendATCommand("AT+GSMBUSY=1");
  delay(1000);
}
/*******************************END_SETUP_SIM_MODEM*****************************************************/

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

  SerialMon.println(" ");
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
  SerialMon.println(" ");
}
/**********************************END_READ_TERMISTOR*****************************************************/
