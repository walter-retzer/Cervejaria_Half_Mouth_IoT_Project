/**
   NANO SMART IOT

   Controlador: TCALL ESP32 - SIM800L
   Date: 07/09/2023
   Autor: Walter Dawid Retzer
   Comunicação: ESPNOW SLAVE
   Firmware: V1.0.0
   
   Note: Configuração de envio dos dados o servidor ThingSpeak;
         Configuração de inicialização do modem SIM800L;
         Configuração e estabilidade de conexão do modo GPRS do modem SIM800L;
*/

#define SIM800L_IP5306_VERSION_20200811
#include "utilities.h"         // Biblioteca de configuração inicial do módulo SIM800L
#define TINY_GSM_MODEM_SIM800  // Select your modem: SIM800L

#include <SoftwareSerial.h>  // Biblioteca para comunicação Serial entre o ESP32 e SIM800L
#include <TinyGsmClient.h>   // Biblioteca com funções do SIM800L
#include <PubSubClient.h>    // Biblioteca para auxiliar na comunicação com o Server ThingsPeak
#include <String.h>          // Biblioteca para uso de Strings
#include <time.h>            // Biblioteca para uso de variaveis de Tempo
#include <TimeLib.h>         // Biblioteca para uso de variaveis de Tempo
#include <esp_now.h>         // Biblioteca para uso do Esp Now comunications
#include <WiFi.h>            // Biblioteca para uso do WIFI
#include <ArduinoJson.h>     // Biblioteca para uso do formato Json

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
// Variavel de Configuração do Modo Slave do ESP NOW:
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
// Variaveis de dados recebidos via ESPNOW do ESP32 em modo Master:
String jsonData;
StaticJsonDocument<200> doc;
int chiller = 2;
int camara = 2;
int temp = 0;
int temp2 = 0;
int temp3 = 0;
int temp4 = 0;
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
#define STATUS_CHILLER 21
// ========================================================================================================




/****************************************SETUP***********************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println("ESPNOW CONFIG IN SLAVE");

  WiFi.mode(WIFI_AP);
  configDeviceAP();
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());

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
    ledState = !ledState;
    digitalWrite(LED_GPIO, ledState);
    auxTimeLed = millis();
  }
}
/**************************************END_LOOP**********************************************************/



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



/**********************************cONFIG_DEVICE_AP******************************************************/
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL ");
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

  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");
  Serial.println(jsonData);

  if (!error) {
    chiller = doc["CHILLER-001"];
    camara = doc["CAMARA-001"];
    temp = doc["TI-001"];
    temp2 = doc["TI-002"];
    temp3 = doc["TI-003"];
    temp4 = doc["TI-004"];

    Serial.print("Chiller1: ");
    Serial.println(chiller);

    Serial.print("Temp1: ");
    Serial.println(temp);

    Serial.print("Temp2: ");
    Serial.println(temp2);

    Serial.print("Temp3: ");
    Serial.println(temp3);

    Serial.print("Temp4: ");
    Serial.println(temp4);

    Serial.println("");
  }

  else {
    Serial.println("Falha na Deserialização!!");
    Serial.println(error.f_str());
    Serial.println("");
    return;
  }

  sendDataToServerIoT(server_T1, typeThingspeak, channelID, clientIdMQTT, mqttUserName, mqttPassword, "");

  bool isSimGprsConnected = modem.isGprsConnected();
  if (!isSimGprsConnected) {
    SerialMon.print("Habilitando conexão GPRS: ");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println("Sem conexão GPRS!!");
      delay(10000);
    } else {
      SerialMon.println("Conexão GPRS iniciada!");
    }
  }
}
/********************************END_ON_DATA_RECEV*******************************************************/



/*****************************SEND_DATA_TO_SERVER_IOT****************************************************/
void sendDataToServerIoT(char *serverIoT, char *serverType, long channelID, char *clienId, char *userName, char *password, char *topic) {
  SerialMon.print("Connecting to ");
  SerialMon.print(serverIoT);
  SerialMon.println(" ");

  client.connect(serverIoT, 1883);

  if (!client.connected()) {
    SerialMon.println("Servidor: Sem Conexão!");
  } else {
    SerialMon.println("Servidor: Status Conectado!");
  }

  if (!mqtt.connected()) {
    SerialMon.println("Conexão");
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
    mqtt.disconnect();
    delay(1000);
  }

  if (statusServer == 0) {
    SerialMon.println("Conectado");
    int sinal = modem.getSignalQuality();
    int status = digitalRead(STATUS_CHILLER);
    SerialMon.print("Chiller: ");
    SerialMon.println(status);
    float field1 = temp;
    float field2 = temp2;
    float field3 = temp3;
    float field4 = temp4;
    // float field5 = chiller;
    float field5 = !status;
    float field6 = camara;
    float field7 = sinal;

    if (serverType == typeThingspeak) {
      String data = String("field1=") + String(field1, 1) + "&field2=" + String(field2, 1) + "&field3=" + String(field3, 1) + "&field4=" + String(field4, 1) + "&field5=" + String(field5, 0) + "&field6=" + String(field6, 0) + "&field7=" + String(field7, 0);
      int length = data.length();
      const char *msgBuffer;
      msgBuffer = data.c_str();

      SerialMon.println("STRING:");
      SerialMon.println(data);

      String topicString = "channels/" + String(channelID) + "/publish";
      length = topicString.length();
      const char *topicBuffer;
      topicBuffer = topicString.c_str();

      bool status_publict1 = mqtt.publish(topicBuffer, msgBuffer);

      if (status_publict1) {
        SerialMon.println("Mensagem Publicada!");
        sendATCommand("AT+CIPCLOSE=0,1");
        mqtt.disconnect();
      }
    }
  }
}
/***************************END_SEND_DATA_TO_SERVER_IOT**************************************************/



/*********************************SEND_AT_COMMANDS*******************************************************/
String sendATCommand(String command) {

  String response = "Fail";
  SerialMon.print("SIM800L Send Command: ");
  SerialMon.println(command);
  SerialAT.println(command);

  delay(500);

  if (SerialAT.available()) {
    response = SerialAT.readString();
    response.trim();
    SerialMon.print("SIM800L Response: ");
    SerialMon.print(response);
    SerialMon.println("");
  }

  return response;
}
/*******************************END_SEND_AT_COMMANDS*****************************************************/



/*********************************SETUP_SIM_MODEM*******************************************************/
void setupSimModem() {
  setupModem();

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  SerialMon.println("Iniciando modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  SerialMon.println(" ");
  delay(3000);

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
  SerialMon.println(" ");
  delay(1000);

  SerialMon.print("Habilitando conexão com a internet: ");
  if (modem.isNetworkConnected()) {
    SerialMon.println("Internet Habilitada!");
  } else {
    SerialMon.println("Sem acesso à Internet!");
  }
  SerialMon.println(" ");
  delay(1000);

  SerialMon.print("Habilitando conexão GPRS: ");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("Sem conexão GPRS!!");
    delay(10000);
    return;
  } else {
    SerialMon.println("Conexão GPRS iniciada!");
  }
  SerialMon.println(" ");
  delay(1000);

  SerialMon.print("Status Conexão GPRS: ");
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected!");
  } else {
    SerialMon.println("GPRS not connected!");
  }
  SerialMon.println(" ");
  delay(1000);

  int sinal = modem.getSignalQuality();
  SerialMon.print("Sinal Quality: ");
  SerialMon.println(sinal);
  SerialMon.println(" ");
  delay(1000);

  IPAddress ip = modem.localIP();
  SerialMon.print("IP: ");
  SerialMon.println(ip);
  SerialMon.println(" ");
  delay(1000);
}
/*******************************END_SETUP_SIM_MODEM*****************************************************/
