// temperatura para esp32 devkit v1
// uso com 2 resistores 10k
// youtube https://www.youtube.com/channel/UC1R99C9hNNVb9GBa89DVq2g
// by: osmar
 
 
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
 
esp_adc_cal_characteristics_t adc_cal; //Estrutura que contem as informacoes para calibracao
 
//temperatura
bool esp32 = true;       // mude para falso ao usar o Arduino
 
int ThermistorPin;
double adcMax, Vs;
 
String TempReal;
double R1 = 10000.0;   // resitor 10k
double Beta = 3950.0;  // Beta value
double To = 298.15;    // temperatura em kelvin Kelvin
double Ro = 10000.0;   // resitor 10k em C
 
//calibrando tenção esp
//const float ADC_LUT[4096] PROGMEM =
//////////////////
 
 
void setup() {
  Serial.begin(115200);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0); //pin 34 esp32 devkit v1
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal); //Inicializa a estrutura de calibracao
 
 
  ////////// temperatura
 
  ThermistorPin = 34;
  adcMax = 4095.0; // ADC 12-bit (0-4095)
  Vs = 3.3;        // voltagem
 
 
  ///////////
}
 
 
void loop() {
  uint32_t AD = 0;
  for (int i = 0; i < 100; i++)
  {
    AD += adc1_get_raw(ADC1_CHANNEL_6);//Obtem o valor RAW do ADC
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
 
  T = 1 / (1 / To + log(Rt / Ro) / Beta); //  Kelvin
  Tc = T - 273.15;                   // Celsius
  Tf = Tc * 9 / 5 + 32;              // Fahrenheit
  if (Tc > 0) Serial.print(Tc);
  Serial.println(" Celsius");
  delay(2000);
  if (Tf > 0) Serial.print(Tf);
  Serial.println(" Fahrenheit");
  delay(2000);
}