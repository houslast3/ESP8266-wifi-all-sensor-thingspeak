/*
 * Sistema Universal de Sensores para ESP8266
 * Suporta múltiplos sensores com detecção automática
 * Interface Web com Dashboard, Configuração WiFi, Calibração, ThingSpeak e Alarmes
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>

// Bibliotecas dos Sensores I2C
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <BH1750.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_INA219.h>

// Biblioteca TinyExpr para cálculos
#include "tinyexpr.h"

// Display
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, D6, D5, U8X8_PIN_NONE);

// Servidor Web
ESP8266WebServer server(80);

// Pinos
#define BME_SDA D2
#define BME_SCL D1
#define DHT_PIN D3
#define ALARM_PIN D0
// A0 é usado para reset - não precisa definir pino

// Constantes
#define MAX_WIFI_NETWORKS 5
#define MAX_THINGSPEAK_APIS 5
#define MAX_SENSORS 10
#define MAX_FORMULAS 5
#define MAX_VARIABLES 10
#define EEPROM_SIZE 4096  // Aumentado de 2048 para 4096
#define AP_SSID "ESP8266_SENSOR_HUB"
#define AP_PASS "12345678"
#define EEPROM_MAGIC 0xABCD1234  // Número mágico para validar EEPROM

// Estruturas de Dados
struct WiFiNetwork {
  char ssid[32];
  char password[64];
  bool active;
};

struct SensorData {
  char name[32];
  char type[32];
  uint8_t address;
  bool active;
  float value1, value2, value3, value4;
  char label1[16], label2[16], label3[16], label4[16];
  float calibOffset1, calibOffset2, calibOffset3, calibOffset4;
  float calibScale1, calibScale2, calibScale3, calibScale4;
};

struct ThingSpeakConfig {
  char apiKey[32];
  int sensorIndex[8];
  int valueIndex[8];
  unsigned long interval;
  bool active;
};

struct DisplayConfig {
  int sensor1, value1;
  int sensor2, value2;
  int sensor3, value3;
};

struct AlarmConfig {
  int sensorIndex;
  int valueIndex;
  float threshold;
  bool above;
  bool useLED;
  bool useBuzzer;
  bool active;
};

struct FormulaConfig {
  char formula[128];
  bool active;
};

struct VariableConfig {
  float value;
};

// Variáveis Globais
WiFiNetwork wifiNetworks[MAX_WIFI_NETWORKS];
SensorData sensors[MAX_SENSORS];
ThingSpeakConfig thingspeakAPIs[MAX_THINGSPEAK_APIS];
DisplayConfig displayConfig = {0, 0, -1, 0, -1, 0};
AlarmConfig alarmConfig;
FormulaConfig formulas[MAX_FORMULAS];
VariableConfig variables[MAX_VARIABLES];
float calculatedValues[MAX_FORMULAS] = {0, 0, 0, 0, 0}; // C1, C2, C3, C4, C5
int sensorCount = 0;
bool apMode = true;
unsigned long lastThingSpeakUpdate[MAX_THINGSPEAK_APIS] = {0};
unsigned long lastAlarmBlink = 0;
int alarmBlinkCount = 0;
bool alarmState = false;
bool simulatedSensorEnabled = false;
unsigned long lastSimulatedUpdate = 0;

// Objetos dos Sensores
Adafruit_BME280 bme280;
Adafruit_BMP280 bmp280;
DHT dht11(DHT_PIN, DHT11);
DHT dht22(DHT_PIN, DHT22);
Adafruit_MPU6050 mpu6050;
BH1750 bh1750;
Adafruit_ADS1115 ads1115;
Adafruit_CCS811 ccs811;
Adafruit_MLX90614 mlx90614;
Adafruit_AHTX0 aht;
Adafruit_SHT31 sht31;
Adafruit_INA219 ina219;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);
  
  // Inicializa Display
  u8g2.begin();
  showMessage("Iniciando...", "Sistema Universal", "de Sensores");
  
  // Verifica Reset ANTES de inicializar EEPROM
  checkFactoryReset();
  
  // Inicializa EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Calcula tamanho das estruturas
  
  int totalSize = sizeof(uint32_t) + 
                  (sizeof(WiFiNetwork) * MAX_WIFI_NETWORKS) +
                  sizeof(int) +
                  (sizeof(SensorData) * MAX_SENSORS) +
                  (sizeof(ThingSpeakConfig) * MAX_THINGSPEAK_APIS) +
                  sizeof(DisplayConfig) +
                  sizeof(AlarmConfig);
  
  // Carrega Configurações
  loadConfig();
  
  
  // Verifica se precisa detectar sensores (primeira vez ou após reset)
  if (sensorCount == 0 || sensorCount > MAX_SENSORS) {
    detectSensors();
  } else {
    // Reinicializa sensores salvos
    reinitSensors();
  }
  
  // Conecta WiFi
  connectWiFi();
  
  // Inicia Servidor Web
  setupWebServer();
  server.begin();
  
  showMessage("Sistema Pronto!", WiFi.localIP().toString().c_str(), "");
  delay(2000);
}

void loop() {
  server.handleClient();
  
  // Atualiza Leituras dos Sensores
  updateSensors();
  
  // Atualiza Sensor Simulado
  updateSimulatedSensor();
  
  // Calcula Fórmulas
  calculateFormulas();
  
  // Atualiza Display
  updateDisplay();
  
  // Verifica Alarmes
  checkAlarms();
  
  // Envia para ThingSpeak
  updateThingSpeak();
  
  delay(100);
}

// ==================== EEPROM Functions ====================
void loadConfig() {
  int addr = 0;
  
  // Verifica número mágico para saber se EEPROM foi inicializada
  uint32_t magic;
  EEPROM.get(addr, magic);
  addr += sizeof(uint32_t);
  
  if (magic != EEPROM_MAGIC) {
    initDefaultConfig();
    saveConfig();
    return;
  }
  
  
  // Carrega WiFi Networks
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    EEPROM.get(addr, wifiNetworks[i]);
    addr += sizeof(WiFiNetwork);
  }
  
  // Carrega Sensores
  EEPROM.get(addr, sensorCount);
  addr += sizeof(int);
  for (int i = 0; i < MAX_SENSORS; i++) {
    EEPROM.get(addr, sensors[i]);
    addr += sizeof(SensorData);
  }
  
  // Carrega ThingSpeak
  for (int i = 0; i < MAX_THINGSPEAK_APIS; i++) {
    EEPROM.get(addr, thingspeakAPIs[i]);
    addr += sizeof(ThingSpeakConfig);
  }
  
  // Carrega Display Config
  EEPROM.get(addr, displayConfig);
  addr += sizeof(DisplayConfig);
  
  // Carrega Alarm Config
  EEPROM.get(addr, alarmConfig);
  addr += sizeof(AlarmConfig);
  
  // Carrega Fórmulas
  for (int i = 0; i < MAX_FORMULAS; i++) {
    EEPROM.get(addr, formulas[i]);
    addr += sizeof(FormulaConfig);
  }
  
  // Carrega Variáveis
  for (int i = 0; i < MAX_VARIABLES; i++) {
    EEPROM.get(addr, variables[i]);
    addr += sizeof(VariableConfig);
  }
}

void initDefaultConfig() {
  
  // Limpa WiFi Networks
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    memset(&wifiNetworks[i], 0, sizeof(WiFiNetwork));
    wifiNetworks[i].active = false;
  }
  
  // Limpa Sensores
  sensorCount = 0;
  for (int i = 0; i < MAX_SENSORS; i++) {
    memset(&sensors[i], 0, sizeof(SensorData));
    sensors[i].active = false;
  }
  
  // Limpa ThingSpeak
  for (int i = 0; i < MAX_THINGSPEAK_APIS; i++) {
    memset(&thingspeakAPIs[i], 0, sizeof(ThingSpeakConfig));
    thingspeakAPIs[i].active = false;
    thingspeakAPIs[i].interval = 15000; // 15 segundos padrão
    for (int j = 0; j < 8; j++) {
      thingspeakAPIs[i].sensorIndex[j] = -1;
      thingspeakAPIs[i].valueIndex[j] = 0;
    }
  }
  
  // Inicializa Display Config
  displayConfig.sensor1 = 0;
  displayConfig.value1 = 0;
  displayConfig.sensor2 = -1;
  displayConfig.value2 = 0;
  displayConfig.sensor3 = -1;
  displayConfig.value3 = 0;
  
  // Inicializa Alarm Config
  memset(&alarmConfig, 0, sizeof(AlarmConfig));
  alarmConfig.sensorIndex = 0;
  alarmConfig.valueIndex = 0;
  alarmConfig.threshold = 0;
  alarmConfig.above = true;
  alarmConfig.useLED = false;
  alarmConfig.useBuzzer = false;
  alarmConfig.active = false;
  
  // Inicializa Fórmulas
  for (int i = 0; i < MAX_FORMULAS; i++) {
    memset(&formulas[i], 0, sizeof(FormulaConfig));
    formulas[i].active = false;
  }
  
  // Inicializa Variáveis
  for (int i = 0; i < MAX_VARIABLES; i++) {
    variables[i].value = 0;
  }
  
}

void saveConfig() {
  int addr = 0;
  
  // Salva número mágico
  uint32_t magic = EEPROM_MAGIC;
  EEPROM.put(addr, magic);
  addr += sizeof(uint32_t);
  
  // Salva WiFi Networks
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    EEPROM.put(addr, wifiNetworks[i]);
    addr += sizeof(WiFiNetwork);
  }
  
  // Salva Sensores
  EEPROM.put(addr, sensorCount);
  addr += sizeof(int);
  for (int i = 0; i < MAX_SENSORS; i++) {
    EEPROM.put(addr, sensors[i]);
    addr += sizeof(SensorData);
  }
  
  // Salva ThingSpeak
  for (int i = 0; i < MAX_THINGSPEAK_APIS; i++) {
    EEPROM.put(addr, thingspeakAPIs[i]);
    addr += sizeof(ThingSpeakConfig);
    if (thingspeakAPIs[i].active || strlen(thingspeakAPIs[i].apiKey) > 0) {
    }
  }
  
  // Salva Display Config
  EEPROM.put(addr, displayConfig);
  addr += sizeof(DisplayConfig);
  
  // Salva Alarm Config
  EEPROM.put(addr, alarmConfig);
  addr += sizeof(AlarmConfig);
  
  // Salva Fórmulas
  for (int i = 0; i < MAX_FORMULAS; i++) {
    EEPROM.put(addr, formulas[i]);
    addr += sizeof(FormulaConfig);
  }
  
  // Salva Variáveis
  for (int i = 0; i < MAX_VARIABLES; i++) {
    EEPROM.put(addr, variables[i]);
    addr += sizeof(VariableConfig);
  }
  
  
  if (addr > EEPROM_SIZE) {
    return;
  }
  
  bool commitResult = EEPROM.commit();
  
  if (commitResult) {
    
    // Verifica se realmente salvou lendo de volta
    DisplayConfig testConfig;
    int testAddr = sizeof(uint32_t) + 
                   (sizeof(WiFiNetwork) * MAX_WIFI_NETWORKS) +
                   sizeof(int) +
                   (sizeof(SensorData) * MAX_SENSORS) +
                   (sizeof(ThingSpeakConfig) * MAX_THINGSPEAK_APIS);
    EEPROM.get(testAddr, testConfig);
    
    if (testConfig.sensor1 == displayConfig.sensor1 && 
        testConfig.value1 == displayConfig.value1) {
    } else {
    }
  } else {
  }
}

void checkFactoryReset() {
  // Função mantida para compatibilidade, mas não faz nada
  // Reset agora é feito via web na página de Logs
}

// ==================== Sensor Detection ====================
void detectSensors() {
  showMessage("Detectando", "Sensores...", "");
  sensorCount = 0;
  
  Wire.begin(BME_SDA, BME_SCL);
  
  // Tenta detectar sensores I2C
  detectBME280();
  detectBMP280();
  detectMPU6050();
  detectBH1750();
  detectADS1115();
  detectCCS811();
  detectMLX90614();
  detectAHT();
  detectSHT31();
  detectINA219();
  
  // Tenta detectar DHT
  detectDHT();
  
  saveConfig();
  
  char msg[32];
  sprintf(msg, "%d sensores", sensorCount);
  showMessage("Deteccao OK!", msg, "");
  delay(2000);
}

void reinitSensors() {
  showMessage("Carregando", "Sensores...", "");
  Wire.begin(BME_SDA, BME_SCL);
  
  // Reinicializa objetos dos sensores baseado no que está salvo
  for (int i = 0; i < sensorCount; i++) {
    if (!sensors[i].active) continue;
    
    if (strcmp(sensors[i].name, "BME280") == 0) {
      bme280.begin(sensors[i].address, &Wire);
    }
    else if (strcmp(sensors[i].name, "BMP280") == 0) {
      bmp280.begin(sensors[i].address);
    }
    else if (strcmp(sensors[i].name, "MPU6050") == 0) {
      mpu6050.begin();
    }
    else if (strcmp(sensors[i].name, "BH1750") == 0) {
      bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    }
    else if (strcmp(sensors[i].name, "ADS1115") == 0) {
      ads1115.begin();
    }
    else if (strcmp(sensors[i].name, "CCS811") == 0) {
      ccs811.begin();
    }
    else if (strcmp(sensors[i].name, "MLX90614") == 0) {
      mlx90614.begin();
    }
    else if (strcmp(sensors[i].name, "AHT10/20") == 0) {
      aht.begin();
    }
    else if (strcmp(sensors[i].name, "SHT31") == 0) {
      sht31.begin(sensors[i].address);
    }
    else if (strcmp(sensors[i].name, "INA219") == 0) {
      ina219.begin();
    }
    else if (strcmp(sensors[i].name, "DHT11") == 0) {
      dht11.begin();
    }
    else if (strcmp(sensors[i].name, "DHT22") == 0) {
      dht22.begin();
    }
  }
  
  char msg[32];
  sprintf(msg, "%d sensores", sensorCount);
  showMessage("Sensores OK!", msg, "");
  delay(1000);
}

void detectBME280() {
  if (bme280.begin(0x76, &Wire) || bme280.begin(0x77, &Wire)) {
    strcpy(sensors[sensorCount].name, "BME280");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x76;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "Temp");
    strcpy(sensors[sensorCount].label2, "Umid");
    strcpy(sensors[sensorCount].label3, "Press");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectBMP280() {
  if (bmp280.begin(0x76) || bmp280.begin(0x77)) {
    bool isBME = false;
    for (int i = 0; i < sensorCount; i++) {
      if (strcmp(sensors[i].name, "BME280") == 0) {
        isBME = true;
        break;
      }
    }
    if (!isBME) {
      strcpy(sensors[sensorCount].name, "BMP280");
      strcpy(sensors[sensorCount].type, "I2C");
      sensors[sensorCount].address = 0x76;
      sensors[sensorCount].active = true;
      strcpy(sensors[sensorCount].label1, "Temp");
      strcpy(sensors[sensorCount].label2, "Press");
      sensors[sensorCount].calibOffset1 = 0;
      sensors[sensorCount].calibOffset2 = 0;
      sensors[sensorCount].calibOffset3 = 0;
      sensors[sensorCount].calibOffset4 = 0;
      sensors[sensorCount].calibScale1 = 1.0;
      sensors[sensorCount].calibScale2 = 1.0;
      sensors[sensorCount].calibScale3 = 1.0;
      sensors[sensorCount].calibScale4 = 1.0;
      sensors[sensorCount].value1 = 0;
      sensors[sensorCount].value2 = 0;
      sensors[sensorCount].value3 = 0;
      sensors[sensorCount].value4 = 0;
      sensorCount++;
    }
  }
}

void detectMPU6050() {
  if (mpu6050.begin()) {
    strcpy(sensors[sensorCount].name, "MPU6050");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x68;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "AccX");
    strcpy(sensors[sensorCount].label2, "AccY");
    strcpy(sensors[sensorCount].label3, "AccZ");
    strcpy(sensors[sensorCount].label4, "Temp");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectBH1750() {
  if (bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    strcpy(sensors[sensorCount].name, "BH1750");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x23;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "Lux");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectADS1115() {
  if (ads1115.begin()) {
    strcpy(sensors[sensorCount].name, "ADS1115");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x48;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "A0");
    strcpy(sensors[sensorCount].label2, "A1");
    strcpy(sensors[sensorCount].label3, "A2");
    strcpy(sensors[sensorCount].label4, "A3");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectCCS811() {
  if (ccs811.begin()) {
    strcpy(sensors[sensorCount].name, "CCS811");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x5A;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "CO2");
    strcpy(sensors[sensorCount].label2, "TVOC");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectMLX90614() {
  if (mlx90614.begin()) {
    strcpy(sensors[sensorCount].name, "MLX90614");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x5A;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "ObjTemp");
    strcpy(sensors[sensorCount].label2, "AmbTemp");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectAHT() {
  if (aht.begin()) {
    strcpy(sensors[sensorCount].name, "AHT10/20");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x38;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "Temp");
    strcpy(sensors[sensorCount].label2, "Umid");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectSHT31() {
  if (sht31.begin(0x44)) {
    strcpy(sensors[sensorCount].name, "SHT31");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x44;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "Temp");
    strcpy(sensors[sensorCount].label2, "Umid");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectINA219() {
  if (ina219.begin()) {
    strcpy(sensors[sensorCount].name, "INA219");
    strcpy(sensors[sensorCount].type, "I2C");
    sensors[sensorCount].address = 0x40;
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "Volt");
    strcpy(sensors[sensorCount].label2, "Curr");
    strcpy(sensors[sensorCount].label3, "Power");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void detectDHT() {
  // Tenta DHT22 primeiro
  dht22.begin();
  delay(2000);
  float h = dht22.readHumidity();
  float t = dht22.readTemperature();
  
  if (!isnan(h) && !isnan(t)) {
    strcpy(sensors[sensorCount].name, "DHT22");
    strcpy(sensors[sensorCount].type, "Digital");
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "Temp");
    strcpy(sensors[sensorCount].label2, "Umid");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
    return;
  }
  
  // Tenta DHT11
  dht11.begin();
  delay(2000);
  h = dht11.readHumidity();
  t = dht11.readTemperature();
  
  if (!isnan(h) && !isnan(t)) {
    strcpy(sensors[sensorCount].name, "DHT11");
    strcpy(sensors[sensorCount].type, "Digital");
    sensors[sensorCount].active = true;
    strcpy(sensors[sensorCount].label1, "Temp");
    strcpy(sensors[sensorCount].label2, "Umid");
    sensors[sensorCount].calibOffset1 = 0;
    sensors[sensorCount].calibOffset2 = 0;
    sensors[sensorCount].calibOffset3 = 0;
    sensors[sensorCount].calibOffset4 = 0;
    sensors[sensorCount].calibScale1 = 1.0;
    sensors[sensorCount].calibScale2 = 1.0;
    sensors[sensorCount].calibScale3 = 1.0;
    sensors[sensorCount].calibScale4 = 1.0;
    sensors[sensorCount].value1 = 0;
    sensors[sensorCount].value2 = 0;
    sensors[sensorCount].value3 = 0;
    sensors[sensorCount].value4 = 0;
    sensorCount++;
  }
}

void addSimulatedSensor() {
  if (sensorCount >= MAX_SENSORS) return;
  
  strcpy(sensors[sensorCount].name, "SIMSENSOR");
  strcpy(sensors[sensorCount].type, "Simulado");
  sensors[sensorCount].address = 0xFF;
  sensors[sensorCount].active = true;
  strcpy(sensors[sensorCount].label1, "Temp");
  strcpy(sensors[sensorCount].label2, "Umid");
  strcpy(sensors[sensorCount].label3, "Press");
  sensors[sensorCount].calibOffset1 = 0;
  sensors[sensorCount].calibOffset2 = 0;
  sensors[sensorCount].calibOffset3 = 0;
  sensors[sensorCount].calibOffset4 = 0;
  sensors[sensorCount].calibScale1 = 1.0;
  sensors[sensorCount].calibScale2 = 1.0;
  sensors[sensorCount].calibScale3 = 1.0;
  sensors[sensorCount].calibScale4 = 1.0;
  sensors[sensorCount].value1 = 25.0;  // Temperatura inicial
  sensors[sensorCount].value2 = 60.0;  // Umidade inicial
  sensors[sensorCount].value3 = 1013.25; // Pressão inicial
  sensors[sensorCount].value4 = 0;
  sensorCount++;
  simulatedSensorEnabled = true;
  saveConfig();
}

void updateSimulatedSensor() {
  if (!simulatedSensorEnabled) return;
  
  unsigned long now = millis();
  if (now - lastSimulatedUpdate < 2000) return; // Atualiza a cada 2 segundos
  lastSimulatedUpdate = now;
  
  // Encontra o sensor simulado
  for (int i = 0; i < sensorCount; i++) {
    if (strcmp(sensors[i].name, "SIMSENSOR") == 0 && sensors[i].active) {
      // Variação pequena e aleatória
      sensors[i].value1 += (random(-10, 11) / 100.0); // ±0.1°C
      sensors[i].value2 += (random(-5, 6) / 10.0);    // ±0.5%
      sensors[i].value3 += (random(-2, 3) / 10.0);    // ±0.2 hPa
      
      // Limita os valores em ranges realistas
      if (sensors[i].value1 < 20.0) sensors[i].value1 = 20.0;
      if (sensors[i].value1 > 30.0) sensors[i].value1 = 30.0;
      if (sensors[i].value2 < 40.0) sensors[i].value2 = 40.0;
      if (sensors[i].value2 > 80.0) sensors[i].value2 = 80.0;
      if (sensors[i].value3 < 1000.0) sensors[i].value3 = 1000.0;
      if (sensors[i].value3 > 1020.0) sensors[i].value3 = 1020.0;
      
      break;
    }
  }
}

// ==================== Sensor Update ====================
void updateSensors() {
  Wire.begin(BME_SDA, BME_SCL);
  
  for (int i = 0; i < sensorCount; i++) {
    if (!sensors[i].active) continue;
    
    // Pula sensor simulado (ele é atualizado em updateSimulatedSensor)
    if (strcmp(sensors[i].name, "SIMSENSOR") == 0) continue;
    
    bool sensorOK = true;
    
    if (strcmp(sensors[i].name, "BME280") == 0) {
      float temp = bme280.readTemperature();
      if (isnan(temp) || temp < -50 || temp > 100) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (temp * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (bme280.readHumidity() * sensors[i].calibScale2) + sensors[i].calibOffset2;
        sensors[i].value3 = (bme280.readPressure() / 100.0F * sensors[i].calibScale3) + sensors[i].calibOffset3;
      }
    }
    else if (strcmp(sensors[i].name, "BMP280") == 0) {
      float temp = bmp280.readTemperature();
      if (isnan(temp) || temp < -50 || temp > 100) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (temp * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (bmp280.readPressure() / 100.0F * sensors[i].calibScale2) + sensors[i].calibOffset2;
      }
    }
    else if (strcmp(sensors[i].name, "DHT11") == 0 || strcmp(sensors[i].name, "DHT22") == 0) {
      DHT* dht = (strcmp(sensors[i].name, "DHT22") == 0) ? &dht22 : &dht11;
      float temp = dht->readTemperature();
      float hum = dht->readHumidity();
      if (isnan(temp) || isnan(hum)) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (temp * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (hum * sensors[i].calibScale2) + sensors[i].calibOffset2;
      }
    }
    else if (strcmp(sensors[i].name, "MPU6050") == 0) {
      sensors_event_t a, g, temp;
      if (!mpu6050.getEvent(&a, &g, &temp)) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (a.acceleration.x * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (a.acceleration.y * sensors[i].calibScale2) + sensors[i].calibOffset2;
        sensors[i].value3 = (a.acceleration.z * sensors[i].calibScale3) + sensors[i].calibOffset3;
        sensors[i].value4 = (temp.temperature * sensors[i].calibScale4) + sensors[i].calibOffset4;
      }
    }
    else if (strcmp(sensors[i].name, "BH1750") == 0) {
      float lux = bh1750.readLightLevel();
      if (lux < 0 || isnan(lux)) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (lux * sensors[i].calibScale1) + sensors[i].calibOffset1;
      }
    }
    else if (strcmp(sensors[i].name, "ADS1115") == 0) {
      int16_t adc0 = ads1115.readADC_SingleEnded(0);
      if (adc0 < 0) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (adc0 * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (ads1115.readADC_SingleEnded(1) * sensors[i].calibScale2) + sensors[i].calibOffset2;
        sensors[i].value3 = (ads1115.readADC_SingleEnded(2) * sensors[i].calibScale3) + sensors[i].calibOffset3;
        sensors[i].value4 = (ads1115.readADC_SingleEnded(3) * sensors[i].calibScale4) + sensors[i].calibOffset4;
      }
    }
    else if (strcmp(sensors[i].name, "CCS811") == 0) {
      if (!ccs811.available()) {
        sensorOK = false;
      } else {
        ccs811.readData();
        sensors[i].value1 = (ccs811.geteCO2() * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (ccs811.getTVOC() * sensors[i].calibScale2) + sensors[i].calibOffset2;
      }
    }
    else if (strcmp(sensors[i].name, "MLX90614") == 0) {
      float objTemp = mlx90614.readObjectTempC();
      if (isnan(objTemp) || objTemp < -70) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (objTemp * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (mlx90614.readAmbientTempC() * sensors[i].calibScale2) + sensors[i].calibOffset2;
      }
    }
    else if (strcmp(sensors[i].name, "AHT10/20") == 0) {
      sensors_event_t humidity, temp;
      if (!aht.getEvent(&humidity, &temp)) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (temp.temperature * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (humidity.relative_humidity * sensors[i].calibScale2) + sensors[i].calibOffset2;
      }
    }
    else if (strcmp(sensors[i].name, "SHT31") == 0) {
      float temp = sht31.readTemperature();
      if (isnan(temp)) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (temp * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (sht31.readHumidity() * sensors[i].calibScale2) + sensors[i].calibOffset2;
      }
    }
    else if (strcmp(sensors[i].name, "INA219") == 0) {
      float voltage = ina219.getBusVoltage_V();
      if (isnan(voltage) || voltage < 0) {
        sensorOK = false;
      } else {
        sensors[i].value1 = (voltage * sensors[i].calibScale1) + sensors[i].calibOffset1;
        sensors[i].value2 = (ina219.getCurrent_mA() * sensors[i].calibScale2) + sensors[i].calibOffset2;
        sensors[i].value3 = (ina219.getPower_mW() * sensors[i].calibScale3) + sensors[i].calibOffset3;
      }
    }
    
    // Marca sensor como ativo/inativo baseado na leitura
    if (!sensorOK && sensors[i].active) {
      sensors[i].active = false;
    } else if (sensorOK && !sensors[i].active) {
      sensors[i].active = true;
    }
  }
}

// ==================== WiFi Functions ====================
void connectWiFi() {
  showMessage("Conectando", "WiFi...", "");
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  // Ordena redes por força de sinal
  int n = WiFi.scanNetworks();
  int bestIndex = -1;
  int bestRSSI = -100;
  
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    if (!wifiNetworks[i].active) continue;
    
    for (int j = 0; j < n; j++) {
      if (strcmp(wifiNetworks[i].ssid, WiFi.SSID(j).c_str()) == 0) {
        if (WiFi.RSSI(j) > bestRSSI) {
          bestRSSI = WiFi.RSSI(j);
          bestIndex = i;
        }
      }
    }
  }
  
  if (bestIndex >= 0) {
    WiFi.begin(wifiNetworks[bestIndex].ssid, wifiNetworks[bestIndex].password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      apMode = false;
      WiFi.softAPdisconnect(true);
    }
  }
}

// ==================== Display Functions ====================
void showMessage(const char* line1, const char* line2, const char* line3) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 15);
  u8g2.print(line1);
  u8g2.setCursor(0, 35);
  u8g2.print(line2);
  u8g2.setCursor(0, 55);
  u8g2.print(line3);
  u8g2.sendBuffer();
}

void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  
  // Linha 1: IP
  u8g2.setCursor(0, 10);
  if (apMode) {
    u8g2.print("AP: ");
    u8g2.print(WiFi.softAPIP().toString());
  } else {
    u8g2.print("IP: ");
    u8g2.print(WiFi.localIP().toString());
  }
  
  u8g2.drawHLine(0, 12, 128);
  
  // Verifica se há sensores desconectados
  bool hasDisconnected = false;
  String disconnectedSensor = "";
  for (int i = 0; i < sensorCount; i++) {
    if (!sensors[i].active) {
      hasDisconnected = true;
      disconnectedSensor = String(sensors[i].name);
      break;
    }
  }
  
  if (hasDisconnected) {
    // Mostra aviso de sensor desconectado
    u8g2.setCursor(0, 30);
    u8g2.print("SENSOR DESCONECTADO:");
    u8g2.setCursor(0, 45);
    u8g2.print(disconnectedSensor);
    u8g2.setCursor(0, 60);
    u8g2.print("Verifique conexoes");
  } else {
    // Linhas 2-4: Dados dos Sensores Configurados
    int yPos = 25;
    
    if (displayConfig.sensor1 >= 0 && displayConfig.sensor1 < sensorCount && sensors[displayConfig.sensor1].active) {
      u8g2.setCursor(0, yPos);
      u8g2.print(sensors[displayConfig.sensor1].name);
      u8g2.print(" ");
      u8g2.print(getValueLabel(displayConfig.sensor1, displayConfig.value1));
      u8g2.print(": ");
      u8g2.print(getValue(displayConfig.sensor1, displayConfig.value1), 1);
      yPos += 13;
    }
    
    if (displayConfig.sensor2 >= 0 && displayConfig.sensor2 < sensorCount && sensors[displayConfig.sensor2].active) {
      u8g2.setCursor(0, yPos);
      u8g2.print(sensors[displayConfig.sensor2].name);
      u8g2.print(" ");
      u8g2.print(getValueLabel(displayConfig.sensor2, displayConfig.value2));
      u8g2.print(": ");
      u8g2.print(getValue(displayConfig.sensor2, displayConfig.value2), 1);
      yPos += 13;
    }
    
    if (displayConfig.sensor3 >= 0 && displayConfig.sensor3 < sensorCount && sensors[displayConfig.sensor3].active) {
      u8g2.setCursor(0, yPos);
      u8g2.print(sensors[displayConfig.sensor3].name);
      u8g2.print(" ");
      u8g2.print(getValueLabel(displayConfig.sensor3, displayConfig.value3));
      u8g2.print(": ");
      u8g2.print(getValue(displayConfig.sensor3, displayConfig.value3), 1);
    }
  }
  
  u8g2.sendBuffer();
}

float getValue(int sensorIndex, int valueIndex) {
  // Se sensorIndex >= 100, é um valor calculado (C1-C5)
  if (sensorIndex >= 100 && sensorIndex < 100 + MAX_FORMULAS) {
    return calculatedValues[sensorIndex - 100];
  }
  if (sensorIndex < 0 || sensorIndex >= sensorCount) return 0;
  switch(valueIndex) {
    case 0: return sensors[sensorIndex].value1;
    case 1: return sensors[sensorIndex].value2;
    case 2: return sensors[sensorIndex].value3;
    case 3: return sensors[sensorIndex].value4;
    default: return 0;
  }
}

const char* getValueLabel(int sensorIndex, int valueIndex) {
  // Se sensorIndex >= 100, é um valor calculado (C1-C5)
  static char calcLabel[8];
  if (sensorIndex >= 100 && sensorIndex < 100 + MAX_FORMULAS) {
    sprintf(calcLabel, "C%d", sensorIndex - 99);
    return calcLabel;
  }
  if (sensorIndex < 0 || sensorIndex >= sensorCount) return "";
  switch(valueIndex) {
    case 0: return sensors[sensorIndex].label1;
    case 1: return sensors[sensorIndex].label2;
    case 2: return sensors[sensorIndex].label3;
    case 3: return sensors[sensorIndex].label4;
    default: return "";
  }
}

// ==================== Alarm Functions ====================
void checkAlarms() {
  if (!alarmConfig.active) {
    digitalWrite(ALARM_PIN, LOW);
    return;
  }
  
  float value = getValue(alarmConfig.sensorIndex, alarmConfig.valueIndex);
  bool triggered = alarmConfig.above ? (value > alarmConfig.threshold) : (value < alarmConfig.threshold);
  
  if (triggered) {
    if (alarmConfig.useBuzzer) {
      digitalWrite(ALARM_PIN, HIGH);
    } else if (alarmConfig.useLED) {
      unsigned long now = millis();
      if (now - lastAlarmBlink > 1000) {
        alarmBlinkCount = 0;
        lastAlarmBlink = now;
      }
      
      if (alarmBlinkCount < 6) {
        if ((now - lastAlarmBlink) % 200 < 100) {
          digitalWrite(ALARM_PIN, HIGH);
        } else {
          digitalWrite(ALARM_PIN, LOW);
          if ((now - lastAlarmBlink) % 200 == 0) alarmBlinkCount++;
        }
      } else {
        digitalWrite(ALARM_PIN, LOW);
      }
    }
  } else {
    digitalWrite(ALARM_PIN, LOW);
    alarmBlinkCount = 0;
  }
}

// ==================== ThingSpeak Functions ====================
void updateThingSpeak() {
  for (int i = 0; i < MAX_THINGSPEAK_APIS; i++) {
    if (!thingspeakAPIs[i].active) continue;
    
    unsigned long now = millis();
    if (now - lastThingSpeakUpdate[i] < thingspeakAPIs[i].interval) continue;
    
    lastThingSpeakUpdate[i] = now;
    
    WiFiClient client;
    if (client.connect("api.thingspeak.com", 80)) {
      String postStr = "api_key=" + String(thingspeakAPIs[i].apiKey);
      
      for (int j = 0; j < 8; j++) {
        if (thingspeakAPIs[i].sensorIndex[j] >= 0) {
          float value = getValue(thingspeakAPIs[i].sensorIndex[j], thingspeakAPIs[i].valueIndex[j]);
          postStr += "&field" + String(j + 1) + "=" + String(value);
        }
      }
      
      client.print("POST /update HTTP/1.1\n");
      client.print("Host: api.thingspeak.com\n");
      client.print("Connection: close\n");
      client.print("Content-Type: application/x-www-form-urlencoded\n");
      client.print("Content-Length: " + String(postStr.length()) + "\n\n");
      client.print(postStr);
      client.stop();
    }
  }
}

// ==================== Web Server Setup ====================
void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/scan", handleScan);
  server.on("/wifi", handleWiFiConfig);
  server.on("/sensors", handleSensorsJSON);
  server.on("/calibrate", handleCalibrate);
  server.on("/thingspeak", handleThingSpeak);
  server.on("/display", handleDisplayConfig);
  server.on("/alarm", handleAlarmConfig);
  server.on("/calculations", handleCalculations);
  server.on("/calcvalues", handleCalcValuesJSON);
  server.on("/logs", handleLogs);
  server.on("/rescan", handleRescan);
}

// ==================== Web Handlers ====================
void handleRoot() {
  String h = getHTMLHeader("Dashboard");
  h += getSidebar();
  h += "<main><h1><span class='dot'></span>Dashboard IoT</h1>";
  h += "<div id='d' style='display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:1rem'></div></main>";
  h += "<script>";
  // icon + unit + range maps
  h += "const IC={BME280:'thermometer',BMP280:'thermometer',DHT11:'droplets',DHT22:'droplets',MPU6050:'activity',BH1750:'sun',ADS1115:'cpu',CCS811:'wind',MLX90614:'flame',AHT10:'droplets',AHT20:'droplets',SHT31:'droplets',INA219:'zap',SIMSENSOR:'radio'};";
  h += "const UN={Temp:'°C',Umid:'%',Press:'hPa',Lux:'lx',Volt:'V',Curr:'mA',AccX:'g',AccY:'g',AccZ:'g'};";
  h += "const RG={Temp:[-10,60],Umid:[0,100],Press:[900,1100],Lux:[0,2000],Volt:[0,5],Curr:[0,500],AccX:[-2,2],AccY:[-2,2],AccZ:[-2,2]};";
  h += "const GC=['#6366f1','#22d3ee','#a78bfa','#34d399'];";
  h += "const gi=n=>IC[n]||'cpu';";
  // gauge draw function — pure Canvas 2D, no library, no scroll issues
  h += "function dg(cv,val,lbl,ci){";
  h += "const ctx=cv.getContext('2d'),W=cv.width,H=cv.height;";
  h += "ctx.clearRect(0,0,W,H);";
  h += "const cx=W>>1,cy=~~(H*.76),r=~~(W*.37);";
  h += "const SA=Math.PI*.75,EA=Math.PI*2.25;";
  h += "const rg=RG[lbl]||[0,Math.max(Math.abs(val)*2,1)];";
  h += "const pct=Math.max(0,Math.min(1,(val-rg[0])/(rg[1]-rg[0])));";
  h += "const VA=SA+pct*(EA-SA),col=GC[ci%4];";
  // outer glow ring
  h += "ctx.beginPath();ctx.arc(cx,cy,r+4,SA,EA);";
  h += "ctx.strokeStyle='rgba(255,255,255,.04)';ctx.lineWidth=2;ctx.stroke();";
  // background track
  h += "ctx.beginPath();ctx.arc(cx,cy,r,SA,EA);";
  h += "ctx.strokeStyle='rgba(255,255,255,.1)';ctx.lineWidth=10;ctx.lineCap='round';ctx.stroke();";
  // colored value arc
  h += "ctx.beginPath();ctx.arc(cx,cy,r,SA,SA+Math.max(pct,.01)*(EA-SA));";
  h += "ctx.strokeStyle=col;ctx.lineWidth=10;ctx.lineCap='round';ctx.stroke();";
  // center dot
  h += "ctx.beginPath();ctx.arc(cx,cy,4,0,6.28);ctx.fillStyle=col;ctx.fill();";
  // value text
  h += "ctx.fillStyle='#f1f5f9';ctx.font='bold 14px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';";
  h += "ctx.fillText(val.toFixed(1),cx,cy-~~(r*.3));";
  // unit text
  h += "ctx.fillStyle=col;ctx.font='bold 9px sans-serif';";
  h += "ctx.fillText(UN[lbl]||'',cx,cy-~~(r*.05));";
  // label text
  h += "ctx.fillStyle='#94a3b8';ctx.font='9px sans-serif';";
  h += "ctx.fillText(lbl,cx,cy+~~(r*.55));";
  h += "}";
  // build cards DOM
  h += "function mk(d){";
  h += "let c='';";
  h += "d.forEach((s,i)=>{";
  h += "c+='<article style=\"padding:0;overflow:hidden\">';";
  h += "c+='<header style=\"padding:.65rem 1rem;display:flex;align-items:center;gap:.5rem\"><i data-lucide=\"'+gi(s.name)+'\"></i><strong>'+s.name+'</strong></header>';";
  h += "c+='<div style=\"padding:.75rem .5rem;display:flex;flex-wrap:wrap;justify-content:center;gap:.25rem\">';";
  h += "[[s.v1,s.l1,0],[s.v2,s.l2,1],[s.v3,s.l3,2],[s.v4,s.l4,3]].forEach(([v,l,j])=>{";
  h += "if(v!==undefined&&l)c+='<canvas id=\"G_'+i+'_'+j+'\" width=\"110\" height=\"100\" style=\"display:block\"></canvas>';";
  h += "});";
  h += "c+='</div></article>';";
  h += "});";
  h += "document.getElementById('d').innerHTML=c;";
  h += "lucide.createIcons();";
  // initial draw
  h += "d.forEach((s,i)=>{[[s.v1,s.l1,0],[s.v2,s.l2,1],[s.v3,s.l3,2],[s.v4,s.l4,3]].forEach(([v,l,j])=>{";
  h += "if(v!==undefined&&l){const cv=document.getElementById('G_'+i+'_'+j);if(cv)dg(cv,v,l,j);}";
  h += "});});";
  h += "}";
  // update loop
  h += "let rdy=false;";
  h += "function upd(){fetch('/sensors').then(r=>r.json()).then(d=>{";
  h += "if(!rdy){mk(d);rdy=true;return;}";
  h += "d.forEach((s,i)=>{[[s.v1,s.l1,0],[s.v2,s.l2,1],[s.v3,s.l3,2],[s.v4,s.l4,3]].forEach(([v,l,j])=>{";
  h += "if(v!==undefined&&l){const cv=document.getElementById('G_'+i+'_'+j);if(cv)dg(cv,v,l,j);}";
  h += "});});}).catch(()=>{});}";
  h += "upd();setInterval(upd,2000);";
  h += "</script>";
  h += getHTMLFooter();
  server.send(200, "text/html", h);
}

void handleScan() {
  String json = "[";
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\"", "\\\"");
    json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + ",\"enc\":" + String(WiFi.encryptionType(i)) + "}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleWiFiConfig() {
  if (server.method() == HTTP_POST) {
    int slot = server.arg("slot").toInt();
    if (slot >= 0 && slot < MAX_WIFI_NETWORKS) {
      strcpy(wifiNetworks[slot].ssid, server.arg("ssid").c_str());
      strcpy(wifiNetworks[slot].password, server.arg("pass").c_str());
      wifiNetworks[slot].active = true;
      saveConfig();
      server.send(200, "text/plain", "OK");
      delay(1000);
      ESP.restart();
    }
    return;
  }
  
  String h = getHTMLHeader("WiFi");
  h += getSidebar();
  h += "<main><h1><i data-lucide='wifi'></i> Configuração WiFi</h1>";
  h += "<button onclick='scanWiFi()' style='margin-bottom:1rem'><i data-lucide='search'></i> Escanear Redes</button>";
  h += "<div id='networks'></div>";
  
  h += "<h2>Redes Salvas</h2>";
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    h += "<article><header>Slot " + String(i + 1) + "</header>";
    if (wifiNetworks[i].active) {
      h += "<p>SSID: " + String(wifiNetworks[i].ssid) + "</p>";
    } else {
      h += "<p>Vazio</p>";
    }
    h += "<input type='text' id='ssid" + String(i) + "' placeholder='SSID'>";
    h += "<input type='password' id='pass" + String(i) + "' placeholder='Senha'>";
    h += "<button onclick='saveWiFi(" + String(i) + ")'>Salvar</button>";
    h += "</article>";
  }
  
  h += "<script>";
  h += "function scanWiFi(){";
  h += "fetch('/scan').then(r=>r.json()).then(data=>{";
  h += "let html='<article><header><i data-lucide=\"radio\"></i> Redes Disponíveis</header><div style=\"padding:.75rem\">';";
  h += "data.forEach(n=>{";
  h += "let sig=n.rssi>-60?'#22d3ee':n.rssi>-75?'#fbbf24':'#f87171';";
  h += "html+='<button onclick=\"selectNetwork(\\''+n.ssid.replace(/'/g,\"\\\\'\")+'\\')\" style=\"margin-bottom:.4rem;width:100%;text-align:left;background:rgba(99,102,241,.15)!important;display:flex;justify-content:space-between\">';";
  h += "html+='<span>'+n.ssid+'</span><span style=\"color:'+sig+'\">'+n.rssi+' dBm</span></button>';";
  h += "});";
  h += "html+='</div></article>';";
  h += "document.getElementById('networks').innerHTML=html;";
  h += "if(typeof lucide!=='undefined')lucide.createIcons();";
  h += "});";
  h += "}";
  h += "function selectNetwork(ssid){document.getElementById('ssid0').value=ssid;}";
  h += "function saveWiFi(slot){";
  h += "let ssid=document.getElementById('ssid'+slot).value;";
  h += "let pass=document.getElementById('pass'+slot).value;";
  h += "fetch('/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},";
  h += "body:'slot='+slot+'&ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)})";
  h += ".then(()=>alert('Salvo! Reiniciando...'));";
  h += "}";
  h += "</script></main>";
  
  h += getHTMLFooter();
  server.send(200, "text/html", h);
}

void handleSensorsJSON() {
  String json = "[";
  for (int i = 0; i < sensorCount; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + String(sensors[i].name) + "\"";
    
    // Verifica e adiciona valor 1
    if (strlen(sensors[i].label1) > 0 && strcmp(sensors[i].label1, "") != 0) {
      json += ",\"l1\":\"" + String(sensors[i].label1) + "\"";
      // Verifica se é NaN ou infinito
      if (isnan(sensors[i].value1) || isinf(sensors[i].value1)) {
        json += ",\"v1\":0";
      } else {
        json += ",\"v1\":" + String(sensors[i].value1, 2);
      }
    }
    
    // Verifica e adiciona valor 2
    if (strlen(sensors[i].label2) > 0 && strcmp(sensors[i].label2, "") != 0) {
      json += ",\"l2\":\"" + String(sensors[i].label2) + "\"";
      if (isnan(sensors[i].value2) || isinf(sensors[i].value2)) {
        json += ",\"v2\":0";
      } else {
        json += ",\"v2\":" + String(sensors[i].value2, 2);
      }
    }
    
    // Verifica e adiciona valor 3
    if (strlen(sensors[i].label3) > 0 && strcmp(sensors[i].label3, "") != 0) {
      json += ",\"l3\":\"" + String(sensors[i].label3) + "\"";
      if (isnan(sensors[i].value3) || isinf(sensors[i].value3)) {
        json += ",\"v3\":0";
      } else {
        json += ",\"v3\":" + String(sensors[i].value3, 2);
      }
    }
    
    // Verifica e adiciona valor 4
    if (strlen(sensors[i].label4) > 0 && strcmp(sensors[i].label4, "") != 0) {
      json += ",\"l4\":\"" + String(sensors[i].label4) + "\"";
      if (isnan(sensors[i].value4) || isinf(sensors[i].value4)) {
        json += ",\"v4\":0";
      } else {
        json += ",\"v4\":" + String(sensors[i].value4, 2);
      }
    }
    
    json += "}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleCalibrate() {
  if (server.method() == HTTP_POST) {
    int sensorIdx = server.arg("sensor").toInt();
    int valueIdx = server.arg("value").toInt();
    float reading = server.arg("reading").toFloat();
    float reference = server.arg("reference").toFloat();
    
    if (sensorIdx >= 0 && sensorIdx < sensorCount) {
      float currentValue = getValue(sensorIdx, valueIdx);
      float offset = reference - currentValue;
      
      switch(valueIdx) {
        case 0: sensors[sensorIdx].calibOffset1 = offset; break;
        case 1: sensors[sensorIdx].calibOffset2 = offset; break;
        case 2: sensors[sensorIdx].calibOffset3 = offset; break;
        case 3: sensors[sensorIdx].calibOffset4 = offset; break;
      }
      
      // Debug
      
      saveConfig();
      server.send(200, "text/plain", "OK");
    }
    return;
  }
  
  String html = getHTMLHeader("Calibração");
  html += getSidebar();
  html += "<main><h1><i data-lucide='sliders-horizontal'></i> Calibração de Sensores</h1>";
  
  for (int i = 0; i < sensorCount; i++) {
    html += "<div class='card'>";
    html += "<h3>" + String(sensors[i].name) + "</h3>";
    
    for (int j = 0; j < 4; j++) {
      const char* label = getValueLabel(i, j);
      if (strlen(label) == 0) continue;
      
      html += "<div><strong>" + String(label) + ":</strong> <span id='v" + String(i) + "_" + String(j) + "'>--</span></div>";
      html += "<input type='number' step='0.01' id='ref" + String(i) + "_" + String(j) + "' placeholder='Valor de Referencia'>";
      html += "<button onclick='calibrate(" + String(i) + "," + String(j) + ")' style='margin-top:.4rem'><i data-lucide='check-circle'></i> Calibrar</button>";
    }
    
    html += "</div>";
  }
  
  html += "<script>";
  html += "setInterval(()=>{fetch('/sensors').then(r=>r.json()).then(d=>{";
  html += "d.forEach((s,i)=>{";
  html += "if(s.v1!==undefined){let e=document.getElementById('v'+i+'_0');if(e)e.innerText=s.v1.toFixed(2);}";
  html += "if(s.v2!==undefined){let e=document.getElementById('v'+i+'_1');if(e)e.innerText=s.v2.toFixed(2);}";
  html += "if(s.v3!==undefined){let e=document.getElementById('v'+i+'_2');if(e)e.innerText=s.v3.toFixed(2);}";
  html += "if(s.v4!==undefined){let e=document.getElementById('v'+i+'_3');if(e)e.innerText=s.v4.toFixed(2);}";
  html += "});});},2000);";
  html += "function calibrate(sensor,value){";
  html += "let reading=parseFloat(document.getElementById('v'+sensor+'_'+value).innerText);";
  html += "let reference=parseFloat(document.getElementById('ref'+sensor+'_'+value).value);";
  html += "fetch('/calibrate',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},";
  html += "body:'sensor='+sensor+'&value='+value+'&reading='+reading+'&reference='+reference})";
  html += ".then(()=>alert('Calibrado!'));}";
  html += "</script></main>";
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}

void handleThingSpeak() {
  if (server.method() == HTTP_POST) {
    int apiIdx = server.arg("api").toInt();
    if (apiIdx >= 0 && apiIdx < MAX_THINGSPEAK_APIS) {
      String key = server.arg("key");
      key.trim();
      strcpy(thingspeakAPIs[apiIdx].apiKey, key.c_str());
      thingspeakAPIs[apiIdx].interval = server.arg("interval").toInt() * 1000;
      thingspeakAPIs[apiIdx].active = server.arg("active") == "1";
      
      for (int i = 0; i < 8; i++) {
        String sArg = "s" + String(i);
        String vArg = "v" + String(i);
        if (server.hasArg(sArg)) {
          thingspeakAPIs[apiIdx].sensorIndex[i] = server.arg(sArg).toInt();
          thingspeakAPIs[apiIdx].valueIndex[i] = server.arg(vArg).toInt();
        }
      }
      
      saveConfig();
      server.send(200, "text/plain", "OK");
    }
    return;
  }
  
  String h = getHTMLHeader("ThingSpeak");
  h += getSidebar();
  h += "<main><h1><i data-lucide='cloud-upload'></i> Configuração ThingSpeak</h1>";
  
  // Conta quantas APIs estão ativas
  int activeCount = 0;
  for (int i = 0; i < MAX_THINGSPEAK_APIS; i++) {
    if (thingspeakAPIs[i].active) activeCount++;
  }
  
  // Mostra APIs ativas
  for (int i = 0; i < MAX_THINGSPEAK_APIS; i++) {
    if (thingspeakAPIs[i].active) {
      h += "<div class='card'>";
      h += "<h3>API " + String(i + 1) + "</h3>";
      h += "<label>API Key:</label>";
      h += "<input type='text' id='key" + String(i) + "' value='" + String(thingspeakAPIs[i].apiKey) + "' maxlength='20'>";
      h += "<label>Intervalo (segundos):</label>";
      h += "<input type='number' id='interval" + String(i) + "' value='" + String(thingspeakAPIs[i].interval / 1000) + "' min='15'>";
      
      h += "<h3>Campos (Fields)</h3>";
      for (int j = 0; j < 8; j++) {
        h += "<label>Field " + String(j + 1) + ":</label>";
        h += "<select id='s" + String(i) + "_" + String(j) + "' onchange='updateValTS(" + String(i) + "," + String(j) + ")'>";
        h += "<option value='-1'>Nenhum</option>";
        
        // Grupo de Sensores
        h += "<optgroup label='Sensores'>";
        for (int k = 0; k < sensorCount; k++) {
          h += "<option value='" + String(k) + "' " + String(thingspeakAPIs[i].sensorIndex[j] == k ? "selected" : "") + ">" + String(sensors[k].name) + "_" + String(k+1) + "</option>";
        }
        h += "</optgroup>";
        
        // Grupo de Cálculos
        h += "<optgroup label='Calculos'>";
        for (int k = 0; k < MAX_FORMULAS; k++) {
          if (formulas[k].active && strlen(formulas[k].formula) > 0 && formulas[k].formula[0] != '\0') {
            int calcIdx = 100 + k;
            h += "<option value='" + String(calcIdx) + "' " + String(thingspeakAPIs[i].sensorIndex[j] == calcIdx ? "selected" : "") + ">C" + String(k+1) + "</option>";
          }
        }
        h += "</optgroup>";
        h += "</select>";
        
        h += "<select id='v" + String(i) + "_" + String(j) + "'>";
        int curSensor = thingspeakAPIs[i].sensorIndex[j];
        if (curSensor >= 100) {
          h += "<option value='0' selected>Valor</option>";
        } else {
          h += "<option value='0' " + String(thingspeakAPIs[i].valueIndex[j] == 0 ? "selected" : "") + ">Val 1</option>";
          h += "<option value='1' " + String(thingspeakAPIs[i].valueIndex[j] == 1 ? "selected" : "") + ">Val 2</option>";
          h += "<option value='2' " + String(thingspeakAPIs[i].valueIndex[j] == 2 ? "selected" : "") + ">Val 3</option>";
          h += "<option value='3' " + String(thingspeakAPIs[i].valueIndex[j] == 3 ? "selected" : "") + ">Val 4</option>";
        }
        h += "</select>";
      }
      
      h += "<button onclick='saveAPI(" + String(i) + ")'>Salvar</button>";
      h += "<button onclick='deleteAPI(" + String(i) + ")' class='btn-danger'>Remover</button>";
      h += "</div>";
    }
  }
  
  // Botão para adicionar nova API
  if (activeCount < MAX_THINGSPEAK_APIS) {
    h += "<div class='card' style='border:2px dashed #03dac6'>";
    h += "<h3>+ Adicionar Nova API</h3>";
    h += "<div id='addForm' style='display:none'>";
    h += "<label>API Key:</label>";
    h += "<input type='text' id='newKey' maxlength='20'>";
    h += "<label>Intervalo (seg):</label>";
    h += "<input type='number' id='newInterval' value='15' min='15'>";
    h += "<button onclick='addNewAPI()'>Confirmar</button>";
    h += "<button onclick='hideAdd()'>Cancelar</button>";
    h += "</div>";
    h += "<button onclick='showAdd()' id='btnAdd'>+ Adicionar</button>";
    h += "</div>";
  }
  
  h += "<script>";
  h += "function showAdd(){document.getElementById('addForm').style.display='block';document.getElementById('btnAdd').style.display='none';}";
  h += "function hideAdd(){document.getElementById('addForm').style.display='none';document.getElementById('btnAdd').style.display='block';}";
  
  h += "function updateValTS(api,field){";
  h += "let s=document.getElementById('s'+api+'_'+field).value;";
  h += "let v=document.getElementById('v'+api+'_'+field);";
  h += "if(s>=100){v.innerHTML='<option value=\"0\">Valor</option>';v.disabled=true;}";
  h += "else{v.innerHTML='<option value=\"0\">Val 1</option><option value=\"1\">Val 2</option><option value=\"2\">Val 3</option><option value=\"3\">Val 4</option>';v.disabled=false;}";
  h += "}";
  
  h += "function addNewAPI(){";
  h += "let key=document.getElementById('newKey').value.trim();";
  h += "if(!key){alert('Digite API Key');return;}";
  h += "let d='api=" + String(activeCount) + "&key='+encodeURIComponent(key)+'&interval='+document.getElementById('newInterval').value+'&active=1';";
  h += "for(let i=0;i<8;i++)d+='&s'+i+'=-1&v'+i+'=0';";
  h += "fetch('/thingspeak',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  h += ".then(()=>{alert('Adicionada!');location.reload();});";
  h += "}";
  
  h += "function saveAPI(api){";
  h += "let d='api='+api+'&active=1&key='+encodeURIComponent(document.getElementById('key'+api).value)+'&interval='+document.getElementById('interval'+api).value;";
  h += "for(let i=0;i<8;i++){d+='&s'+i+'='+document.getElementById('s'+api+'_'+i).value+'&v'+i+'='+document.getElementById('v'+api+'_'+i).value;}";
  h += "fetch('/thingspeak',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  h += ".then(()=>alert('Salvo!'));";
  h += "}";
  
  h += "function deleteAPI(api){";
  h += "if(!confirm('Remover?'))return;";
  h += "let d='api='+api+'&active=0&key=&interval=15';";
  h += "for(let i=0;i<8;i++)d+='&s'+i+'=-1&v'+i+'=0';";
  h += "fetch('/thingspeak',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  h += ".then(()=>{alert('Removida!');location.reload();});";
  h += "}";
  h += "</script></main>";
  
  h += getHTMLFooter();
  server.send(200, "text/html", h);
}

void handleDisplayConfig() {
  if (server.method() == HTTP_POST) {
    displayConfig.sensor1 = server.arg("s1").toInt();
    displayConfig.value1 = server.arg("v1").toInt();
    displayConfig.sensor2 = server.arg("s2").toInt();
    displayConfig.value2 = server.arg("v2").toInt();
    displayConfig.sensor3 = server.arg("s3").toInt();
    displayConfig.value3 = server.arg("v3").toInt();
    saveConfig();
    server.send(200, "text/plain", "OK");
    return;
  }
  
  String h = getHTMLHeader("Display");
  h += getSidebar();
  h += "<main><h1><i data-lucide='monitor'></i> Configuração do Display</h1>";
  h += "<div class='card'><h3>Selecione 3 valores para exibir</h3>";
  
  for (int i = 1; i <= 3; i++) {
    int cs = (i == 1) ? displayConfig.sensor1 : (i == 2) ? displayConfig.sensor2 : displayConfig.sensor3;
    int cv = (i == 1) ? displayConfig.value1 : (i == 2) ? displayConfig.value2 : displayConfig.value3;
    
    h += "<label>Linha " + String(i) + ":</label>";
    h += "<select id='s" + String(i) + "' onchange='updateVal(" + String(i) + ")'>";
    h += "<option value='-1'>Nenhum</option>";
    
    // Grupo de Sensores
    h += "<optgroup label='Sensores'>";
    for (int j = 0; j < sensorCount; j++) {
      h += "<option value='" + String(j) + "' " + String(cs == j ? "selected" : "") + ">" + String(sensors[j].name) + "_" + String(j+1) + "</option>";
    }
    h += "</optgroup>";
    
    // Grupo de Cálculos
    h += "<optgroup label='Calculos'>";
    for (int j = 0; j < MAX_FORMULAS; j++) {
      if (formulas[j].active && strlen(formulas[j].formula) > 0) {
        int calcIdx = 100 + j;
        h += "<option value='" + String(calcIdx) + "' " + String(cs == calcIdx ? "selected" : "") + ">C" + String(j+1) + "</option>";
      }
    }
    h += "</optgroup>";
    h += "</select>";
    
    h += "<select id='v" + String(i) + "'>";
    // Se for cálculo (>= 100), não precisa de sub-valor
    if (cs >= 100) {
      h += "<option value='0' selected>Valor</option>";
    } else {
      h += "<option value='0' " + String(cv == 0 ? "selected" : "") + ">Valor 1</option>";
      h += "<option value='1' " + String(cv == 1 ? "selected" : "") + ">Valor 2</option>";
      h += "<option value='2' " + String(cv == 2 ? "selected" : "") + ">Valor 3</option>";
      h += "<option value='3' " + String(cv == 3 ? "selected" : "") + ">Valor 4</option>";
    }
    h += "</select>";
  }
  
  h += "<button onclick='save()'>Salvar</button></div>";
  h += "<script>";
  h += "function updateVal(line){";
  h += "let s=document.getElementById('s'+line).value;";
  h += "let v=document.getElementById('v'+line);";
  h += "if(s>=100){v.innerHTML='<option value=\"0\">Valor</option>';v.disabled=true;}";
  h += "else{v.innerHTML='<option value=\"0\">Valor 1</option><option value=\"1\">Valor 2</option><option value=\"2\">Valor 3</option><option value=\"3\">Valor 4</option>';v.disabled=false;}";
  h += "}";
  h += "function save(){";
  h += "let d='s1='+document.getElementById('s1').value";
  h += "+'&v1='+document.getElementById('v1').value";
  h += "+'&s2='+document.getElementById('s2').value";
  h += "+'&v2='+document.getElementById('v2').value";
  h += "+'&s3='+document.getElementById('s3').value";
  h += "+'&v3='+document.getElementById('v3').value;";
  h += "fetch('/display',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  h += ".then(()=>alert('Salvo!'));}";
  h += "</script></main>";
  h += getHTMLFooter();
  server.send(200, "text/html", h);
}

void handleAlarmConfig() {
  if (server.method() == HTTP_POST) {
    alarmConfig.sensorIndex = server.arg("sensor").toInt();
    alarmConfig.valueIndex = server.arg("value").toInt();
    alarmConfig.threshold = server.arg("threshold").toFloat();
    alarmConfig.above = server.arg("above") == "1";
    alarmConfig.useLED = server.arg("led") == "1";
    alarmConfig.useBuzzer = server.arg("buzzer") == "1";
    alarmConfig.active = server.arg("active") == "1";
    
    // Debug
    
    saveConfig();
    server.send(200, "text/plain", "OK");
    return;
  }
  
  String html = getHTMLHeader("Alarmes");
  html += getSidebar();
  html += "<main><h1><i data-lucide='bell'></i> Configuração de Alarmes</h1>";
  
  html += "<div class='card'>";
  html += "<h3>Configurar Alarme</h3>";
  
  html += "<label>Sensor:</label>";
  html += "<select id='sensor'>";
  for (int i = 0; i < sensorCount; i++) {
    html += "<option value='" + String(i) + "' " + String(alarmConfig.sensorIndex == i ? "selected" : "") + ">" + String(sensors[i].name) + "</option>";
  }
  html += "</select>";
  
  html += "<label>Valor:</label>";
  html += "<select id='value'>";
  html += "<option value='0'>Valor 1</option>";
  html += "<option value='1'>Valor 2</option>";
  html += "<option value='2'>Valor 3</option>";
  html += "<option value='3'>Valor 4</option>";
  html += "</select>";
  
  html += "<label>Limite:</label>";
  html += "<input type='number' step='0.01' id='threshold' value='" + String(alarmConfig.threshold) + "'>";
  
  html += "<label>Condicao:</label>";
  html += "<select id='above'>";
  html += "<option value='1' " + String(alarmConfig.above ? "selected" : "") + ">Acima do limite</option>";
  html += "<option value='0' " + String(!alarmConfig.above ? "selected" : "") + ">Abaixo do limite</option>";
  html += "</select>";
  
  html += "<label><input type='checkbox' id='led' " + String(alarmConfig.useLED ? "checked" : "") + "> LED (pisca 3x)</label>";
  html += "<label><input type='checkbox' id='buzzer' " + String(alarmConfig.useBuzzer ? "checked" : "") + "> Buzzer (continuo)</label>";
  html += "<label><input type='checkbox' id='active' " + String(alarmConfig.active ? "checked" : "") + "> Ativo</label>";
  
  html += "<button onclick='saveAlarm()'><i data-lucide='save'></i> Salvar</button>";
  html += "</div>";

  html += "<script>";
  html += "function saveAlarm(){";
  html += "let d='sensor='+document.getElementById('sensor').value";
  html += "+'&value='+document.getElementById('value').value";
  html += "+'&threshold='+document.getElementById('threshold').value";
  html += "+'&above='+document.getElementById('above').value";
  html += "+'&led='+(document.getElementById('led').checked?'1':'0')";
  html += "+'&buzzer='+(document.getElementById('buzzer').checked?'1':'0')";
  html += "+'&active='+(document.getElementById('active').checked?'1':'0');";
  html += "fetch('/alarm',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  html += ".then(()=>alert('Salvo!'));}";
  html += "</script></main>";
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}

// ==================== Calculation Functions ====================
void calculateFormulas() {
  // Para cada fórmula ativa
  for (int i = 0; i < MAX_FORMULAS; i++) {
    if (!formulas[i].active || strlen(formulas[i].formula) == 0 || formulas[i].formula[0] == '\0') {
      calculatedValues[i] = 0;
      continue;
    }
    
    // Copia a fórmula para um buffer
    String formula = String(formulas[i].formula);
    
    // Substitui VAR1 a VAR10
    for (int v = 0; v < MAX_VARIABLES; v++) {
      String varName = "VAR" + String(v + 1);
      String varValue = String(variables[v].value, 6);
      formula.replace(varName, varValue);
    }
    
    // Substitui C1 a C5
    for (int c = 0; c < MAX_FORMULAS; c++) {
      String calcName = "C" + String(c + 1);
      String calcValue = String(calculatedValues[c], 6);
      formula.replace(calcName, calcValue);
    }
    
    // Substitui sensores
    for (int s = 0; s < sensorCount; s++) {
      if (!sensors[s].active) continue;
      
      String sensorBase = String(sensors[s].name) + "_" + String(s + 1);
      
      if (strlen(sensors[s].label1) > 0) {
        String sensorName = sensorBase + "_1";
        String sensorValue = String(sensors[s].value1, 6);
        formula.replace(sensorName, sensorValue);
      }
      
      if (strlen(sensors[s].label2) > 0) {
        String sensorName = sensorBase + "_2";
        String sensorValue = String(sensors[s].value2, 6);
        formula.replace(sensorName, sensorValue);
      }
      
      if (strlen(sensors[s].label3) > 0) {
        String sensorName = sensorBase + "_3";
        String sensorValue = String(sensors[s].value3, 6);
        formula.replace(sensorName, sensorValue);
      }
      
      if (strlen(sensors[s].label4) > 0) {
        String sensorName = sensorBase + "_4";
        String sensorValue = String(sensors[s].value4, 6);
        formula.replace(sensorName, sensorValue);
      }
    }
    
    // Avalia a fórmula com valores substituídos
    int error;
    double result = te_interp(formula.c_str(), &error);
    
    if (error == 0) {
      calculatedValues[i] = (float)result;
    } else {
      calculatedValues[i] = 0;
    }
  }
}

void handleCalcValuesJSON() {
  String json = "{";
  
  // Variáveis VAR1-VAR10
  json += "\"vars\":[";
  for (int i = 0; i < MAX_VARIABLES; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"VAR" + String(i + 1) + "\",\"value\":" + String(variables[i].value, 4) + "}";
  }
  json += "],";
  
  // Cálculos C1-C5
  json += "\"calcs\":[";
  for (int i = 0; i < MAX_FORMULAS; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"C" + String(i + 1) + "\",\"value\":" + String(calculatedValues[i], 4) + ",\"formula\":\"" + String(formulas[i].formula) + "\",\"active\":" + String(formulas[i].active ? "true" : "false") + "}";
  }
  json += "],";
  
  // Sensores
  json += "\"sensors\":[";
  for (int i = 0; i < sensorCount; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + String(sensors[i].name) + "\",\"idx\":" + String(i + 1) + ",\"active\":" + String(sensors[i].active ? "true" : "false");
    json += ",\"v1\":" + String(sensors[i].value1, 2);
    json += ",\"v2\":" + String(sensors[i].value2, 2);
    json += ",\"v3\":" + String(sensors[i].value3, 2);
    json += ",\"v4\":" + String(sensors[i].value4, 2);
    json += "}";
  }
  json += "]";
  
  json += "}";
  server.send(200, "application/json", json);
}

void handleCalculations() {
  // Reset de cálculos
  if (server.method() == HTTP_POST && server.hasArg("reset")) {
    for (int i = 0; i < MAX_FORMULAS; i++) {
      memset(formulas[i].formula, 0, sizeof(formulas[i].formula));
      formulas[i].active = false;
      calculatedValues[i] = 0;
    }
    for (int i = 0; i < MAX_VARIABLES; i++) {
      variables[i].value = 0;
    }
    saveConfig();
    server.send(200, "text/plain", "OK");
    return;
  }
  
  if (server.method() == HTTP_POST) {
    // Salva fórmulas
    for (int i = 0; i < MAX_FORMULAS; i++) {
      String formulaArg = "formula" + String(i);
      if (server.hasArg(formulaArg)) {
        String formula = server.arg(formulaArg);
        formula.trim();
        if (formula.length() > 0 && formula != "") {
          strcpy(formulas[i].formula, formula.c_str());
          formulas[i].active = true;
        } else {
          memset(formulas[i].formula, 0, sizeof(formulas[i].formula));
          formulas[i].active = false;
        }
      }
    }
    
    // Salva variáveis
    for (int i = 0; i < MAX_VARIABLES; i++) {
      String varArg = "var" + String(i);
      if (server.hasArg(varArg)) {
        variables[i].value = server.arg(varArg).toFloat();
      }
    }
    
    saveConfig();
    server.send(200, "text/plain", "OK");
    return;
  }
  
  String h = getHTMLHeader("Cálculos");
  h += getSidebar();
  h += "<main><h1><i data-lucide='calculator'></i> Configuração de Cálculos</h1>";
  
  // Info sobre variáveis
  h += "<article>";
  h += "<header>Variáveis Disponíveis</header>";
  h += "<p><strong>Sensores:</strong> ";
  for (int i = 0; i < sensorCount; i++) {
    if (!sensors[i].active) continue;
    h += String(sensors[i].name) + "_" + String(i + 1) + "_1";
    if (strlen(sensors[i].label2) > 0) h += ", " + String(sensors[i].name) + "_" + String(i + 1) + "_2";
    if (strlen(sensors[i].label3) > 0) h += ", " + String(sensors[i].name) + "_" + String(i + 1) + "_3";
    if (strlen(sensors[i].label4) > 0) h += ", " + String(sensors[i].name) + "_" + String(i + 1) + "_4";
    h += " | ";
  }
  h += "</p>";
  h += "<p><strong>Variáveis:</strong> VAR1 a VAR10 | <strong>Cálculos:</strong> C1 a C5</p>";
  h += "<p><strong>Operadores:</strong> + - * / ^ ( )</p>";
  h += "</article>";
  
  // Variáveis personalizadas
  h += "<article><header>Variáveis Personalizadas</header>";
  h += "<div class='grid'>";
  for (int i = 0; i < MAX_VARIABLES; i++) {
    h += "<label>VAR" + String(i + 1);
    h += "<input type='number' step='0.01' id='var" + String(i) + "' value='" + String(variables[i].value, 2) + "'></label>";
  }
  h += "</div></article>";
  
  // Lista de fórmulas existentes
  h += "<article><header>Fórmulas Ativas</header>";
  
  // Conta quantas fórmulas ativas existem
  int activeCount = 0;
  for (int i = 0; i < MAX_FORMULAS; i++) {
    if (formulas[i].active && strlen(formulas[i].formula) > 0 && formulas[i].formula[0] != '\0') {
      activeCount++;
    }
  }
  
  if (activeCount == 0) {
    h += "<p>Nenhuma fórmula configurada. Clique em 'Adicionar' para criar.</p>";
  } else {
    // Mostra apenas fórmulas existentes
    for (int i = 0; i < MAX_FORMULAS; i++) {
      if (formulas[i].active && strlen(formulas[i].formula) > 0 && formulas[i].formula[0] != '\0') {
        h += "<details open><summary>C" + String(i + 1) + " = " + String(calculatedValues[i], 2) + "</summary>";
        h += "<label>Fórmula<input type='text' id='formula" + String(i) + "' value='" + String(formulas[i].formula) + "'></label>";
        h += "<button onclick='del(" + String(i) + ")' class='secondary'>Deletar</button>";
        h += "</details>";
      }
    }
  }
  
  // Botão adicionar nova fórmula (se houver slots disponíveis)
  if (activeCount < MAX_FORMULAS) {
    h += "<button onclick='add()'>+ Adicionar Fórmula</button>";
  }
  
  h += "</article>";
  
  // Área para nova fórmula (oculta inicialmente)
  h += "<dialog id='newForm'>";
  h += "<article><header>Nova Fórmula</header>";
  h += "<label>Slot<select id='newSlot'>";
  for (int i = 0; i < MAX_FORMULAS; i++) {
    if (!formulas[i].active || strlen(formulas[i].formula) == 0 || formulas[i].formula[0] == '\0') {
      h += "<option value='" + String(i) + "'>C" + String(i + 1) + "</option>";
    }
  }
  h += "</select></label>";
  h += "<label>Fórmula<input type='text' id='newFormula' placeholder='Ex: (BMP280_1_1*2)+VAR1'></label>";
  h += "<button onclick='saveNew()'>Salvar</button>";
  h += "<button onclick='cancel()' class='secondary'>Cancelar</button>";
  h += "</article></dialog>";
  
  h += "<button onclick='save()' style='margin:20px 0'>💾 Salvar Alterações</button>";
  
  // Botão de reset
  h += "<article class='danger-zone'>";
  h += "<header>Zona de Perigo</header>";
  h += "<p>Resetar todos os cálculos e variáveis (não afeta sensores ou outras configurações)</p>";
  h += "<button onclick='resetCalcs()' class='secondary'>🗑️ Resetar Cálculos</button>";
  h += "</article>";
  
  h += "<script>";
  h += "function add(){document.getElementById('newForm').showModal();}";
  h += "function cancel(){document.getElementById('newForm').close();document.getElementById('newFormula').value='';}";
  
  h += "function resetCalcs(){";
  h += "if(!confirm('ATENÇÃO: Isso vai deletar TODAS as fórmulas e variáveis!'))return;";
  h += "if(!confirm('Tem certeza? Esta ação não pode ser desfeita!'))return;";
  h += "fetch('/calculations',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'reset=1'})";
  h += ".then(r=>r.text()).then(()=>{alert('Cálculos resetados!');location.reload();});";
  h += "}";
  
  h += "function del(i){";
  h += "if(!confirm('Deletar C'+(i+1)+'?'))return;";
  h += "let d='';";
  for (int i = 0; i < MAX_FORMULAS; i++) {
    h += "if(" + String(i) + "==i)d+='formula" + String(i) + "=&';";
    h += "else{let f=document.getElementById('formula" + String(i) + "');";
    h += "d+='formula" + String(i) + "='+(f?encodeURIComponent(f.value):'')+'&';}";
  }
  for (int i = 0; i < MAX_VARIABLES; i++) {
    h += "d+='var" + String(i) + "='+document.getElementById('var" + String(i) + "').value+'&';";
  }
  h += "fetch('/calculations',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  h += ".then(r=>r.text()).then(()=>{alert('Deletado!');location.reload();});";
  h += "}";
  
  h += "function saveNew(){";
  h += "let nf=document.getElementById('newFormula').value;";
  h += "if(!nf){alert('Digite uma fórmula');return;}";
  h += "let slot=document.getElementById('newSlot').value;";
  h += "let d='';";
  for (int i = 0; i < MAX_FORMULAS; i++) {
    h += "if(" + String(i) + "==slot)d+='formula" + String(i) + "='+encodeURIComponent(nf)+'&';";
    h += "else{let f=document.getElementById('formula" + String(i) + "');";
    h += "d+='formula" + String(i) + "='+(f?encodeURIComponent(f.value):'')+'&';}";
  }
  for (int i = 0; i < MAX_VARIABLES; i++) {
    h += "d+='var" + String(i) + "='+document.getElementById('var" + String(i) + "').value+'&';";
  }
  h += "fetch('/calculations',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  h += ".then(r=>r.text()).then(()=>{alert('Salvo!');location.reload();});";
  h += "}";
  
  h += "function save(){";
  h += "let d='';";
  for (int i = 0; i < MAX_FORMULAS; i++) {
    h += "let f" + String(i) + "=document.getElementById('formula" + String(i) + "');";
    h += "d+='formula" + String(i) + "='+(f" + String(i) + "?encodeURIComponent(f" + String(i) + ".value):'')+'&';";
  }
  for (int i = 0; i < MAX_VARIABLES; i++) {
    h += "d+='var" + String(i) + "='+document.getElementById('var" + String(i) + "').value+'&';";
  }
  h += "fetch('/calculations',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:d})";
  h += ".then(r=>r.text()).then(()=>{alert('Salvo!');location.reload();});";
  h += "}";
  
  h += "</script></main>";
  h += getHTMLFooter();
  server.send(200, "text/html", h);
}

void handleLogs() {
  // Se receber POST para adicionar sensor simulado
  if (server.method() == HTTP_POST && server.hasArg("addsim")) {
    addSimulatedSensor();
    server.send(200, "text/plain", "OK");
    return;
  }
  
  // Se receber POST, é para fazer reset
  if (server.method() == HTTP_POST && server.hasArg("reset")) {
    
    // Limpa EEPROM
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0xFF);
    }
    
    if (EEPROM.commit()) {
      server.send(200, "text/plain", "OK");
      delay(1000);
      ESP.restart();
    } else {
      server.send(500, "text/plain", "ERRO");
    }
    return;
  }
  
  String html = getHTMLHeader("Logs");
  html += getSidebar();
  html += "<main><h1><i data-lucide='activity'></i> Logs do Sistema</h1>";

  html += "<div class='card'>";
  html += "<h3><i data-lucide='server'></i> Status do Sistema</h3>";
  html += "<div style='display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:.75rem;margin-top:.75rem'>";
  html += "<div><div class='lbl'>WiFi</div><div style='color:" + String(WiFi.status() == WL_CONNECTED ? "#22d3ee" : "#f87171") + ";font-weight:600'>" + String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado") + "</div></div>";
  html += "<div><div class='lbl'>IP</div><div style='color:#e2e8f0'>" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "</div></div>";
  html += "<div><div class='lbl'>RSSI</div><div style='color:#e2e8f0'>" + String(WiFi.RSSI()) + " dBm</div></div>";
  html += "<div><div class='lbl'>Sensores</div><div style='color:#e2e8f0'>" + String(sensorCount) + "</div></div>";
  html += "<div><div class='lbl'>Mem. Livre</div><div style='color:#e2e8f0'>" + String(ESP.getFreeHeap()) + " B</div></div>";
  html += "<div><div class='lbl'>Uptime</div><div style='color:#e2e8f0'>" + String(millis() / 1000) + " s</div></div>";
  html += "</div></div>";
  
  html += "<div class='card'>";
  html += "<h3><i data-lucide='cpu'></i> Sensores Ativos</h3>";
  for (int i = 0; i < sensorCount; i++) {
    html += "<p>" + String(i + 1) + ". " + String(sensors[i].name) + " (" + String(sensors[i].type) + ")";
    if (strcmp(sensors[i].name, "SIMSENSOR") == 0) {
      html += " <span style='color:#03dac6'>✓ SIMULADO</span>";
    }
    html += "</p>";
  }
  html += "<button onclick='location.href=\"/rescan\"' style='margin-top:.75rem'><i data-lucide='scan-search'></i> Redetectar Sensores</button>";
  
  // Verifica se já existe sensor simulado
  bool hasSimulated = false;
  for (int i = 0; i < sensorCount; i++) {
    if (strcmp(sensors[i].name, "SIMSENSOR") == 0) {
      hasSimulated = true;
      break;
    }
  }
  
  if (!hasSimulated && sensorCount < MAX_SENSORS) {
    html += "<button onclick='addSimSensor()' style='margin-left:8px;background:linear-gradient(135deg,#7c3aed,#6366f1)!important'><i data-lucide='plus-circle'></i> Sensor Simulado</button>";
  }
  html += "</div>";

  html += "<div class='danger-zone'>";
  html += "<h3><i data-lucide='triangle-alert'></i> Zona de Perigo</h3>";
  html += "<p style='color:#fca5a5'>Apaga TODAS as configurações: WiFi, sensores, calibrações, ThingSpeak, alarmes.</p>";
  html += "<button onclick='confirmReset()' class='danger'><i data-lucide='trash-2'></i> RESETAR TUDO</button>";
  html += "</div>";

  html += "<script>";
  html += "function addSimSensor(){if(!confirm('Adicionar sensor simulado para testes?'))return;";
  html += "fetch('/logs',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'addsim=1'})";
  html += ".then(r=>{if(r.ok){alert('Sensor simulado adicionado!');location.reload();}else alert('ERRO!');});}";
  html += "function confirmReset(){if(!confirm('TEM CERTEZA? Apagara TUDO!'))return;";
  html += "if(!confirm('ULTIMA CHANCE!'))return;";
  html += "fetch('/logs',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'reset=1'})";
  html += ".then(r=>{if(r.ok){alert('Reset! ESP reiniciando...');setTimeout(()=>location.href='/',3000);}else alert('ERRO!');});}";
  html += "</script></main>";
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}

void handleRescan() {
  detectSensors();
  server.sendHeader("Location", "/logs");
  server.send(303);
}

// ==================== HTML Templates ====================
String getHTMLHeader(String title) {
  String h = "<!DOCTYPE html><html data-theme='dark'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + "</title>";
  h += "<link rel='stylesheet' href='https://cdn.jsdelivr.net/npm/@picocss/pico@2/css/pico.min.css'>";
  h += "<script src='https://unpkg.com/lucide@latest/dist/umd/lucide.min.js'></script>";
  h += "<style>";
  h += ":root{--a:#6366f1;--b:#22d3ee}";
  h += "body{background:linear-gradient(135deg,#0f0f1a,#1a0f2e 50%,#0f1a2e);min-height:100vh;padding:0}";
  h += "main{padding:1.25rem;max-width:1200px;margin:0 auto}";
  h += "nav{background:rgba(15,15,26,.88);backdrop-filter:blur(20px);border-bottom:1px solid rgba(255,255,255,.08);padding:.5rem 1.25rem;position:sticky;top:0;z-index:99}";
  h += "nav a{color:#a5b4fc!important;transition:color .2s}nav a:hover{color:#fff!important;text-decoration:none!important}";
  h += "nav ul{margin:0}";
  h += "article{background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.08);border-radius:1rem;transition:transform .2s,box-shadow .2s;margin-bottom:1rem}";
  h += "article:hover{transform:translateY(-2px);box-shadow:0 8px 32px rgba(99,102,241,.15)}";
  h += "article>header{background:rgba(99,102,241,.08);border-radius:1rem 1rem 0 0;border-bottom:1px solid rgba(255,255,255,.08)}";
  h += ".card{background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.08);border-radius:1rem;padding:1.25rem;margin-bottom:1rem}";
  h += ".val{font-size:1.8rem;font-weight:700;background:linear-gradient(135deg,var(--a),var(--b));-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;line-height:1.1}";
  h += ".lbl{font-size:.7rem;color:#94a3b8;text-transform:uppercase;letter-spacing:.06em;margin-bottom:.2rem}";
  h += ".unit{font-size:.8rem;color:#64748b;margin-left:2px}";
  h += "h1{color:#e2e8f0;font-weight:700;margin-bottom:1.25rem}";
  h += "h2,h3{color:#c7d2fe}";
  h += "button,[type=submit]{background:linear-gradient(135deg,var(--a),#818cf8)!important;border:none!important;border-radius:.5rem!important;color:#fff!important;transition:opacity .2s,transform .1s!important}";
  h += "button:hover{opacity:.88!important;transform:translateY(-1px)!important}";
  h += ".danger{background:linear-gradient(135deg,#ef4444,#dc2626)!important}";
  h += ".secondary{background:rgba(99,102,241,.2)!important}";
  h += ".danger-zone{border:1px solid rgba(239,68,68,.3);background:rgba(239,68,68,.05);border-radius:1rem;padding:1rem}";
  h += "input,select,textarea{background:rgba(255,255,255,.06)!important;border:1px solid rgba(255,255,255,.12)!important;color:#e2e8f0!important;border-radius:.5rem!important}";
  h += "input:focus,select:focus{border-color:var(--a)!important;box-shadow:0 0 0 3px rgba(99,102,241,.2)!important;outline:none!important}";
  h += "label{color:#cbd5e1}";
  h += ".lucide{width:16px;height:16px;vertical-align:middle;margin-right:4px;stroke:currentColor}";
  h += "article header .lucide,h1 .lucide,h2 .lucide,h3 .lucide{width:18px;height:18px}";
  h += "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}";
  h += ".dot{width:8px;height:8px;border-radius:50%;background:var(--b);display:inline-block;animation:pulse 2s infinite;margin-right:6px;vertical-align:middle}";
  h += "optgroup{color:#94a3b8}option{background:#1e1b4b;color:#e2e8f0}";
  h += "details summary{color:#c7d2fe}dialog article{border-radius:1rem}";
  h += "</style></head><body>";
  return h;
}

String getHTMLFooter() {
  return "<script>if(typeof lucide!=='undefined')lucide.createIcons()</script></body></html>";
}

String getSidebar() {
  String s = "<nav><ul><li><strong style='background:linear-gradient(135deg,#6366f1,#22d3ee);-webkit-background-clip:text;-webkit-text-fill-color:transparent;font-size:1.1rem'>&#9889; ESP Sensor Hub</strong></li></ul><ul>";
  s += "<li><a href='/'><i data-lucide='layout-dashboard'></i>Dashboard</a></li>";
  s += "<li><a href='/wifi'><i data-lucide='wifi'></i>WiFi</a></li>";
  s += "<li><a href='/calibrate'><i data-lucide='sliders-horizontal'></i>Calibração</a></li>";
  s += "<li><a href='/thingspeak'><i data-lucide='cloud-upload'></i>ThingSpeak</a></li>";
  s += "<li><a href='/display'><i data-lucide='monitor'></i>Display</a></li>";
  s += "<li><a href='/alarm'><i data-lucide='bell'></i>Alarmes</a></li>";
  s += "<li><a href='/calculations'><i data-lucide='calculator'></i>Cálculos</a></li>";
  s += "<li><a href='/logs'><i data-lucide='activity'></i>Logs</a></li>";
  s += "</ul></nav>";
  return s;
}
