/*
    DIY Arduino ELRS Controller
    Created by Novosleov https://github.com/NovoselovMilk/Arduino-ELRS-Controller
*/

#include <EEPROM.h>

/*  Для отладки и проверки верного подключения стиков и кнопок можно раскомментировать строкчку "#define DEBUG" и открыть COM порт со скоростью 9600
  Sticks AETR(AIL,ELE,THR,RUD) - Значения показаний стиков от 172 до 1811
  (при движении стика вправо значения должны увеличиваться, влево - уменьшаться, вверх - увеличиваться, вниз - уменьшаться )
  Trim value (AER) - Значение триммеров от -100 до +100
*/
//#define DEBUG

// Отслеживание напряжения однобаночного аккумулятора. При питании от батареек, необходимо закомментировать эту строчку.
#define BAT_CONTROL

// Расходы по каналам
#define RATE_AIL    100
#define RATE_ELE    100
#define RATE_THR    100
#define RATE_RUD    100

// REVERSES
#define INV_AIL      0
#define INV_ELE      0
#define INV_THR      0
#define INV_RUD      0

// Пины подключения стиков к аналоговым входам
#define PIN_AIL      A0
#define PIN_ELE      A1
#define PIN_THR      A4
#define PIN_RUD      A5

#define CH_MAX        5

#define PIN_CH5       8
#define PIN_BZZ       9
#define PIN_BAT       A7

// Пины подключения кнопок триммеров
#define PIN_AIL_PLUS  5
#define PIN_AIL_MINUS 4
#define PIN_ELE_PLUS  2
#define PIN_ELE_MINUS 3
#define PIN_RUD_MINUS 6
#define PIN_RUD_PLUS  7

#define BAT_MIN 3.3

#define TIMELOOP 1666
#define DEBOUNCE 200

#define EE_INIT 0xEE
#define EE_ADDR_INIT 0
#define EE_ADDR_AIL_TRIM 1
#define EE_ADDR_ELE_TRIM 2
#define EE_ADDR_RUD_TRIM 3
#define EE_ADDR_DIR_INPUT 4
#define EE_ADDR_MIN_AIL_IN 5
#define EE_ADDR_MAX_AIL_IN 7
#define EE_ADDR_MIN_ELE_IN 9
#define EE_ADDR_MAX_ELE_IN 11
#define EE_ADDR_MIN_THR_IN 13
#define EE_ADDR_MAX_THR_IN 15
#define EE_ADDR_MIN_RUD_IN 17
#define EE_ADDR_MAX_RUD_IN 19

#define CRSF_TIME_BETWEEN_FRAMES_US     1666 // 1.6 ms 500Hz
//#define CRSF_PAYLOAD_OFFSET             offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE            128
#define CRSF_MSP_TX_BUF_SIZE            128
#define CRSF_PAYLOAD_SIZE_MAX           60
#define CRSF_PACKET_LENGTH              22
#define CRSF_PACKET_SIZE                26
#define CRSF_FRAME_LENGTH               24 // length of type + payload + crc
#define CRSF_CMD_PACKET_SIZE            8

// Basic setup
#define CRSF_MAX_CHANNEL        16
#define CRSF_FRAME_SIZE_MAX     64
// Device address & type
#define RADIO_ADDRESS           0xEA
// #define ADDR_MODULE          0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS           0x16

// Define AUX channel input limite
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

// ELRS command
#define ELRS_ADDRESS                    0xEE
#define ELRS_BIND_COMMAND               0xFF
#define ELRS_WIFI_COMMAND               0x0E
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TLM_RATIO_COMMAND          0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x04
#define ELRS_POWER_COMMAND              0x06
#define ELRS_BLE_JOYSTIC_COMMAND        17
#define TYPE_SETTINGS_WRITE             0x2D
#define ADDR_RADIO                      0xEA
#define port                            Serial

#define PWR_25MW                1
#define PWR_50MW                2

#define RATE_25HZ                1
#define RATE_50HZ                2
#define RATE_100HZ               3
#define RATE_100HZ_FULL          4
#define RATE_200HZ               5

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
uint8_t pkt_rate = 0;
uint8_t pwr;

uint8_t input_pins[4] = {PIN_AIL, PIN_ELE, PIN_THR, PIN_RUD};
uint8_t rate_value[4] = {RATE_AIL, RATE_ELE, RATE_THR, RATE_RUD};
int8_t trim_value[4] = {0, 0, 0, 0};              // переменные для работы триммеров
uint16_t calibrate_value[8] = {0, 1023, 0, 1023, 0, 1023, 0, 1023};


bool inv_flag[4] = {INV_AIL, INV_ELE, INV_THR, INV_RUD};

bool ch1_flag_plus;
bool ch1_flag_minus;
bool ch2_flag_plus;
bool ch2_flag_minus;
bool dir_sticks[4];

bool ch1_flag_center = 1;
bool ch2_flag_center = 1;

uint32_t last_time = micros();
uint32_t last_press = millis();
uint32_t last_bat_time = millis();
uint32_t crsfTime = 0;

bool change_settings = false;
bool noPulses = false;

int loopCount = 0;


enum {AIL = 0, ELE, THR, RUD};
enum {TRIM_AIL = 0, TRIM_ELE, TRIM_THR, TRIM_RUD};
enum {MIN_AIL = 0, MAX_AIL, MIN_ELE, MAX_ELE, MIN_THR, MAX_THR, MIN_RUD, MAX_RUD};




void setup() {

  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;


  init_eeprom();

#ifdef DEBUG
  Serial.begin(9600);
#else
  Serial.begin(400000);
#endif

  delay(100);

  pinMode(PIN_CH5, INPUT_PULLUP);
  pinMode(PIN_AIL_PLUS, INPUT_PULLUP);
  pinMode(PIN_AIL_MINUS, INPUT_PULLUP);
  pinMode(PIN_ELE_PLUS, INPUT_PULLUP);
  pinMode(PIN_ELE_MINUS, INPUT_PULLUP);
  pinMode(PIN_RUD_MINUS, INPUT_PULLUP);
  pinMode(PIN_RUD_PLUS, INPUT_PULLUP);

  if (!digitalRead(PIN_ELE_MINUS) && !digitalRead(PIN_AIL_MINUS)) {
    if (calibrate_sticks()) {
      tone(PIN_BZZ, 2500, 100);
      delay(200);
      tone(PIN_BZZ, 2500, 100);
      delay(200);
    }
  } else if (!digitalRead(PIN_RUD_PLUS)) {
    noPulses = true;
  }
  delay(1000);
  change_settings_elrs();

  tone(PIN_BZZ, 2100, 100);
}


void loop() {

#ifdef DEBUG
  if ((millis() - last_time) >= 20) {
    last_time = millis();
    dataRead();
    trimer();
    print_info();
  }
  if ((millis() - last_bat_time) >= 10000) {
    last_bat_time = millis();
    bat_control();
  }
#else

  if ((micros() - last_time) >= TIMELOOP) {
    last_time = micros();

    if (!noPulses) {
      crsfPrepareDataPacket(crsfPacket, rcChannels);
      CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
      dataRead();
      trimer();
    }

  }

#ifdef BAT_CONTROL
  if ((millis() - last_bat_time) >= 10000) {
    last_bat_time = millis();
    bat_control();
  }
#endif

#endif

}


void trimer() {
  if (!digitalRead(PIN_ELE_PLUS) && millis() - last_press >= DEBOUNCE) {
    last_press = millis();
    if (trim_value[TRIM_ELE] <  100) {
      trim_value[TRIM_ELE] += 10;
      EEPROM.write(EE_ADDR_ELE_TRIM, trim_value[TRIM_ELE]);
    } else  tone(PIN_BZZ, 1900, 200);
    if (trim_value[TRIM_ELE] == 0) tone(PIN_BZZ, 2200, 100);
  }

  if (!digitalRead(PIN_ELE_MINUS) && millis() - last_press >= DEBOUNCE) {
    last_press = millis();
    if (trim_value[TRIM_ELE] > -100) {
      trim_value[TRIM_ELE] -= 10;
      EEPROM.write(EE_ADDR_ELE_TRIM, trim_value[TRIM_ELE]);
    } else  tone(PIN_BZZ, 1900, 200);
    if (trim_value[TRIM_ELE] == 0) tone(PIN_BZZ, 2200, 100);
  }

  if (!digitalRead(PIN_AIL_PLUS) && millis() - last_press >= DEBOUNCE) {
    last_press = millis();
    if (trim_value[TRIM_AIL] <  100) {
      trim_value[TRIM_AIL] += 10;
      EEPROM.write(EE_ADDR_AIL_TRIM, trim_value[TRIM_AIL]);
    } else  tone(PIN_BZZ, 1900, 200);
    if (trim_value[TRIM_AIL] == 0) tone(PIN_BZZ, 2200, 100);
  }

  if (!digitalRead(PIN_AIL_MINUS) && millis() - last_press >= DEBOUNCE) {
    last_press = millis();
    if (trim_value[TRIM_AIL] > -100) {
      trim_value[TRIM_AIL] -= 10;
      EEPROM.write(EE_ADDR_AIL_TRIM, trim_value[TRIM_AIL]);
    } else  tone(PIN_BZZ, 1900, 200);
    if (trim_value[TRIM_AIL] == 0) tone(PIN_BZZ, 2200, 100);
  }

  if (!digitalRead(PIN_RUD_PLUS) && millis() - last_press >= DEBOUNCE) {
    last_press = millis();
    if (trim_value[TRIM_RUD] <  100) {
      trim_value[TRIM_RUD] += 10;
      EEPROM.write(EE_ADDR_RUD_TRIM, trim_value[TRIM_RUD]);
    } else  tone(PIN_BZZ, 1900, 200);
    if (trim_value[TRIM_RUD] == 0) tone(PIN_BZZ, 2200, 100);
  }

  if (!digitalRead(PIN_RUD_MINUS) && millis() - last_press >= DEBOUNCE) {
    last_press = millis();
    if (trim_value[TRIM_RUD] > -100) {
      trim_value[TRIM_RUD] -= 10;
      EEPROM.write(EE_ADDR_RUD_TRIM, trim_value[TRIM_RUD]);
    } else  tone(PIN_BZZ, 1900, 200);
    if (trim_value[TRIM_RUD] == 0) tone(PIN_BZZ, 2200, 100);
  }
}


void dataRead() {

  for (uint8_t i = 0; i < (CH_MAX - 1); i++) {
    int16_t read_value = 0;
    uint16_t raw;

    if (dir_sticks[i]) read_value = analogRead(input_pins[i]);
    else  {
      raw = analogRead(input_pins[i]);
      raw = ~raw & 0x3FF;
      read_value = raw;
    }

    read_value = (map(read_value, calibrate_value[i * 2], calibrate_value[(i * 2) + 1], 0, 1023)) + trim_value[i] ;

    read_value = constrain(read_value, 0, 1023);
    if (inv_flag[i]) read_value = (~read_value) & 0x3FF;
    read_value = (read_value * (0.01 * rate_value[i])) + (1023 - (1023 * 0.01 * rate_value[i])) / 2;

    rcChannels[i] = map(read_value, 0, 1023, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
  }

  rcChannels[4] = (!digitalRead(PIN_CH5)) ? CRSF_DIGITAL_CHANNEL_MIN : CRSF_DIGITAL_CHANNEL_MAX;
}

void bat_control() {
  float vbat = (float)(analogRead(A7) * 5.0) / 1024;
  if (vbat < BAT_MIN) tone(PIN_BZZ, 1500, 800);
}

bool calibrate_sticks() {
  tone(PIN_BZZ, 2300, 300);
  delay(500);
  tone(PIN_BZZ, 2300, 300);
  delay(500);
  tone(PIN_BZZ, 2300, 300);
  delay(500);

  for (int i = 0; i < 8; i++) {
    calibrate_value[i] = 512;
  }

  uint16_t last_raw;
  uint16_t raw;
  bool dir_set = false;
  int16_t moution = 0;

  for (int i = 0; i < 4; i++) {
    dir_set = false;
    last_raw = analogRead(input_pins[i]);
    raw = analogRead(input_pins[i]);

    while (!dir_set) {
      raw = analogRead(input_pins[i]);
      moution = raw - last_raw;
      if (moution > 7) {
        dir_sticks[i] = true;
        dir_set = true;
      } else if (moution < -7) {
        dir_sticks[i] = false;
        dir_set = true;
      }
      last_raw = raw;
      delay(20);
    }

    for (int j = 0; j < 400; j++) {
      raw = analogRead(input_pins[i]);
      if (raw < calibrate_value[i * 2]) {
        calibrate_value[i * 2] = raw;
      }
      if (raw > calibrate_value[(i * 2) + 1]) calibrate_value[(i * 2) + 1] = raw;

      delay(10);
    }
    switch (i) {
      case AIL :
        if (dir_sticks[i]) EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) | 0x01) );
        else EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) & 0xFE) );

        EEPROM.put(EE_ADDR_MIN_AIL_IN, calibrate_value[MIN_AIL]);
        EEPROM.put(EE_ADDR_MAX_AIL_IN, calibrate_value[MAX_AIL]);
        delay(20);

      case ELE :
        if (dir_sticks[i]) EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) | 0x02));
        else EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) & 0xFD) );

        EEPROM.put(EE_ADDR_MIN_ELE_IN, calibrate_value[MIN_ELE]);
        EEPROM.put(EE_ADDR_MAX_ELE_IN, calibrate_value[MAX_ELE]);
        delay(20);

      case THR :
        if (dir_sticks[i]) EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) | 0x04) );
        else EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) & 0xFB));

        EEPROM.put(EE_ADDR_MIN_THR_IN, calibrate_value[MIN_THR]);
        EEPROM.put(EE_ADDR_MAX_THR_IN, calibrate_value[MAX_THR]);
        delay(20);

      case RUD :
        if (dir_sticks[i]) EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) | 0x08));
        else EEPROM.write(EE_ADDR_DIR_INPUT, (EEPROM.read(EE_ADDR_DIR_INPUT) & 0xF7));

        EEPROM.put(EE_ADDR_MIN_RUD_IN, calibrate_value[MIN_RUD]);
        EEPROM.put(EE_ADDR_MAX_RUD_IN, calibrate_value[MAX_RUD]);
        delay(20);
    }

    tone(PIN_BZZ, 2000, 30);
  }




#ifdef DEBUG
  for (int i = 0; i < 8; i++) {
    Serial.print(calibrate_value[i]);
    Serial.print("\t");
  }
  Serial.println();
#endif
  delay(500);
  return true;
}

void init_eeprom() {
  if (EEPROM.read(EE_ADDR_INIT) != EE_INIT) {
    EEPROM.write(EE_ADDR_AIL_TRIM, trim_value[TRIM_AIL]);
    EEPROM.write(EE_ADDR_ELE_TRIM, trim_value[TRIM_ELE]);
    EEPROM.write(EE_ADDR_RUD_TRIM, trim_value[TRIM_RUD]);
    EEPROM.write(EE_ADDR_DIR_INPUT, 0xFF);
    EEPROM.put(EE_ADDR_MIN_AIL_IN, calibrate_value[MIN_AIL]);
    EEPROM.put(EE_ADDR_MAX_AIL_IN, calibrate_value[MAX_AIL]);
    EEPROM.put(EE_ADDR_MIN_ELE_IN, calibrate_value[MIN_ELE]);
    EEPROM.put(EE_ADDR_MAX_ELE_IN, calibrate_value[MAX_ELE]);
    EEPROM.put(EE_ADDR_MIN_THR_IN, calibrate_value[MIN_THR]);
    EEPROM.put(EE_ADDR_MAX_THR_IN, calibrate_value[MAX_THR]);
    EEPROM.put(EE_ADDR_MIN_RUD_IN, calibrate_value[MIN_RUD]);
    EEPROM.put(EE_ADDR_MAX_RUD_IN, calibrate_value[MAX_RUD]);

    EEPROM.write(EE_ADDR_INIT, EE_INIT);
  }

  trim_value[TRIM_AIL] = EEPROM.read(EE_ADDR_AIL_TRIM);
  trim_value[TRIM_ELE] = EEPROM.read(EE_ADDR_ELE_TRIM);
  trim_value[TRIM_RUD] = EEPROM.read(EE_ADDR_RUD_TRIM);

  dir_sticks[AIL] =  EEPROM.read(EE_ADDR_DIR_INPUT) & 0x01;
  dir_sticks[ELE] = (EEPROM.read(EE_ADDR_DIR_INPUT) >> 1) & 0x01;
  dir_sticks[THR] = (EEPROM.read(EE_ADDR_DIR_INPUT) >> 2) & 0x01;
  dir_sticks[RUD] = (EEPROM.read(EE_ADDR_DIR_INPUT) >> 3) & 0x01;

  EEPROM.get(EE_ADDR_MIN_AIL_IN, calibrate_value[MIN_AIL]);
  EEPROM.get(EE_ADDR_MAX_AIL_IN, calibrate_value[MAX_AIL]);
  EEPROM.get(EE_ADDR_MIN_ELE_IN, calibrate_value[MIN_ELE]);
  EEPROM.get(EE_ADDR_MAX_ELE_IN, calibrate_value[MAX_ELE]);
  EEPROM.get(EE_ADDR_MIN_THR_IN, calibrate_value[MIN_THR]);
  EEPROM.get(EE_ADDR_MAX_THR_IN, calibrate_value[MAX_THR]);

  EEPROM.get(EE_ADDR_MIN_RUD_IN, calibrate_value[MIN_RUD]);
  EEPROM.get(EE_ADDR_MAX_RUD_IN, calibrate_value[MAX_RUD]);
}


void change_settings_elrs() {


  for ( int i = 0 ; i < 4; i++) {
    int16_t read_value = 0;
    uint16_t raw;

    if (dir_sticks[i]) read_value = analogRead(input_pins[i]);
    else  {
      raw = analogRead(input_pins[i]);
      raw = ~raw & 0x3FF;
      read_value = raw;
    }

    if ((i == RUD) && (read_value <= (calibrate_value[MIN_RUD] + 50))) {
      crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, PWR_25MW);
      CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    } else if ((i == RUD) && (read_value >= (calibrate_value[MAX_RUD] - 50))) {

      crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, PWR_50MW);
      CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    }

    if ( (i == AIL) && (read_value <= (calibrate_value[MIN_AIL] + 50))) {
      crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, RATE_25HZ);
      CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    } else if ((i == AIL) && (read_value >= (calibrate_value[MAX_AIL] - 50))) {
      crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, RATE_50HZ);
      CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    }

    if ((i == ELE) && (read_value <= (calibrate_value[MIN_ELE] + 50))) {
      crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, RATE_100HZ);
      CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    } else if ((i == ELE) && (read_value >= (calibrate_value[MAX_ELE] - 50))) {
      crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, RATE_200HZ);
      CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    }
  }
}

void print_info() {

  Serial.print("Sticks AETR - ");
  Serial.print("\t");
  for (int i = 0; i < CH_MAX; i++) {
    Serial.print(rcChannels[i]);
    Serial.print("\t");
  }

  Serial.print("Trim values -" );
  Serial.print("\t");
  for (int i = 0; i < sizeof(trim_value); i++) {
    if (i != THR) {
      Serial.print(trim_value[i]);
      Serial.print("\t");
    }
  }
  Serial.println();
}

// crc implementation from CRSF protocol document rev7
static uint8_t crsf_crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = crsf_crc8tab[crc ^ *ptr++];
  }
  return crc;
}


void crsfPrepareDataPacket(uint8_t packet[], int16_t channels[]) {


  // packet[0] = UART_SYNC; //Header
  packet[0] = ELRS_ADDRESS; // Header
  packet[1] = 24;           // length of type (24) + payload + crc
  packet[2] = TYPE_CHANNELS;
  packet[3] = (uint8_t)(channels[0] & 0x07FF);
  packet[4] = (uint8_t)((channels[0] & 0x07FF) >> 8 | (channels[1] & 0x07FF) << 3);
  packet[5] = (uint8_t)((channels[1] & 0x07FF) >> 5 | (channels[2] & 0x07FF) << 6);
  packet[6] = (uint8_t)((channels[2] & 0x07FF) >> 2);
  packet[7] = (uint8_t)((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
  packet[8] = (uint8_t)((channels[3] & 0x07FF) >> 7 | (channels[4] & 0x07FF) << 4);
  packet[9] = (uint8_t)((channels[4] & 0x07FF) >> 4 | (channels[5] & 0x07FF) << 7);
  packet[10] = (uint8_t)((channels[5] & 0x07FF) >> 1);
  packet[11] = (uint8_t)((channels[5] & 0x07FF) >> 9 | (channels[6] & 0x07FF) << 2);
  packet[12] = (uint8_t)((channels[6] & 0x07FF) >> 6 | (channels[7] & 0x07FF) << 5);
  packet[13] = (uint8_t)((channels[7] & 0x07FF) >> 3);
  packet[14] = (uint8_t)((channels[8] & 0x07FF));
  packet[15] = (uint8_t)((channels[8] & 0x07FF) >> 8 | (channels[9] & 0x07FF) << 3);
  packet[16] = (uint8_t)((channels[9] & 0x07FF) >> 5 | (channels[10] & 0x07FF) << 6);
  packet[17] = (uint8_t)((channels[10] & 0x07FF) >> 2);
  packet[18] = (uint8_t)((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
  packet[19] = (uint8_t)((channels[11] & 0x07FF) >> 7 | (channels[12] & 0x07FF) << 4);
  packet[20] = (uint8_t)((channels[12] & 0x07FF) >> 4 | (channels[13] & 0x07FF) << 7);
  packet[21] = (uint8_t)((channels[13] & 0x07FF) >> 1);
  packet[22] = (uint8_t)((channels[13] & 0x07FF) >> 9 | (channels[14] & 0x07FF) << 2);
  packet[23] = (uint8_t)((channels[14] & 0x07FF) >> 6 | (channels[15] & 0x07FF) << 5);
  packet[24] = (uint8_t)((channels[15] & 0x07FF) >> 3);

  packet[25] = crsf_crc8(&packet[2], packet[1] - 1); // CRC
}

// prepare elrs setup packet (power, packet rate...)
void crsfPrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value) {
  packetCmd[0] = ELRS_ADDRESS;
  packetCmd[1] = 6; // length of Command (4) + payload + crc
  packetCmd[2] = TYPE_SETTINGS_WRITE;
  packetCmd[3] = ELRS_ADDRESS;
  packetCmd[4] = ADDR_RADIO;
  packetCmd[5] = command;
  packetCmd[6] = value;
  packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1] - 1); // CRC
}

void CrsfWritePacket(uint8_t packet[], uint8_t packetLength) {
  Serial.write(packet, packetLength);
}
