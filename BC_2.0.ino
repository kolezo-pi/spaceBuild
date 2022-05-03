#include <SPI.h>                // Подключаем библиотеку  для работы с шиной SPI
#include <nRF24L01.h>           // Подключаем файл настроек из библиотеки RF24
#include <RF24.h>               // Подключаем библиотеку  для работы с nRF24L01+
#include <Wire.h>               // Библиотека для работы с шиной I2C
#include <SoftwareSerial.h>     // Библиотека для создания программиного UART интерфейса (для GPS)
#include "PetitFS.h"

#define I2C_SEP 0x1             // I2C-адресс СЭП
#define I2C_BUEMU 0x2           // I2C-адресс БУЭМУ
#define I2C_BUSOS 0x3           // I2C-адресс БУСОС
#define I2C_BC 0x4              // I2C-адресс БС

#define pinSD 8                 // Номер пина, к которому подключен пин CS модуля SD-карты
#define pinRX 2                 // Номер пина, к которому подключен модуль GPS
#define pinTX 3                 // Номер пина, к которому подключен модуль GPS
#define pinCE 9                 // Номер пина, к которому подключен пин CE радио-модуля NRF24L01
#define pinCSN 10               // Номер пина, к которому подключен пин CSN радио-модуля NRF24L01

#define radioChannel 0x69             // Номер канала (00-7D | 0-125)
#define radioDataRate RF24_250KBPS    // Возможны: RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define pipeID 0x1234567890LL         // ID трубы, по которой вещаем
#define radioPALevel RF24_PA_LOW      // RF24_PA_MIN = -18dBm, RF24_PA_LOW = -12dBm, RF24_PA_HIGH = -6dBm, RF24_PA_MAX = 0dBm

#define SEPARATOR '|'                 // Разделитель между данными при сохранении на SD карту и передачи по радио

// раздефайнить или задефайнить для использования
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif

struct telemetryRadio             //RADIO | 242 bit = 30.25 byte
{
  //28 bit
  uint32_t  localTime:      18;   // 2^18 = 262144 - макс. число, время полета 6 часов=360 минут=21.600 секунд =21.600,0 с - +1 знак после запятой 216.000 < 262.144, 0...262143
  uint16_t  voltageBat:     10;   // 2^10 = 1024 - макс. число, напряжение АКБ 0...+8,4 или 84 +1 знак после запятой = 840, значения строго больше 0, 0...1023
  //111 bit
  uint8_t   satTimeHour:    5;    // 2^5 = 32 - макс. число, в сутках 0...24 часа, 24 < 32, значения строго больше 0, 0...31
  uint8_t   satTimeMinute:  6;    // 2^6 = 64 - макс. число, в часе 0...60 минут, 60 < 64, значения строго больше 0, 0...63
  uint8_t   satTimeSecond:  6;    // 2^6 = 64 - макс. число, в минуте 0...60 секунд, 60 < 64, значения строго больше 0...63
  int32_t   satLongitude:   29;   // 2^29 = 536.870.912 - макс. число, долгота -180...+180, для точности берем 6 знаков после запятой: +-180,000000 или +-180.000.000, значения могут быть меньше нуля => -268435456...0...+268435455
  int32_t   satLatitude:    28;   // 2^28 = 268.435.456 - макс. число, широта -90...+90, для точности берем 6 знаков после запятой: +-90,000000 или +-90.000.000, значения могут быть меньше нуля => -134217728...0...+134217727
  uint32_t  satAltitude:    19;   // 2^19 = 524.288 - макс. число, высота над уровнем моря 0...30.000, для точности берем 1 знак после запятой: 30.000,0 или 300.000, значения строго больше 0, 0...524287
  uint16_t  satSpeed:       12;   // 2^12 = 4096 - макс. число, скорость по GPS 0...+210, для точности берем 1 знак после запятой: 210,0 или 2100, значения строго больше 0, 0...4095
  uint8_t   satCount:       6;    // 2^6 = 64 - макс. число, количество спутников 0...64, значения строго больше 0, 0...63
  //49 bit
  int16_t   temperatureBat: 14;   // 2^14 = 16.384 - макс. число, температура АКБ -70...+70, для точности берем 2 знака после запятой: +-70,00 или +-7000, значения могут быть меньше нуля => -8192...0...+8191
  int16_t   temperatureOut: 14;   // 2^14 = 16.384 - макс. число, температура за бортом -70...+70, для точности берем 2 знака после запятой: +-70,00 или +-7000, значения могут быть меньше нуля => -8192...0...+8191
  uint32_t  pressureExt:    21;   // 2^21 = 2.097.152 - макс. число, давление по датчик XGZP, 0...100.000 Па, для точности берем 1 знак после запятой: 100.000,0 или 1.000.000, значени строго больше 0, 0...2097151
  //54 bit
  int16_t   temperatureIMU: 14;   // 2^13 = 8.192 - макс. число, температура бортовых систем (с IMU-сенсора) -40...+40, для точности берем 2 знака после запятой: +-40,00 или +-4000, значения могут быть меньше нуля => -4096...0...+4095
  uint32_t  pressureIMU:    21;   // 2^21 = 2.097.152 - макс. число, давление по IMU-сенсору, 26.000...100.000 Па, для точности берем 1 знак после запятой: 100.000,0 или 1.000.000, значени строго больше 0, 0...2097151
  uint32_t  IMUAltitude:    19;   // 2^19 = 524.288 - макс. число, высота над уровнем моря 0...30.000, для точности берем 1 знак после запятой: 30.000,0 или 300.000, значения строго больше 0, 0...524287
};

struct telemSEP                   //SEP   | 28 bit = 3.5 byte | 10 symbols | 2 params
{
  uint32_t  localTime:      18;   // 2^18 = 262144 - макс. число, время полета 6 часов=360 минут=21.600 секунд =21.600,0 с - +1 знак после запятой 216.000 < 262.144, 0...262143
  uint16_t  voltageBat:     10;   // 2^10 = 1024 - макс. число, напряжение АКБ 0...+8,4 или 84 +1 знак после запятой = 840, значения строго больше 0, 0...1023

  void getChar(char* telemChar)
  {
    ultoa(localTime, telemChar, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(voltageBat, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
  }
};

struct telemBUEMU                 //BUEMU | 49 bit = 6.125 byte | 17 symbols | 3 params
{
  int16_t   temperatureBat: 14;   // 2^14 = 16.384 - макс. число, температура АКБ -70...+70, для точности берем 2 знака после запятой: +-70,00 или +-7000, значения могут быть меньше нуля => -8192...0...+8191
  int16_t   temperatureOut: 14;   // 2^14 = 16.384 - макс. число, температура за бортом -70...+70, для точности берем 2 знака после запятой: +-70,00 или +-7000, значения могут быть меньше нуля => -8192...0...+8191
  uint32_t  pressureExt:    21;   // 2^21 = 2.097.152 - макс. число, давление по датчик XGZP, 0...100.000 Па, для точности берем 1 знак после запятой: 100.000,0 или 1.000.000, значени строго больше 0, 0...2097151

  void getChar(char* telemChar)
  {
    itoa(temperatureBat, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(temperatureOut, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    ultoa(pressureExt, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
  }
};

struct telemBUSOS                 //BUSOS | 256 bit = 32 байта | 93 symbols | 20 params
{
  //54 bit | 18 symbols
  int16_t   temperatureIMU: 14;   // 2^14 = 16.384 - макс. число, температура бортовых систем (с IMU-сенсора) -70...+70, для точности берем 2 знака после запятой: +-70,00 или +-7000, значения могут быть меньше нуля => -8192...0...+8191
  uint32_t  pressureIMU:    21;   // 2^21 = 2.097.152 - макс. число, давление по IMU-сенсору, 26.000...100.000 Па, для точности берем 1 знак после запятой: 100.000,0 или 1.000.000, значени строго больше 0, 0...2097151
  uint32_t  IMUAltitude:    19;   // 2^19 = 524.288 - макс. число, высота над уровнем моря 0...30.000, для точности берем 1 знак после запятой: 30.000,0 или 300.000, значения строго больше 0, 0...524287
  //64 bit | 24 symbols
  uint8_t  solar_0:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  uint8_t  solar_1:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  uint8_t  solar_2:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  uint8_t  solar_3:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  uint8_t  solar_4:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  uint8_t  solar_5:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  uint8_t  solar_6:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  uint8_t  solar_7:         8;    // 2^8 = 256, показания датчиков освещенности, 0...255
  //51 bit | 18 symbols
  int32_t    gyro_X:        17;   // 2^18=131027, +-35 c датчика + 3 знака после запятой,  -65536...+65535
  int32_t    gyro_Y:        17;   // 2^18=131027, +-35 c датчика + 3 знака после запятой,  -65536...+65535
  int32_t    gyro_Z:        17;   // 2^18=131027, +-35 c датчика + 3 знака после запятой,  -65536...+65535
  //42 bit | 15 symbols
  int16_t    accel_X:       14;   //2^14 = 16.384, +-8 с датчика + 3 знака после запятой, -8192...0...+8191
  int16_t    accel_Y:       14;   //2^14 = 16.384, +-8 с датчика + 3 знака после запятой, -8192...0...+8191
  int16_t    accel_Z:       14;   //2^14 = 16.384, +-8 с датчика + 3 знака после запятой, -8192...0...+8191
  //45 bit | 18 symbols
  int16_t    compass_X:     15;   //2^15 = 32.768, +-16 с датчика + 3 знака после запятой, -16384...0...+16383
  int16_t    compass_Y:     15;   //2^15 = 32.768, +-16 с датчика + 3 знака после запятой, -16384...0...+16383
  int16_t    compass_Z:     15;   //2^15 = 32.768, +-16 с датчика + 3 знака после запятой, -16384...0...+16383

  void getChar(char* telemChar)
  {
    itoa(temperatureIMU, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    ultoa(pressureIMU, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    ultoa(IMUAltitude, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_0, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_1, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_2, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_3, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_4, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_5, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_6, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(solar_7, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    ltoa(gyro_X, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    ltoa(gyro_Y, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    ltoa(gyro_Z, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(accel_X, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(accel_Y, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(accel_Z, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(compass_X, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(compass_Y, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(compass_Z, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strrchr(telemChar, '\0') = '\n';
    *(strrchr(telemChar, '\n') + 1) = '\0';
  }
};

struct telemBC                    //BC    | 111 bit = 13.875 byte | 38 symbols | 8 params
{
  //111 bit
  uint8_t   satTimeHour:    5;    // 2^5 = 32 - макс. число, в сутках 0...24 часа, 24 < 32, значения строго больше 0, 0...31
  uint8_t   satTimeMinute:  6;    // 2^6 = 64 - макс. число, в часе 0...60 минут, 60 < 64, значения строго больше 0, 0...63
  uint8_t   satTimeSecond:  6;    // 2^6 = 64 - макс. число, в минуте 0...60 секунд, 60 < 64, значения строго больше 0...63
  int32_t   satLongitude:   29;   // 2^29 = 536.870.912 - макс. число, долгота -180...+180, для точности берем 6 знаков после запятой: +-180,000000 или +-180.000.000, значения могут быть меньше нуля => -268435456...0...+268435455
  int32_t   satLatitude:    28;   // 2^28 = 268.435.456 - макс. число, широта -90...+90, для точности берем 6 знаков после запятой: +-90,000000 или +-90.000.000, значения могут быть меньше нуля => -134217728...0...+134217727
  uint32_t  satAltitude:    19;   // 2^19 = 524.288 - макс. число, высота над уровнем моря 0...30.000, для точности берем 1 знак после запятой: 30.000,0 или 300.000, значения строго больше 0, 0...524287
  uint16_t  satSpeed:       12;   // 2^12 = 4096 - макс. число, скорость по GPS 0...+210, для точности берем 1 знак после запятой: 210,0 или 2100, значения строго больше 0, 0...4095
  uint8_t   satCount:       6;    // 2^6 = 64 - макс. число, количество спутников 0...64, значения строго больше 0, 0...63

  void getChar(char* telemChar)
  {
    itoa(satTimeHour, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
    itoa(satTimeMinute, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
    itoa(satTimeSecond, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
    ltoa(satLongitude, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
    ltoa(satLatitude, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
    ultoa(satAltitude, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
    itoa(satSpeed, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
    itoa(satCount, strrchr(telemChar, SEPARATOR) + 1, DEC);
    *strchr(telemChar, '\0') = SEPARATOR;
  }
};

//############################### Объявление глобальных переменных ################################
RF24 radio(pinCE, pinCSN);
SoftwareSerial gpsSerial(pinRX, pinTX);
FATFS fs;
telemBC BC;

void setup() {
#ifdef DEBUG_ENABLE
  Serial.begin(115200);
#endif
  gpsSerial.begin(9600);
  radioSetup();

  while (pf_mount(&fs))
  {
    DEBUG(F("pf_mount!"));
  }
  Wire.begin(I2C_BC);

  BC.satTimeHour    = 0;
  BC.satTimeMinute  = 0;
  BC.satTimeSecond  = 0;
  BC.satLongitude   = 0;
  BC.satLatitude    = 0;
  BC.satAltitude    = 0;
  BC.satSpeed       = 0;
  BC.satCount       = 0;


  DEBUG(F("Setuped"));
}

void loop()
{
  uint32_t tt = millis();
  for (;;)
  {
    if (millis() - tt >= 2000)
    {
      tt = millis();
      DEBUG(F("Save telem"));
      saveTelem();

    }
  }
}

void radioSetup()
{
  radio.begin();
  radio.setChannel(radioChannel);
  for (uint8_t i = 0; i < 5; i++)
  {
    bool isOK = radio.setDataRate (radioDataRate);
    if (isOK) {
      DEBUG(F("SUCCESSFUL: DataRate radio set"));
      break;
    }
    if (i == 4)
    {
      DEBUG(F("ERROR: Set DataRate Radio"));
      while (1);
    }
  }
  radio.setPALevel (radioPALevel);
  radio.openWritingPipe (pipeID);
  radio.stopListening();
}

uint8_t updateTelem()
{
  char gpsRMC[100] = {};
  char gpsGGA[100] = {};
  DEBUG(F("char inited"));
  getNMEA(gpsRMC, gpsGGA);

  DEBUG(gpsRMC);
  DEBUG(gpsGGA);

  DEBUG(F("start save"));
  static uint32_t ofsetNMEA = getOfset("NMEA.TXT");
  if (!pf_open("NMEA.TXT"))
  {
    if (!pf_lseek(ofsetNMEA))
    {
      uint16_t nr;
      if (!pf_write(gpsRMC, sizeof(gpsRMC), &nr))
      {
        if (!pf_write(gpsGGA, sizeof(gpsGGA), &nr))
        {
          if (!pf_write(0, 0, &nr))
          {
            ofsetNMEA += 512;
            DEBUG(F("NMEA SAVED"));
            DEBUG(fs.fptr);
          } else DEBUG(F("pf_write_end_nmea"));
        } else DEBUG(F("pf_write_2_nmea"));
      } else DEBUG(F("pf_write_nmea"));
    } else DEBUG(F("pf_lseek_nmea"));
  } else DEBUG(F("pf_open_nmea"));
  DEBUG(F("char saved"));

  if (!pf_open("NMEA.TXT"))
  {
    char c[16] = {};
    ultoa(ofsetNMEA, c, DEC);
    uint16_t nr;
    if (!pf_write(c, 16, &nr))
    {
      if (!pf_write(0, 0, &nr))
      {
        DEBUG(F("NMEA ofset SAVED"));
        DEBUG(fs.fptr);
      } else DEBUG(F("pf_write_end_ofset"));
    } else DEBUG(F("pf_write_ofset"));
  } else DEBUG(F("pf_open_ofset"));

  uint8_t gpsHour = NULL;
  uint8_t gpsMinute  = NULL;
  uint8_t gpsSecond  = NULL;
  uint16_t gpsSpeed  = NULL;
  int32_t gpsLatitude  = NULL;
  int32_t gpsLongitude  = NULL;
  uint8_t gpsSat  = NULL;
  uint32_t gpsAltitude  = NULL;
  if (parsNMEA(gpsRMC, gpsGGA, &gpsHour, &gpsMinute, &gpsSecond, &gpsSpeed, &gpsLatitude, &gpsLongitude, &gpsSat, &gpsAltitude) == 1)
  {
    BC.satTimeHour   = constrain(gpsHour, 0, 31);
    BC.satTimeMinute = constrain(gpsMinute, 0, 63);
    BC.satTimeSecond = constrain(gpsSecond, 0, 63);
    BC.satLongitude  = constrain(gpsLongitude, -268435456, 268435455);
    BC.satLatitude   = constrain(gpsLatitude, -134217728, 134217727);
    BC.satAltitude   = constrain(gpsAltitude, 0, 524287);
    BC.satSpeed      = constrain(gpsSpeed, 0, 4095);
    BC.satCount      = constrain(gpsSat, 0, 63);
    return true;
  } else {

    DEBUG(F("No satilittes!"));

    return false;
  }
}

void saveTelem()
{
  char telemChar[230] = {};
  telemetryRadio radioTelem;

  {
    telemSEP SEP;
    DEBUG(F("Rqst SEP"));
    Wire.requestFrom(I2C_SEP, sizeof(SEP));
    I2C_read(SEP);
    SEP.getChar(telemChar);

    radioTelem.localTime       = SEP.localTime;
    radioTelem.voltageBat      = SEP.voltageBat;
  }

  {
    DEBUG(F("UPdate Telem"));
    updateTelem();
    BC.getChar(telemChar);

    radioTelem.satTimeHour     = BC.satTimeHour;
    radioTelem.satTimeMinute   = BC.satTimeMinute;
    radioTelem.satTimeSecond   = BC.satTimeSecond;
    radioTelem.satLongitude    = BC.satLongitude;
    radioTelem.satLatitude     = BC.satLatitude;
    radioTelem.satAltitude     = BC.satAltitude;
    radioTelem.satSpeed        = BC.satSpeed;
    radioTelem.satCount        = BC.satCount;
  }
/*
  {
    telemBUEMU BUEMU;
    DEBUG(F("Rqst BUEMU"));
    Wire.requestFrom(I2C_BUEMU, sizeof(BUEMU));
    I2C_read(BUEMU);
    BUEMU.getChar(telemChar);

    radioTelem.temperatureBat  = BUEMU.temperatureBat;
    radioTelem.temperatureOut  = BUEMU.temperatureOut;
    radioTelem.pressureExt     = BUEMU.pressureExt;
  }
*/
  {
    telemBUSOS BUSOS;
    DEBUG(F("Rqst BUSOS"));
    Wire.requestFrom(I2C_BUSOS, sizeof(BUSOS));
    I2C_read(BUSOS);
    BUSOS.getChar(telemChar);

    radioTelem.temperatureIMU  = BUSOS.temperatureIMU;
    radioTelem.pressureIMU     = BUSOS.pressureIMU;
    radioTelem.IMUAltitude     = BUSOS.IMUAltitude;
  }

  DEBUG(F("Create 316 byte mass"));

  DEBUG(telemChar);

  DEBUG(F("Vik"));
  static uint32_t ofsetTelem = getOfset("TELEM.TXT");
  if (!pf_open("TELEM.TXT"))
  {
    if (!pf_lseek(ofsetTelem))
    {
      uint16_t nr;
      if (!pf_write(telemChar, sizeof(telemChar), &nr))
      {
        if (!pf_write(0, 0, &nr))
        {
          ofsetTelem += 512;
          DEBUG(F("TELEM SAVED"));
          DEBUG(telemChar);
          DEBUG(fs.fptr);
        } else DEBUG(F("pf_write_end_tele"));
      } else DEBUG(F("pf_write_tele"));
    } else DEBUG(F("pf_lseek_tele"));
  } else DEBUG(F("pf_open_tele"));

  if (!pf_open("TELEM.TXT"))
  {
    char c[16] = {};
    ultoa(ofsetTelem, c, DEC);
    uint16_t nr;
    if (!pf_write(c, 16, &nr))
    {
      if (!pf_write(0, 0, &nr))
      {
        DEBUG(F("TELE ofset SAVED"));
        DEBUG(fs.fptr);
      } else DEBUG(F("pf_write_end_ofset"));
    } else DEBUG(F("pf_write_ofset"));
  } else DEBUG(F("pf_open_ofset"));


  DEBUG(F("Lets send radio: "));
  DEBUG(sizeof(radioTelem));
  radio.write(&radioTelem, sizeof(radioTelem));

  DEBUG(F("Radio sended"));

  DEBUG(telemChar);
}

//##############################################################################################

uint32_t getOfset(char* fileName)
{
  uint32_t ofset = 512;
  if (!pf_open(fileName))
  {
    uint16_t nr;
    char c[16] = {};
    if (!pf_read(&c, 16, &nr))
    {
      ofset = atol(c);
    } else DEBUG(F("ERROR READ_get_ofset"));
  } else DEBUG(F("pf_open_get_ofset"));
  return ofset;
  /*
    if (!pf_open(fileName))
    {
    char c = 'A';
    while (c != '\0')
    {
      if (!pf_lseek(ofset))
      {
        uint16_t nr;
        if (!pf_read(&c, 1, &nr))
        {
          ofset += 512;
        } else DEBUG(F("ERROR READ"));
      } else DEBUG(F("pf_lseek"));
    }
    } else DEBUG(F("pf_open"));

    return ofset - 512; */
}

template <typename T> void I2C_write(const T & value)
{
  const byte* p = (const byte*) &value;
  for (uint8_t i = 0; i < sizeof(value); i++)
    Wire.write(*p++);
}

template <typename T> void I2C_read(const T & value)
{
  byte* p = (byte*) &value;
  for (uint8_t i = 0; i < sizeof(value); i++)
    *p++ = Wire.read();
}

void parsingGNGGA(char* gpsBuffer, char* sat, char* altitude) {
  int i = 0;
  int j = 0;
  // пропуск типа сообщения
  if (gpsBuffer[i] == '$') {
    i++;
    while (gpsBuffer[i] != ',') {
      i++;
    }
  }

  // пропуск записи данных о времени
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != '.') {
      i++;
    }
    i++;
    while (gpsBuffer[i] != ',') {
      i++;
    }
  }

  // пропуск данных широты
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      i++;
    }
  }
  // пропуск буквы широты «N»
  i = i + 2;

  // пропуск данных долготы
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      i++;
    }
  }

  // пропуск буквы долготы «E»
  i = i + 2;
  // пропуск типа решения
  i = i + 2;

  // запись данных кол-ва спутников
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      sat[j++] = gpsBuffer[i++];
    }
    sat[j] = '\0';
  }

  // пропуск геометрического фактора
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      i++;
    }
  }
  // запись данных высоты
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      altitude[j++] = gpsBuffer[i++];
    }
    altitude[j] = '\0';
  }
}

void parsingGNRMC(char* gpsBuffer, char* time, char* latitude, char* longitude, char* speed) {
  uint8_t i = 0;
  uint8_t j = 0;

  if (gpsBuffer[i] == '$') {
    i++;
    while (gpsBuffer[i] != ',') {
      i++;
    }
  }
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != '.') {
      time[j++] = gpsBuffer[i++];
    }

    i++;
    time[j] = '\0';
    while (gpsBuffer[i] != ',') {
      i++;
    }
  }

  // состояние GPS
  if (gpsBuffer[i] == ',') {
    i++;
    i++;
  }

  // запись данных широты
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      latitude[j++] = gpsBuffer[i++];
    }
    i++;
    while (gpsBuffer[i] != ',') {
      latitude[j++] = gpsBuffer[i++];
    }
    latitude[j] = '\0';
  }

  // запись данных долготы
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      longitude[j++] = gpsBuffer[i++];
    }
    i++;
    while (gpsBuffer[i] != ',') {
      longitude[j++] = gpsBuffer[i++];
    }
    longitude[j] = '\0';
  }

  // запись данных скорости
  if (gpsBuffer[i] == ',') {
    i++;
    j = 0;
    while (gpsBuffer[i] != ',') {
      speed[j++] = gpsBuffer[i++];
    }
    speed[j] = '\0';
  }
}

void getNMEA(char* gpsRMC, char* gpsGGA)
{
  while (gpsRMC[0] != '$' or gpsGGA[0] != '$')
  {
    if (gpsSerial.available())
    {
      if (gpsSerial.read() == '$')
      {
        uint8_t i = 0;
        char self[4] = {};
        self[0] = '$';
        while (i < 4)
        {
          if (gpsSerial.available())
          {
            self[i] = gpsSerial.read();
            i++;
          }
        }

        if (self[0] == 'G' and self[1] == 'N' and self[2] == 'R' and self[3] == 'M')
        {
          gpsRMC[0] = '$';
          gpsRMC[1] = 'G';
          gpsRMC[2] = 'N';
          gpsRMC[3] = 'R';
          gpsRMC[4] = 'M';

          char c = NULL;
          i = 5;
          while (c != '\n')
          {
            if (gpsSerial.available())
            {
              c = gpsSerial.read();
              gpsRMC[i] = c;
              i++;
            }
          }
          gpsRMC[i] = '\0';
        } else if (self[0] == 'G' and self[1] == 'N' and self[2] == 'G' and self[3] == 'G')
        {
          gpsGGA[0] = '$';
          gpsGGA[1] = 'G';
          gpsGGA[2] = 'N';
          gpsGGA[3] = 'G';
          gpsGGA[4] = 'G';

          char c = NULL;
          i = 5;
          while (c != '\n')
          {
            if (gpsSerial.available())
            {
              c = gpsSerial.read();
              gpsGGA[i] = c;
              i++;
            }
          }
          gpsGGA[i] = '\0';
        }
      }
    }
  }
}

uint8_t parsNMEA(char* gpsRMC, char* gpsGGA, uint8_t* _hour, uint8_t* _minute, uint8_t* _second, uint16_t* _speed, int32_t* _latitudeBase10, int32_t* _longitudeBase10, uint8_t* _sat, uint32_t* _altitude)
{
  if (gpsRMC[17] == 'A')
  {
    char speed[8];
    char time[16];
    char latitude[16];
    char longitude[16];
    char parsingLatitude[16];
    char parsingLongitude[16];

    parsingGNRMC(gpsRMC, time, latitude, longitude, speed);

    *_hour = (time[0] - '0') * 10 + (time[1] - '0');
    *_minute = (time[2] - '0') * 10 + (time[3] - '0');
    *_second = (time[4] - '0') * 10 + (time[5] - '0');

    *_speed = atof(speed) * 10 * 1.852;

    uint8_t i = 2;
    uint8_t j = 0;
    while (latitude[i] != '\0') {
      parsingLatitude[j++] = latitude[i++];
    }
    parsingLatitude[j] = '\0';
    *_latitudeBase10 = int32_t(atof(parsingLatitude) * 50000 / 3);
    *_latitudeBase10 = *_latitudeBase10 + (latitude[0] - '0') * 10000000 + (latitude[1] - '0') * 1000000;
    if (latitude[strlen(latitude) - 1] == 'S') {
      *_latitudeBase10 = -1 * *_latitudeBase10;
    }

    i = 3;
    j = 0;
    while (longitude[i] != '\0') {
      parsingLongitude[j++] = longitude[i++];
    }
    parsingLongitude[j] = '\0';
    *_longitudeBase10 = int32_t(atof(parsingLongitude) * 50000 / 3);
    *_longitudeBase10 = *_longitudeBase10 + (longitude[0] - '0') * 100000000 + (longitude[1] - '0') * 10000000 + (longitude[2] - '0') * 1000000;
    if (longitude[strlen(longitude) - 1] == 'W') {
      *_longitudeBase10 = -1 * *_longitudeBase10;
    }

    char altitude[8];
    char sat[8];

    parsingGNGGA(gpsGGA, sat, altitude);

    *_sat = atoi(sat);
    *_altitude = uint32_t(atof(altitude) * 10);

    return true;
  }
  return false;
}
