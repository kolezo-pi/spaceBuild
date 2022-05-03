#include <Servo.h>      // Библиотека для управления бесколлекторными моторами - маховики для ориентации
#include <Wire.h>       // Библиотека для связи по шине I2C
#include <TroykaIMU.h>  // Библиотека для работы с IMU-датчиком (Акселлерометр, Гироскоп, Магнитометр (компас), барометр(+температура))
#include <MCP3008.h>    // Библиотека для работы с АЦП, который получается значения с датчиков освещенности

#define I2C_SEP   0x1     // I2C-адресс СЭП
#define I2C_BUEMU 0x2     // I2C-адресс БУЭМУ
#define I2C_BUSOS 0x3     // I2C-адресс БУСОС
#define I2C_BC    0x4     // I2C-адресс БС

#define pinMotor_1 11     // Пин канала управления мотором 1
#define pinMotor_2 12     // Пин канала управления мотором 2
#define pinMotor_3 10     // Пин канала управления мотором 3
#define pinMotor_4 13     // Пин канала управления мотором 4
#define pinESC     4      // Пин вкл/выкл платы ESC - контроллер бесколлекторных моторов

#define pinCLK  5         // Пин CLK АЦП датчиков освещенности
#define pinDOUT 6         // Пин DOUT АЦП датчиков освещенности
#define pinDIN  7         // Пин DIN АЦП датчиков освещенности
#define pinCS   8         // Пин CS АЦП датчиков освещенности

#define SEPARATOR '|'                 // Разделитель между данными при сохранении на SD карту и передачи по радио

// раздефайнить или задефайнить для использования
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif

const int solResistCalibration[8][2] = {
  {0, 1020},
  {0, 1021},
  {0, 1021},
  {0, 1020},
  {0, 1022},
  {0, 1022},
  {0, 1022},
  {0, 1022}
};

const float compassCalibrationBias[3] = {
  -266.013,
  -2925.756,
  5463.331
};

const float compassCalibrationMatrix[3][3] = {
  {2.347, 0.075, 0.043},
  {0.086, 2.276, 0.052},
  {-0.054, 0.088, 1.865}
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
    itoa(temperatureIMU, telemChar, DEC);
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
  }
};

telemBUSOS trans;
bool isMotor = false;

Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;

MCP3008 adc(pinCLK, pinDIN, pinDOUT, pinCS);
Gyroscope gyroscope;
Accelerometer accelerometer;
Compass compass;
Barometer barometer;

void setup()
{
#ifdef DEBUG_ENABLE
  Serial.begin(115200);
#endif

  accelerometer.begin();
  accelerometer.setRange(AccelerometerRange::RANGE_8G);

  compass.begin();
  compass.setRange(CompassRange::RANGE_16GAUSS);
  compass.setCalibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);

  gyroscope.begin();
  gyroscope.setRange(GyroscopeRange::RANGE_2000DPS);

  barometer.begin();

  Wire.begin(I2C_BUSOS);
  Wire.onRequest(I2C_dataRequest);
}

void loop()
{
  uint32_t tt = millis();
  for (;;)
  {
    //if (!isMotor) motor_init();
    if (millis() - tt >= 250)
    {
      telemBUSOS local;
      local.temperatureIMU  = constrain(barometer.readTemperatureC() * 100, -8192, 8191);
      local.pressureIMU     = constrain(barometer.readPressurePascals() * 10ul, 0, 2097151);
      local.IMUAltitude     = constrain(barometer.readAltitude() * 10ul, 0, 524287);

      local.solar_0 = constrain(map(adc.readADC(0), solResistCalibration[0][0], solResistCalibration[0][1], 0, 255), 0, 255);
      local.solar_1 = constrain(map(adc.readADC(1), solResistCalibration[1][0], solResistCalibration[1][1], 0, 255), 0, 255);
      local.solar_2 = constrain(map(adc.readADC(2), solResistCalibration[2][0], solResistCalibration[2][1], 0, 255), 0, 255);
      local.solar_3 = constrain(map(adc.readADC(3), solResistCalibration[3][0], solResistCalibration[3][1], 0, 255), 0, 255);
      local.solar_4 = constrain(map(adc.readADC(4), solResistCalibration[4][0], solResistCalibration[4][1], 0, 255), 0, 255);
      local.solar_5 = constrain(map(adc.readADC(5), solResistCalibration[5][0], solResistCalibration[5][1], 0, 255), 0, 255);
      local.solar_6 = constrain(map(adc.readADC(6), solResistCalibration[6][0], solResistCalibration[6][1], 0, 255), 0, 255);
      local.solar_7 = constrain(map(adc.readADC(7), solResistCalibration[7][0], solResistCalibration[7][1], 0, 255), 0, 255);

      float mx, my, mz; compass.readCalibrateMagneticGaussXYZ(mx, my, mz);
      float ax, ay, az; accelerometer.readAccelerationGXYZ(ax, ay, az);
      float gx, gy, gz; gyroscope.readRotationRadXYZ(gx, gy, gz);

      local.accel_X = constrain(ax * 1000, -8192, 8191);
      local.accel_Y = constrain(ay * 1000, -8192, 8191);
      local.accel_Z = constrain(az * 1000, -8192, 8191);

      local.compass_X = constrain((mx * 1000), -16384, 16383);
      local.compass_Y = constrain((my * 1000), -16384, 16383);
      local.compass_Z = constrain((mz * 1000), -16384, 16383);

      local.gyro_X = constrain((gx * 1000), -65536, 65535);
      local.gyro_Y = constrain((gy * 1000), -65536, 65535);
      local.gyro_Z = constrain((gz * 1000), -65536, 65535);

      trans = local;
      DEBUG(F("DATA UPDATED"));
      tt = millis();
    }
  }
}

void I2C_dataRequest()
{
  DEBUG();
  DEBUG(F("WAIT: Request data... Start send"));
  I2C_write(trans);
#ifdef DEBUG_ENABLE
  char aa[120] = {};
  trans.getChar(aa);
  DEBUG(aa);
#endif
  DEBUG(F("SUCCESS: Requested data sended!"));
}

void motor_init()
{
  static uint8_t sw = 1;
  static uint32_t tt = millis();
  switch (sw)
  {
    case 1:
      DEBUG();
      DEBUG(F("WAIT: MOTOR INIT..."));
      DEBUG();
      pinMode(pinESC, OUTPUT);
      digitalWrite(pinESC, LOW);
      sw = 2;
      tt = millis();
      break;

    case 2:
      if (millis() - tt >= 250) sw = 3;
      break;

    case 3:
      motor_1.attach(pinMotor_1);
      motor_2.attach(pinMotor_2);
      motor_3.attach(pinMotor_3);
      motor_4.attach(pinMotor_4);

      motor_1.writeMicroseconds(2000);
      motor_2.writeMicroseconds(2000);
      motor_3.writeMicroseconds(2000);
      motor_4.writeMicroseconds(2000);

      digitalWrite(pinESC, HIGH);
      sw = 4;
      tt = millis();
    case 4:
      if (millis() - tt >= 6000) sw = 5;
      break;

    case 5:
      motor_1.writeMicroseconds(1000);
      motor_2.writeMicroseconds(1000);
      motor_3.writeMicroseconds(1000);
      motor_4.writeMicroseconds(1000);
      sw = 6;
      break;

    case 6:
      if (millis() - tt >= 7000)
      {
        isMotor = true;
        DEBUG();
        DEBUG(F("SUCCESS: MOTOR INITED!"));
        DEBUG();
      }
      break;
  }
}

template <typename T> void I2C_write(const T& value)
{
  const byte* p = (const byte*) &value;
  for (uint8_t i = 0; i < sizeof(value); i++)
    Wire.write(*p++);
}

template <typename T> void I2C_read(const T& value)
{
  byte* p = (byte*) &value;
  for (uint8_t i = 0; i < sizeof(value); i++)
    *p++ = Wire.read();
}
