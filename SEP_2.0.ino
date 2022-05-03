#include <Wire.h>         // Библиотека для работы с шиной I2C

#define I2C_SEP 0x1       // I2C-адресс СЭП
#define I2C_BUEMU 0x2     // I2C-адресс БУЭМУ
#define I2C_BUSOS 0x3     // I2C-адресс БУСОС
#define I2C_BC 0x4        // I2C-адресс БС

#define pinBtn_1 2        // Пин пусковой кнопки 1
#define pinBtn_2 3        // Пин пусковой кнопки 2

#define pinEn_BC 5        // Пин вкл/выкл БС
#define pinEn_BUSOS 6     // Пин вкл/выкл БУСОС
#define pinEn_BUEMU 7     // Пин вкл/выкл БУЭМУ
#define pinEn_ESC 4       // Пин вкл/выкл маховики (плата ESC)

#define pinAnlg_VCC A0    // VCC 2S 18650
#define pinAnlg_Bat A1    // 1/2 VCC - напряжение с 1S аккума
#define pinAnlg_VDD A2    // VDD - напряжение с солнечных панелей? (VDD~=VCC)

#define AREF 5.04         // Точное опорное напряжение (напряжение питания VCC) !!!!!!!!!! Сменить опорное напряжение на внутренний источник 1.1VOLT НО пересчитать номиналы резисторов в делителях A0 A1 A2
//#define AREFmode DEFAULT  // DEFAULT, AREF = VCC | INTERNAL, AREF = 1.1V | EXTERNAL, AREF = VCCaref
#define batLowVoltage 640 // (умноженое на 100, оригинал: 6.4)Напряжение батареи, при котором происходит переход в режим энергосбережения (отключение всех систем)

#define SEPARATOR '|'                 // Разделитель между данными при сохранении на SD карту и передачи по радио

// раздефайнить или задефайнить для использования
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif

struct telemSEP                   //SEP   | 28 bit = 3.5 byte | 10 symbols | 2 params
{
  uint32_t  localTime:      18;   // 2^18 = 262144 - макс. число, время полета 6 часов=360 минут=21.600 секунд =21.600,0 с - +1 знак после запятой 216.000 < 262.144, 0...262143
  uint16_t  voltageBat:     10;   // 2^10 = 1024 - макс. число, напряжение АКБ 0...+8,4 или 84 +1 знак после запятой = 840, значения строго больше 0, 0...1023

  void getChar(char* telemChar)
  {
    ultoa(localTime, telemChar, DEC);
    *strrchr(telemChar, '\0') = SEPARATOR;
    itoa(voltageBat, strrchr(telemChar, SEPARATOR) + 1, DEC);
  }
};

telemSEP trans;
bool isON = false;
volatile uint32_t lastTimeRqst = millis();

void setup()
{
  pinSetup();
#ifdef DEBUG_ENABLE
  Serial.begin(115200);
#endif

#ifdef AREFmode
  analogReference(AREFmode);
#endif

  Wire.begin(I2C_SEP);
  Wire.onRequest(I2C_dataRequest);
}

void loop() {
  uint32_t isTime = millis();
  uint16_t voltage;
  for (;;)
  {
    voltage = int((float)analogReadFast(A0) * AREF * ((100000 + 100000) / 100000) / 1024 * 100);
    switch (isOK(&voltage))
    {
      case 0:
        if (!isON) {
          systemON();
          lastTimeRqst = millis();
        }

        if (millis() - isTime >= 250)
        {
          DEBUG();
          DEBUG(F("VOLTAGE:"));
          DEBUG(voltage);
          telemSEP SEP;
          SEP.voltageBat = constrain(voltage, 0, 1023);
          SEP.localTime = constrain(millis() / 100, 0, 262143);
          trans = SEP;
          DEBUG(F("DATA UPDATED"));
          isTime = millis();
        }

        if (millis() - lastTimeRqst >= 5000)
        {
          DEBUG(F("WAIT: Restart..."));
          systemRestart();
          DEBUG(F("SUCCESS: Restarted!"));
          lastTimeRqst = millis();
        }
        break;

      case 1:
        DEBUG(F("LOW BATTERY and BUTTONS are PRESSED"));
        if (isON) systemOFF();
        break;

      case 2:
        DEBUG(F("BUTTONS are PRESSED"));
        if (isON) systemOFF();
        break;

      case 3:
        DEBUG(F("LOW BATTERY"));
        if (isON) systemOFF();
        break;
    }
  }
}

void I2C_dataRequest()
{
  DEBUG();
  DEBUG(F("WAIT: Request data... Start send"));
  I2C_write(trans);
#ifdef DEBUG_ENABLE
  char aa[15] = {};
  trans.getChar(aa);
  DEBUG(aa);
#endif
  DEBUG(F("SUCCESS: Requested data sended!"));
  lastTimeRqst = millis();
}

uint8_t isOK(uint16_t *voltage)
{
  if (isReleasedBtn() and (*voltage > batLowVoltage))
  {
    return 0;
  } else if (!isReleasedBtn() and (*voltage <= batLowVoltage))
  {
    return 1;
  } else if (!isReleasedBtn())
  {
    return 2;
  } else if (*voltage <= batLowVoltage)
  {
    return 3;
  }
}

void pinSetup()
{
  pinModeFast(pinEn_BC, OUTPUT);
  pinModeFast(pinEn_BUSOS, OUTPUT);
  pinModeFast(pinEn_BUEMU, OUTPUT);
  pinModeFast(pinEn_ESC, OUTPUT);
  pinModeFast(pinBtn_1, INPUT_PULLUP);
  pinModeFast(pinBtn_2, INPUT_PULLUP);
}

void systemON()
{
  uint32_t tt = millis();
  uint8_t sw = 1;
  DEBUG(F("WAIT: SystemON..."));
  for (;;)
  {
    switch (sw)
    {
      case 1:
        digitalWrite(pinEn_BUEMU, HIGH);
        sw = 2;
        tt = millis();
        break;

      case 2:
        if (millis() - tt >= 200)
        {
          digitalWrite(pinEn_BUSOS, HIGH);
          digitalWrite(pinEn_ESC, HIGH);
          sw = 3;
          tt = millis();
        }
        break;

      case 3:
        if (millis() - tt >= 200)
        {
          digitalWrite(pinEn_BC, HIGH);
          isON = true;
          DEBUG(F("SUCCESS: SystemON!"));
          return;
        }
        break;
    }
  }
}

void systemOFF()
{
  uint32_t tt = millis();
  uint8_t sw = 1;
  DEBUG(F("WAIT: SystemOFF..."));
  for (;;)
  {
    switch (sw)
    {
      case 1:
        digitalWrite(pinEn_BC, LOW);
        sw = 2;
        tt = millis();
        break;

      case 2:
        if (millis() - tt >= 200)
        {
          digitalWrite(pinEn_BUSOS, LOW);
          digitalWrite(pinEn_ESC, LOW);
          sw = 3;
          tt = millis();
        }
        break;

      case 3:
        if (millis() - tt >= 200)
        {
          digitalWrite(pinEn_BUEMU, LOW);
          isON = false;
          DEBUG(F("SUCCESS: SystemOFF!"));
          return;
        }
        break;
    }
  }
}

void systemRestart()
{
  uint32_t tt = millis();
  uint8_t sw = 1;
  for (;;)
  {
    switch (sw)
    {
      case 1:
        systemOFF();
        sw = 2;
        tt = millis();
        break;

      case 2:
        if (millis() - tt >= 500) sw = 3;
        break;

      case 3:
        systemON();
        return;
        break;
    }
  }
}

bool isReleasedBtn()
{
  if ( digitalReadFast(pinBtn_1) and digitalReadFast(pinBtn_2) ) {
    return true;
  }
  else {
    return false;
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

bool digitalReadFast(uint8_t pin) {
  if (pin < 8) {
    return bitRead(PIND, pin);
  } else if (pin < 14) {
    return bitRead(PINB, pin - 8);
  } else if (pin < 20) {
    return bitRead(PINC, pin - 14);    // Return pin state
  }
}

void pinModeFast(uint8_t pin, uint8_t mode) {
  switch (mode) {
    case INPUT:
      if (pin < 8) {
        bitClear(DDRD, pin);
        bitClear(PORTD, pin);
      } else if (pin < 14) {
        bitClear(DDRB, (pin - 8));
        bitClear(PORTB, (pin - 8));
      } else if (pin < 20) {
        bitClear(DDRC, (pin - 14));   // Mode: INPUT
        bitClear(PORTC, (pin - 8));  // State: LOW
      }
      return;
    case OUTPUT:
      if (pin < 8) {
        bitSet(DDRD, pin);
        bitClear(PORTD, pin);
      } else if (pin < 14) {
        bitSet(DDRB, (pin - 8));
        bitClear(PORTB, (pin - 8));
      } else if (pin < 20) {
        bitSet(DDRC, (pin - 14));  // Mode: OUTPUT
        bitClear(PORTC, (pin - 8));  // State: LOW
      }
      return;
    case INPUT_PULLUP:
      if (pin < 8) {
        bitClear(DDRD, pin);
        bitSet(PORTD, pin);
      } else if (pin < 14) {
        bitClear(DDRB, (pin - 8));
        bitSet(PORTB, (pin - 8));
      } else if (pin < 20) {
        bitClear(DDRC, (pin - 14));  // Mode: OUTPUT
        bitSet(PORTC, (pin - 14));  // State: HIGH
      }
      return;
  }
}

uint16_t analogReadFast(uint8_t pin) {
  pin = ((pin < 8) ? pin : pin - 14);    // analogRead(2) = analogRead(A2)
  ADMUX = (DEFAULT << 6) | pin;   // Set analog MUX & reference
  bitSet(ADCSRA, ADSC);            // Start
  while (ADCSRA & (1 << ADSC));        // Wait
  return ADC;                // Return result
}
