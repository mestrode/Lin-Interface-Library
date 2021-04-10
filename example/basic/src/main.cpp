#include <Arduino.h>
#include <Lin-Interface.hpp>

// using UART 1 for LinBus
Lin_Interface LinBus(1);

// data to be filled by bus request
float Cap_Max = 0.0;
float Cap_Available = 0.0;
uint8_t Cap_Configured = 0;
uint8_t CalibByte = 0x00;
bool CalibrationDone = false;

void setup()
{
  Serial.begin(115200);

  // configure baud rate
  Serial.print("configure Lin-Bus to 19200 Baud\n");
  LinBus.baud = 19200;
}

bool readLinData()
{
  bool chkSumValid = LinBus.readFrame(0x2C);
  if (chkSumValid)
  {
    // Data now avaliabele in LinBus.LinMessage

    // decode some bytes (incl. rescaling)
    Cap_Max = (float((LinBus.LinMessage[1] << 8) + LinBus.LinMessage[0])) / 10;
    Cap_Available = (float((LinBus.LinMessage[3] << 8) + LinBus.LinMessage[2])) / 10;

    // receive a single byte
    Cap_Configured = LinBus.LinMessage[4];

    // decode flags within a byte
    CalibByte = LinBus.LinMessage[5];
    CalibrationDone = bitRead(LinBus.LinMessage[5], 0);
  }
  return chkSumValid;
}

void loop()
{
  if (readLinData())
  {
    Serial.print("Data reveived:\n");
    Serial.printf("  Cap_Max = %f\n", Cap_Max);
    Serial.printf("  Cap_Available = %f\n", Cap_Available);
    Serial.printf("  Cap_Configured = %d\n", Cap_Configured);
    Serial.printf("  CalibByte = %02Xh\n", CalibByte);
    Serial.printf("  CalibrationDone = %d\n", CalibrationDone);
  }

  delay(5000);
}