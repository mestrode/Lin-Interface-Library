#include <Arduino.h>
#include <LinFrameTransfer.hpp>
#include <optional>
#include <vector>

// using UART 2 for LinBus, UART 1 for debug messages
LinFrameTransfer LinBus(Serial2, Serial1);

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
  auto rawData = LinBus.readFrame(0x2C);
  if (!rawData)
  {
    return false;
  }

  // Data now avaliabele in data.value() or at address data.data()
  struct ResponseCap {
    uint8_t capMax_LSB;
    uint8_t capMax_MSB;
    uint8_t capAvaliable_LSB;
    uint8_t capAvaliable_MSB;
    uint8_t capConfigured;
    uint8_t capFlags;
  };
  ResponseCap* data = reinterpret_cast<ResponseCap*>(rawData.value().data());

  // decode some bytes (incl. rescaling)
  Cap_Max = ((data->capMax_MSB << 8) + data->capMax_LSB) / 10;
  Cap_Available = (float((data->capAvaliable_MSB << 8) + data->capAvaliable_LSB)) / 10;

  // receive a single byte
  Cap_Configured = data->capConfigured;

  // decode flags within a byte
  CalibByte = data->capFlags;
  CalibrationDone = bitRead(data->capFlags, 0);

  return true;
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