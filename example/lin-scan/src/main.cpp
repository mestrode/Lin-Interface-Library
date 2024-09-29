#include <Arduino.h>
#include <LinFrameTransfer.hpp>

// using UART 2 for LinBus, UART 1 for debug log
LinFrameTransfer LinBus(Serial2, Serial1);

int LIN_ScanIDs()
{
    int Result = 0;
    Serial.print("\n\n############# LIN ID scan started\n");
    Serial.print("FIDs confirmed: ");
    // Scan all IDs
    for (int i = 0; i <= 0x3F; i++)
    {
        if (LinBus.readFrame(i))
        {
            Result++;
            Serial.print(i);
            Serial.print(", ");
        }
        delay(10);
    }
    Serial.print("\n############# Chk Sum valid on ");
    Serial.print(Result);
    Serial.print(" Frames\n############# LIN ID scan finished\n\n");

    return Result;
}

void setup()
{
  Serial.begin(115200);

  // configure baud rate
  Serial.print("configure Lin-Bus to 19200 Baud\n");
  LinBus.baud = 19200;

  LIN_ScanIDs();

  Serial.print("May you want to try other Baud rates?\n");
}

void loop()
{
  delay(5000);
}