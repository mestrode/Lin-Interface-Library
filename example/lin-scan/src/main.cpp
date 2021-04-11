#include <Arduino.h>
#include <Lin_Interface.hpp>

// using UART 1 for LinBus
Lin_Interface LinBus(1);

int LIN_ScanIDs()
{
    int Result = 0;
    Serial.print("\n\n############# LIN ID scan started\n");
    // Scan all IDs
    for (int i = 0; i <= 0x3F; i++)
    {
        if (LinBus.readFrame(i))
        {
            Result++;
        }
        delay(10);
    }
    Serial.print("############# Chk Sum valid on ");
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