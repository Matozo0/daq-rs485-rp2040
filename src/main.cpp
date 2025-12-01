#include <Arduino.h>
#include "SerialRS485.h"

RS485Manager rs485Manager;

const int rxPins[] = {16, 17, 18};
const int deRePins[] = {15, 14, 13};
const int txPins[] = {9, 8, 7};
const int numPorts = sizeof(rxPins)/sizeof(rxPins[0]);

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < numPorts; ++i)
  {
    rs485Manager.addPort(txPins[i], rxPins[i], deRePins[i]);
  }
  rs485Manager.beginAll(1000000);
}

void loop()
{
  for (size_t i = 0; i < rs485Manager.portCount(); ++i)
  {
    SerialRS485* port = rs485Manager.getPort(i);
    if (port && port->available())
    {
      int byteRead = port->read();
      Serial.print("Port ");
      Serial.print(i+1);
      Serial.print(" received: ");
      Serial.println(byteRead, HEX);
    }
  }
}