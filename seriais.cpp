#include <Arduino.h>
#include <SoftwareSerial.h>

const int rxPins[] = {16, 17, 18};
const int deRePins[] = {15, 14, 13};
const int txPins[] = {9, 8, 7};
const int baudRates[] = {19200, 1000000};
const int numSerials = sizeof(rxPins) / sizeof(rxPins[0]);

SerialPIO* serialRS485[numSerials];

const uint8_t commandStrainGauge[] = {0x05, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC4, 0x49};

unsigned long lastMillisPrint;

uint8_t serialBuffer[numSerials];
int serialBufferIndex[numSerials];
double serialValues[numSerials];
int serialPacketCounters[numSerials];
float serialHz[numSerials];

void sendByteRS485(uint8_t byte, int txPin, int baudRate)
{
  // uint32_t bit_delay_ns = 1000000000 / baudRate;
  uint32_t bit_delay = 1000000 / baudRate;

  digitalWrite(txPin, LOW);
  if (baudRate > 500000)
    delayMicroseconds(1);
  else
    delayMicroseconds(bit_delay);

  for (int i = 0; i < 8; i++)
  {
    digitalWrite(txPin, (byte >> i) & 0x01);
    if (baudRate > 500000)
      delayMicroseconds(1);
    else
      delayMicroseconds(bit_delay);
  }

  digitalWrite(txPin, HIGH);
  if (baudRate > 500000)
    delayMicroseconds(1);
  else
    delayMicroseconds(bit_delay);
}

void sendCommandRS485(const uint8_t *buffer, size_t length, int deRePin, int txPin, int baudRate)
{
  digitalWrite(deRePin, HIGH);
  delayMicroseconds(10);

  for (size_t i = 0; i < length; i++)
  {
    sendByteRS485(buffer[i], txPin, baudRate);
  }

  digitalWrite(deRePin, LOW);
  delayMicroseconds(10);
}

bool processPacketStrainGauge(const uint8_t *buffer, int lenght)
{
  if (lenght < 5)
    return false;

  if ((buffer[0] != 0x05 && buffer[0] != 0x0C) || buffer[1] != 0x03)
    return false;

  uint8_t bytesDeDados = buffer[2];

  if (lenght < (3 + bytesDeDados + 2))
    return false;

  for (int i = 0; i < 5; i++)
  {
    // C#: int start = 3 + i * 4;
    int start = 3 + (i * 4);

    // Segurança para não ler fora do array
    if (start + 3 >= lenght)
      break;

    int32_t raw = 0;

    raw |= (int32_t)buffer[start + 1];       // Byte 0 (LSB)
    raw |= (int32_t)buffer[start + 0] << 8;  // Byte 1
    raw |= (int32_t)buffer[start + 3] << 16; // Byte 2
    raw |= (int32_t)buffer[start + 2] << 24; // Byte 3 (MSB)

    double valorLido = raw * 0.1;

    valuesStrain[i] = valorLido;
  }

  return true;
}

// bool processPacketAccel(const uint8_t *buffer, int length)
// {
//   // 1. Validação de Cabeçalho (0xAB 0xCD) e Rodapé (0x88)
//   if (length < 16)
//     return false;
//   if (buffer[0] != 0xAB || buffer[1] != 0xCD)
//     return false;
//   if (buffer[15] != 0x88)
//     return false;

//   // 2. Conversão BIG ENDIAN (O byte mais alto vem primeiro)
//   // O código do sensor faz: data[2] = (acc.x >> 8) & 0xFF; -> Byte Alto
//   //                         data[3] = acc.x & 0xFF;        -> Byte Baixo

//   // Ax
//   valuesAccel[0] = (int16_t)((buffer[2] << 8) | buffer[3]);
//   // Ay
//   valuesAccel[1] = (int16_t)((buffer[4] << 8) | buffer[5]);
//   // Az
//   valuesAccel[2] = (int16_t)((buffer[6] << 8) | buffer[7]);

//   // Gx
//   valuesAccel[3] = (int16_t)((buffer[8] << 8) | buffer[9]);
//   // Gy
//   valuesAccel[4] = (int16_t)((buffer[10] << 8) | buffer[11]);
//   // Gz
//   valuesAccel[5] = (int16_t)((buffer[12] << 8) | buffer[13]);

//   // Temp (apenas 1 byte no protocolo original, cast para int)
//   valuesAccel[6] = (int16_t)buffer[14];

//   return true;
// }

bool processAccel(const uint8_t *buffer)
{
  // Header 0xAB 0xCD
  if (buffer[0] != 0xAB || buffer[1] != 0xCD)
    return false;

  // Big Endian Parsing
  valuesAccel[0] = (int16_t)((buffer[2] << 8) | buffer[3]); // Ax
  valuesAccel[1] = (int16_t)((buffer[4] << 8) | buffer[5]); // Ay
  valuesAccel[2] = (int16_t)((buffer[6] << 8) | buffer[7]); // Az
  // ... ignorando giroscopio e temp por enquanto pra ser rapido
  return true;
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(DE_RE_PIN1, LOW);
  digitalWrite(TX_PIN1, HIGH);
  digitalWrite(DE_RE_PIN2, LOW);
  digitalWrite(TX_PIN2, HIGH);

  Serial.begin(115200);
  Serial.println("Iniciando...");

  delay(1000);

  SerialRS4851.begin(BAUD_RATE1);
  // Serial.println("Serial RS4851 iniciada.");
  // if (SerialRS4851.available())
  // {
  //   Serial.println("RS4851 disponível");
  // }
  // else
  // {
  //   Serial.println("RS4851 não disponível");
  // }

  SerialRS4852.begin(BAUD_RATE2);
  // Serial.println("Serial RS4852 iniciada.");
  // if (SerialRS4852.available())
  // {
  //   Serial.println("RS4852 disponível");
  // }
  // else
  // {
  //   Serial.println("RS4852 não disponível");
  // }
}

void loop()
{
  // if (millis() - lastMillis >= 50)
  // {
  //   // Serial.println("Enviando comando...");
  //   // sendCommandRS485(commandModbus, sizeof(commandModbus), DE_RE_PIN1, TX_PIN1, BAUD_RATE1);
  //   lastMillis = millis();
  // }

  // while (SerialRS4851.available())
  // {
  //   int byteLido = SerialRS4851.read();

  //   if (rxIndex < 25)
  //   {
  //     rxBuffer[rxIndex] = byteLido;
  //     rxIndex++;
  //   }
  // }

  // if (rxIndex >= 25)
  // {
  //   if (processPacketStrainGauge(rxBuffer, sizeof(rxBuffer)))
  //   {
  //     packetCounter++; // Conta pacote válido
  //     unsigned long now = millis();

  //     // A cada 1000ms (1 segundo), calcula a média
  //     if (now - lastFreqTime >= 1000) {
  //        // Calcula Hz
  //        currentHz = (float)packetCounter / ((now - lastFreqTime) / 1000.0);

  //        // Reseta contadores
  //        packetCounter = 0;
  //        lastFreqTime = now;
  //     }
  //     // ----------------------------

  //     Serial.print("[");
  //     Serial.print(currentHz, 1); // Imprime com 1 casa decimal
  //     Serial.print(" Hz] Valores: ");

  //     for (int i = 0; i < 5; i++)
  //     {
  //       Serial.print(values[i], 1);
  //       if (i < 4)
  //         Serial.print(", ");
  //     }
  //     Serial.println();

  //     digitalWrite(LED_BUILTIN, HIGH);
  //     delay(10);
  //     digitalWrite(LED_BUILTIN, LOW);
  //   }
  //   else
  //   {
  //     Serial.println("Erro: Dados corrompidos ou cabeçalho errado.");
  //   }

  //   rxIndex = 0;

  //   // while (SerialRS485.available())
  //   //   SerialRS485.read();
  // }

  unsigned long now = millis();

  while (SerialRS4852.available())
  {
    int b = SerialRS4852.read();

    // Sincronia de Header (Estado 0: Procura 0xAB)
    if (rxIndex2 == 0)
    {
      if (b == 0xAB)
        rxBuffer2[rxIndex2++] = b;
    }
    // Estado 1: Procura 0xCD
    else if (rxIndex2 == 1)
    {
      if (b == 0xCD)
        rxBuffer2[rxIndex2++] = b;
      else
        rxIndex2 = 0; // Falso positivo, volta pro inicio
    }
    // Resto do pacote
    else
    {
      rxBuffer2[rxIndex2++] = b;
      if (rxIndex2 >= 16)
      {
        // Fim do pacote
        if (processAccel(rxBuffer2))
          packetCounter2++;
        rxIndex2 = 0;
      }
    }
  }

  if (now - lastFreqTime2 >= 1000)
  {
    // float hzS = (float)packetCounterStrain / ((now - lastFreqTime) / 1000.0);
    float hzA = (float)packetCounter2 / ((now - lastFreqTime2) / 1000.0);

    // Serial.print("Strain: ");
    // Serial.print(hzS, 0);
    Serial.print(" Hz | ");
    Serial.print("Accel: ");
    Serial.print(hzA, 0);
    Serial.print(" Hz | Ax: ");
    Serial.println(valuesAccel[0]);

    // packetCounterStrain = 0;
    packetCounter2 = 0;
    lastFreqTime2 = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // while (SerialRS4852.available())
  // {
  //   int b = SerialRS4852.read();
  //   Serial.print("0x");
  //   Serial.print(b, HEX);
  //   Serial.print("\n");
  // }
  // Serial.println();
  //   // SINCRONIA: Se o índice é 0, só aceita se for o Header 0xAB
  //   if (rxIndex2 == 0 && b != 0xAB)
  //   {
  //     continue; // Ignora lixo até achar o inicio do pacote
  //   }

  //   if (rxIndex2 < 16)
  //   {
  //     rxBuffer2[rxIndex2++] = b;
  //   }
  // }

  // if (rxIndex2 >= 16)
  // {
  //   if (processPacketAccel(rxBuffer2, 16))
  //   {
  //     packetCounter2++;

  //     // Logica de Hz Accel
  //     if (millis() - lastFreqTime2 >= 1000)
  //     {
  //       hzAccel = (float)packetCounter2 / ((millis() - lastFreqTime2) / 1000.0);
  //       packetCounter2 = 0;
  //       lastFreqTime2 = millis();

  //       // --- PRINT DE DEBUG UNIFICADO (A CADA SEGUNDO) ---
  //       Serial.println("========================================");

  //       Serial.print("[STRAIN] ");
  //       Serial.print(hzStrain, 1);
  //       Serial.print(" Hz | V1: ");
  //       Serial.println(valuesStrain[0], 2);

  //       Serial.print("[ACCEL ] ");
  //       Serial.print(hzAccel, 1);
  //       Serial.print(" Hz | Ax: ");
  //       Serial.print(valuesAccel[0]);
  //       Serial.print(" Ay: ");
  //       Serial.println(valuesAccel[1]);
  //       Serial.println("========================================");

  //       // Pisca LED para indicar vida
  //       digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //     }
  //   }
  //   else
  //   {
  //     // Se processou e deu erro (Checksum ou Header errado), reseta
  //     rxIndex2 = 0;
  //   }
  //   // Limpa Buffer para o próximo
  //   rxIndex2 = 0;
  // }
}