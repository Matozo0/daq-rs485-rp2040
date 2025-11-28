#include <Arduino.h>
#include <SerialPIO.h>

// --- PINAGEM ACELERÔMETRO ---
#define RX_PIN1 16
#define DE_RE_PIN1 15
#define TX_PIN1 9

#define RX_PIN2 17
#define DE_RE_PIN2 14
#define TX_PIN2 8

#define BAUD_RATE 1000000 // 1 Mbps

// Instância do PIO com buffer grande (1024 bytes) para aguentar o fluxo de 1kHz
SerialPIO SerialAccel1(NOPIN, RX_PIN1, 1024);
SerialPIO SerialAccel2(NOPIN, RX_PIN2, 1024);

// --- VARIÁVEIS ---
uint8_t rxBuffer1[20];
int rxIndex1 = 0;
uint8_t rxBuffer2[20];
int rxIndex2 = 0;

// Ax, Ay, Az, Gx, Gy, Gz, Temp
int16_t valuesAccel1[7];
int16_t valuesAccel2[7];

// Contadores de Frequência
unsigned long lastFreqTime1 = 0;
int packetCounter1 = 0;
float hzAccel1 = 0.0;

unsigned long lastFreqTime2 = 0;
int packetCounter2 = 0;
float hzAccel2 = 0.0;
// ============================================================================
// PROCESSAMENTO DO PACOTE (BIG ENDIAN)
// ============================================================================
bool processAccel(const uint8_t *buffer, int num)
{
  // Validação Final (embora a máquina de estados já garanta o header)
  if (buffer[0] != 0xAB || buffer[1] != 0xCD)
    return false;
  if (buffer[15] != 0x88)
    return false; // Verifica rodapé (CRC placeholder)

  if (num == 1)
  {
    valuesAccel1[0] = (int16_t)((buffer[2] << 8) | buffer[3]); // Ax
    valuesAccel1[1] = (int16_t)((buffer[4] << 8) | buffer[5]); // Ay
    valuesAccel1[2] = (int16_t)((buffer[6] << 8) | buffer[7]); // Az

    valuesAccel1[3] = (int16_t)((buffer[8] << 8) | buffer[9]);   // Gx
    valuesAccel1[4] = (int16_t)((buffer[10] << 8) | buffer[11]); // Gy
    valuesAccel1[5] = (int16_t)((buffer[12] << 8) | buffer[13]); // Gz

    valuesAccel1[6] = (int16_t)buffer[14]; // Temperatura
  }
  else {
    valuesAccel2[0] = (int16_t)((buffer[2] << 8) | buffer[3]); // Ax
    valuesAccel2[1] = (int16_t)((buffer[4] << 8) | buffer[5]); // Ay
    valuesAccel2[2] = (int16_t)((buffer[6] << 8) | buffer[7]); // Az

    valuesAccel2[3] = (int16_t)((buffer[8] << 8) | buffer[9]);   // Gx
    valuesAccel2[4] = (int16_t)((buffer[10] << 8) | buffer[11]); // Gy
    valuesAccel2[5] = (int16_t)((buffer[12] << 8) | buffer[13]); // Gz
    valuesAccel2[6] = (int16_t)buffer[14]; // Temperatura
  }
  // Conversão Big Endian: (Byte Alto << 8) | Byte Baixo
  

  return true;
}

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // Configura RS485 para modo ESCUTA (DE/RE = LOW)
  // pinMode(TX_PIN1, OUTPUT);
  pinMode(DE_RE_PIN1, OUTPUT);
  pinMode(DE_RE_PIN2, OUTPUT);
  // digitalWrite(TX_PIN1, HIGH);
  digitalWrite(DE_RE_PIN1, LOW); // Sempre ouvindo
  digitalWrite(DE_RE_PIN2, LOW);

  Serial.begin(115200);
  delay(2000);
  Serial.println("--- Iniciando Teste Acelerometro (1 Mbps) ---");

  SerialAccel1.begin(BAUD_RATE);
  SerialAccel2.begin(BAUD_RATE);
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop()
{
  // // Loop de leitura ultra-rápido (esvazia o buffer do PIO)
  // for (int i = 0; i < 7; ++i) valuesAccel1[i] = 0; // Limpa o último valor para evitar lixo
  // for (int i = 0; i < 7; ++i) valuesAccel2[i] = 0; // Limpa o último valor para evitar

  while (SerialAccel1.available())
  {
    int b = SerialAccel1.read();

    // --- MÁQUINA DE ESTADOS DE SINCRONIA ---

    // Estado 0: Procurando o primeiro byte do cabeçalho (0xAB)
    if (rxIndex1 == 0)
    {
      if (b == 0xAB)
      {
        rxBuffer1[rxIndex1++] = b;
      }
    }
    // Estado 1: Procurando o segundo byte do cabeçalho (0xCD)
    else if (rxIndex1 == 1)
    {
      if (b == 0xCD)
      {
        rxBuffer1[rxIndex1++] = b;
      }
      else
      {
        rxIndex1 = 0; // Não era o cabeçalho, reseta e busca 0xAB de novo
      }
    }
    // Estado 2: Lendo o resto do pacote (Dados)
    else
    {
      rxBuffer1[rxIndex1++] = b;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      // Se completou 16 bytes
      if (rxIndex1 >= 16)
      {
        if (processAccel(rxBuffer1, 1))
        {
          packetCounter1++;
        }
        rxIndex1 = 0; // Prepara para o próximo pacote
      }
    }
  }

  while (SerialAccel2.available())
  {
    int b = SerialAccel2.read();

    // --- MÁQUINA DE ESTADOS DE SINCRONIA ---

    // Estado 0: Procurando o primeiro byte do cabeçalho (0xAB)
    if (rxIndex2 == 0)
    {
      if (b == 0xAB)
      {
        rxBuffer2[rxIndex2++] = b;
      }
    }
    // Estado 1: Procurando o segundo byte do cabeçalho (0xCD)
    else if (rxIndex2 == 1)
    {
      if (b == 0xCD)
      {
        rxBuffer2[rxIndex2++] = b;
      }
      else
      {
        rxIndex2 = 0; // Não era o cabeçalho, reseta e busca 0xAB de novo
      }
    }
    // Estado 2: Lendo o resto do pacote (Dados)
    else
    {
      rxBuffer2[rxIndex2++] = b;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      // Se completou 16 bytes
      if (rxIndex2 >= 16)
      {
        if (processAccel(rxBuffer2, 2))
        {
          packetCounter2++;
        }
        rxIndex2 = 0; // Prepara para o próximo pacote
      }
    }
  }

  // --- RELATÓRIO A CADA 1 SEGUNDO ---
  unsigned long now = millis();
  if (now - lastFreqTime1 >= 1000)
  {
    // Cálculo de Hz
    hzAccel1 = (float)packetCounter1 / ((now - lastFreqTime1) / 1000.0);
    hzAccel2 = (float)packetCounter2 / ((now - lastFreqTime1) / 1000.0);

    Serial.print("[RATE] ");
    Serial.print(hzAccel1, 1);
    Serial.print(" Hz | AX: ");
    Serial.print(valuesAccel1[0]);
    Serial.print(" | AY: ");
    Serial.print(valuesAccel1[1]);
    Serial.print(" | AZ: ");
    Serial.print(valuesAccel1[2]);
    Serial.print(" | [RATE2] ");
    Serial.print(hzAccel2, 1);
    Serial.print(" Hz | AX2: ");
    Serial.print(valuesAccel2[0]);
    Serial.print(" | AY2: ");
    Serial.print(valuesAccel2[1]);
    Serial.print(" | AZ2: ");
    Serial.println(valuesAccel2[2]);

    // Reseta contadores
    packetCounter1 = 0; 
    packetCounter2 = 0;
    lastFreqTime1 = now;
  }
}