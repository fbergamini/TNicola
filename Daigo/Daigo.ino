#include "DaigoFSM.h"
// Seja bem vindo ao programa do sumô semi-auto, o Daigo!!
// Por que Daigo?! Porque é apelão que só a porra - Veja: https://youtu.be/C-p075gnSTw

void setup()
{
  //Serial.begin(115200);
  //Serial.println("multiChannels");
  // INICIALICAÇÃO DOS SENSORES
  pinMode(PRES_FR, INPUT);
  pinMode(PRES_FL, INPUT);
  // INICIALICAÇÃO FAST SERVO
  CRCArduinoFastServos::attach(SERVO_RIGHT, MOT_RIGHT_OUT); // Conecta Motor da direita
  CRCArduinoFastServos::attach(SERVO_LEFT,  MOT_LEFT_OUT); // Conecta Motor da esquerda
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE, 8 * 2000); // Seta taxa de atualização de 50Hz 10 * 2000 = (2 Servos + 8)*2000
  CRCArduinoFastServos::begin(); // Inicia comunicação
  // INICIALICAÇÃO DAS INTERRUPÇOES - Ver DaigoISR
  PCintPort::attachInterrupt(MOT_RIGHT_IN, calcRight, CHANGE);
  PCintPort::attachInterrupt(MOT_LEFT_IN,  calcLeft,  CHANGE);
  PCintPort::attachInterrupt(AUX1_IN,      calcAux1,  CHANGE);
  PCintPort::attachInterrupt(AUX2_IN,      calcAux2,  CHANGE);
  PCintPort::attachInterrupt(PRES_FR,      presFR,    CHANGE);
  PCintPort::attachInterrupt(PRES_FL,      presFL,    CHANGE);
}

void loop()
{
  // DECLARAÇÃO DE VARIÁVEIS LOCAIS
  // Cria cópia local das variáveis compartilhadas - Ver DaigoH
  static uint16_t unMotRightIn;
  static uint16_t unMotLeftIn;
  static uint16_t unAux1In;
  static uint16_t unAux2In;
  static uint8_t bInterruptFlags;
  // ATUALIZAÇÃO DE VALORES DA LEITURA DO RECEPTOR
  // Verifica se algum canal chamou interrupção desde o último loop
  if (bInterruptFlagsShared)
  {
    noInterrupts(); // Desliga interrupções para não cagar a leitura das variáveis compartilhadas
    bInterruptFlags = bInterruptFlagsShared; // Copia byte de flags interrupções para a variável local
    // Realiza a verificação e possível leitura de cada deltaT para a variável local individualmente
    if (bInterruptFlags & MOT_RIGHT_FLAG)  unMotRightIn = unMotRightInShared;
    if (bInterruptFlags & MOT_LEFT_FLAG)   unMotLeftIn = unMotLeftInShared;
    if (bInterruptFlags & AUX1_FLAG)       unAux1In = unAux1InShared;
    if (bInterruptFlags & AUX2_FLAG)       unAux2In = unAux2InShared;
    bInterruptFlagsShared = 0; // Reseta o byte de flags compartilhado
    interrupts(); // Resume as operações de interrupção
  }
  // MÁQUINA DE ESTADOS
  if (analogRead(LINE) < 200) achouLinha(); // Estado "fundamental" da FSM - Verificar sensor de linha
  else // Caso não encontre a linha, segue com a implementação dos estados
  {
    if (unAux1In > 1800 || unAux1In < 1100) // Verifica estado de busca contínua
    {
      if (!(bStateFlags & BUSCA)) // Caso seja a primeira entrada na rotina de busca, seta o flag do estado
      {
        bStateFlags |= BUSCA;
        if (unAux1In > 1800) bStateFlags |= BUSCADIR; // Verifica a direção inicial de busca
        else bitClear(bStateFlags, 4);
      }
      else
      {
        if (bStateFlags & ACHEI) // Caso tenha encontrado, entra no estado ACHEI
        {
          if (bInterruptFlags & MOT_RIGHT_FLAG) // Libera movimentação para frente e para trás apenas
          {
            CRCArduinoFastServos::writeMicroseconds(SERVO_RIGHT, unMotRightIn);
            CRCArduinoFastServos::writeMicroseconds(SERVO_LEFT,  unMotRightIn);
          }
          if (!digitalRead(PRES_FR) && !digitalRead(PRES_FL)) perdaAlvo(); // Caso tenha perdido o alvo, sai do estado ACHEI e verifica próxima direção de busca
        }
        else // Implementa busca
        {
          if (bStateFlags & BUSCADIR) buscaDir();
          else                         buscaEsq();
        }
      }
    }
    else
    {
      if (bInterruptFlags & AUX2_FLAG) // Verifica Segundo canal auxiliar
      {
        if (unAux2In > 1400 && unAux2In < 1600)    bStateFlags &= B11111100; // Caso o canal esteja em posição neutra, reseta os flags
        if (unAux2In < 1000 && !(bStateFlags & VIRAE)) viraEsq(); // Vira 90 p/ esquerda
        if (unAux2In > 1900 && !(bStateFlags & VIRAD)) viraDir(); // Vira 90 p/ direita       
      }
      // OPERAÇÃO MANUAL
      if (bInterruptFlags & MOT_RIGHT_FLAG)  CRCArduinoFastServos::writeMicroseconds(SERVO_RIGHT, unMotRightIn); // Verifica e implementa o pulso do canal 1
      if (bInterruptFlags & MOT_LEFT_FLAG)   CRCArduinoFastServos::writeMicroseconds(SERVO_LEFT, unMotLeftIn);   // Verifica e implementa o pulso do canal 2
    }
    delay(2);
    //Serial.print(unMotRightIn); Serial.print(" "); Serial.print(unMotLeftIn); Serial.print(" "); Serial.print(unAux1In); Serial.print(" "); Serial.println(unAux2In);
  }
  bInterruptFlags = 0; // Reseta o byte de flags de interrupções
}

