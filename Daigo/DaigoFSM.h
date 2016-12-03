/*---------------Daigo Finite-State Machine----------------*/
/*                                                         */
/*  Esse arquivo contém os estados da máquina de estados   */
/*        contida na função principal                      */
/*                                                         */
/*---------------------------------------------------------*/

#include "DaigoISR.h" // Inclui o arquivo anterior da cadeia

/***** PROTÓTIPOS *****/
void achouLinha(); // Encontrou linha branca
void buscaDir();   // Busca no sentido horário
void buscaEsq();   // Busca no sentido anti-horário
void viraDir();    // Vira 90 graus p/ direita
void viraEsq();    // Vira 90 graus p/ esquerda
void perdaAlvo();  // Rotina auxiliar relizada no momento da perda de contato visual com o meliante

/***** IMPLEMENTAÇÃO DAS FUNÇÕES *****/
void achouLinha() //  MUDAR - Por enquanto, essa rotina só retorna durante um período de tempo controlado por delay
{
  CRCArduinoFastServos::writeMicroseconds(SERVO_RIGHT, 1000);
  CRCArduinoFastServos::writeMicroseconds(SERVO_LEFT , 1000);
  delay(150);
}
void buscaDir()
{
  CRCArduinoFastServos::writeMicroseconds(SERVO_RIGHT, 1800);
  CRCArduinoFastServos::writeMicroseconds(SERVO_LEFT , 1200);
  //Serial.println("BUSCA DIR");
}
void buscaEsq()
{
  CRCArduinoFastServos::writeMicroseconds(SERVO_RIGHT, 1200);
  CRCArduinoFastServos::writeMicroseconds(SERVO_LEFT , 1800);
  //Serial.println("BUSCA ESQ");
}
void viraDir() // Estado termporizado!! - POSSIVELMENTA ESTA ERROADO, TEM QUE VERIFICAR NO ROBO
{
  CRCArduinoFastServos::writeMicroseconds(SERVO_RIGHT, 1000);
  CRCArduinoFastServos::writeMicroseconds(SERVO_LEFT , 2000);
  //Serial.println("Vira Direita");
  delay(250);
  bStateFlags |= VIRAD; // Seta flag de estado
}
void viraEsq() // Estado termporizado!! - POSSIVELMENTA ESTA ERROADO, TEM QUE VERIFICAR NO ROBO
{
  CRCArduinoFastServos::writeMicroseconds(SERVO_RIGHT, 2000);
  CRCArduinoFastServos::writeMicroseconds(SERVO_LEFT , 1000);
  //Serial.println("Vira Esquerda");
  delay(250);
  bStateFlags |= VIRAE;
}
void perdaAlvo()
{
  bStateFlags ^= ACHEI;
  if (bStateFlags & PERDADIR) bStateFlags |= BUSCADIR;
  else bitClear(bStateFlags, 4);
}

