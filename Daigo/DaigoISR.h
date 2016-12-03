/*-----------Daigo Interrupt Services Routines-------------*/
/*                                                         */
/*       Esse arquivo contém as funções que serão          */
/*        chamadas nos eventos de interrupção              */
/*                                                         */
/*---------------------------------------------------------*/

#include "DaigoH.h" // Inclui o arquivo anterior da cadeia

/***** INTERRUPÇÕES DOS SENSORES *****/
void presFR() // Sensor de presença da direita
{
  if (PCintPort::pinState)  bStateFlags |= ACHEI; // Seta flag de encontro frontal
  else bStateFlags |= PERDADIR; // Seta flag de borda de descida DIR
}
void presFL() // Sensor de presença da esquerda
{
  if (PCintPort::pinState)  bStateFlags |= ACHEI; // Seta flag de encontro frontal
  else bitClear(bStateFlags, 5); // Reseta flag de borda de descida DIR
}

/***** INTERRUPÇÕES DO RECEPTOR RF *****/
void calcRight() // Canal 1 - Aileron/Motor da direita
{
  if (PCintPort::pinState) unMotRightInStart = TCNT1; // Guarda valor de Tzero lido no timer
  else
  {
    unMotRightInShared = (TCNT1 - unMotRightInStart) >> 1; // Calcula valor de deltaT
    bInterruptFlagsShared |= MOT_RIGHT_FLAG; // Seta a flag de acordo
  }
}
void calcLeft() // Canal 2 - Elevator/Motor da esquerda
{
  if (PCintPort::pinState) unMotLeftInStart = TCNT1; // Guarda valor de Tzero lido no timer
  else
  {
    unMotLeftInShared = (TCNT1 - unMotLeftInStart) >> 1; // Calcula valor de deltaT
    bInterruptFlagsShared |= MOT_LEFT_FLAG; // Seta a flag de acordo
  }
}
void calcAux1() // Canal 3 - Throttle/Aux1
{
  if (PCintPort::pinState) unAux1InStart = TCNT1; // Guarda valor de Tzero lido no timer
  else
  {
    unAux1InShared = (TCNT1 - unAux1InStart) >> 1; // Calcula valor de deltaT
    bInterruptFlagsShared |= AUX1_FLAG; // Seta a flag de acordo
  }
}
void calcAux2() // Canal 4 - Steering/Aux2
{
  if (PCintPort::pinState) unAux2InStart = TCNT1; // Guarda valor de Tzero lido no timer
  else
  {
    unAux2InShared = (TCNT1 - unAux2InStart) >> 1; // Calcula valor de deltaT
    bInterruptFlagsShared |= AUX2_FLAG; // Seta a flag de acordo
  }
}
