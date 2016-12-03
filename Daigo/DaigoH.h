/*---------------------Daigo Header------------------------*/
/*                                                         */
/*       Esse arquivo contém todas as variáveis que        */
/*        serão usadas no decorrer do resto do programa    */
/*                                                         */
/*---------------------------------------------------------*/

// Bibliotecas de uso geral
#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>

/***** PINOS DE ENTRADA *****/
// Sensores
int LINE =  A5; // Sensor de linha
#define PRES_FR 3 // Sensor de presença dianteiro direito
#define PRES_FL 4 // Sensor de presença dianteiro esquerdo
// Canais principais
#define MOT_RIGHT_IN 2
#define MOT_LEFT_IN  5
// Canais secundários
#define AUX1_IN 6
#define AUX2_IN 7

/***** PINOS DE SAÍDA E VARIÁVEIS FAST SERVO *****/
// Motores
#define MOT_RIGHT_OUT 8
#define MOT_LEFT_OUT  9
// Indíces dos servos - Basicamente define quem vai ser atualizado antes
#define SERVO_RIGHT       0
#define SERVO_LEFT        1
#define SERVO_FRAME_SPACE 2

/***** FLAG BYTES *****/
// Definiçao de cada flag byte e seus 8 bits
// Basicamente trata-se de um conjunto de bits para uso de registros auxiliares
// INTERRUPÇÕES
volatile uint8_t bInterruptFlagsShared;
#define MOT_RIGHT_FLAG B00000001
#define MOT_LEFT_FLAG  B00000010
#define AUX1_FLAG      B00000100
#define AUX2_FLAG      B00001000
// ESTADOS
uint8_t bStateFlags;
#define VIRAD    B00000001 // Registra estado de virada de 90 graus p/ DIR
#define VIRAE    B00000010 // Registra estado de virada de 90 graus p/ ESQ
#define ACHEI    B00000100 // Registra estado de fim de busca/encontro frontal
#define BUSCA    B00001000 // Registra estado de busca
#define BUSCADIR B00010000 // Registra a direção de busca. 1 - DIR, 0 - ESQ
#define PERDADIR B00100000 // Registra a borda de descida do sensor de presença da direita 

/***** VARIÁVEIS COMPARTILHADAS E DE TIMER *****/
// As variáveis compartilhadas são atualizadas na função ISR para, mais tarde, serem lidas no loop
// Essas variáveis consistem em, basicamente, a largura de pulso em microssegundos de cada canal do receptor IR
volatile uint16_t unMotRightInShared;
volatile uint16_t unMotLeftInShared;
volatile uint16_t unAux1InShared;
volatile uint16_t unAux2InShared;
// Se as variáveis de cima /\ guardam um deltaT, essas a seguir guardam o Tzero do pulso
uint16_t unMotRightInStart;
uint16_t unMotLeftInStart;
uint16_t unAux1InStart;
uint16_t unAux2InStart;
