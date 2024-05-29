
/***********************************************************************************
* LecturaMotorEncoder_V7
*   Programa de control y recogida de datos de la garra subactuada de cuatro dedos 
*   con encoders magnéticos en todas las articulaciones y función de apertura y cierre
*   a traves del puerto serie.
*   
*   ---CAMBIOS DE VERSION---
*     Implementacion de PI para control en par de los motores
*     Comandos a traves de UART en par y no en PWM
*   
*   pin 12 --  señal rgb
*   
*   27/04/2021 - 14:00
***********************************************************************************/

#include "AS5048A.h"
#include <Dynamixel2Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


#define PERIODO      10000       // Periodo 10 milisegundos
#define SYNC         22
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
#define DXL_ID_CNT   4        // numero de motores conectados
#define DEG_CONV     0.088    // factor de conversion de posicion en º/bit (motor)
#define RAW2DEG      0.022    // factor de conversion de posicion en º/bit (sensor)
#define CONTROL_P    90
#define CONTROL_I    10
#define CONTROL_D    0
#define LED_PIN      12
#define LED_COUNT    36
#define BRIGHTNESS   70
#define K_P

const uint8_t DXL_DIR_PIN = 8; //DYNAMIXEL Shield
//const uint8_t DXL_ID = 1;
//const float DXL_PROTOCOL_VERSION = 2.0;
const char CLOSE_COMMAND = 'c';  // Comando de control por puerto serie
const char OPEN_COMMAND = 'a';   // Comando de control por puerto serie
const char RELAX_COMMAND = 'r';  // Comando de control por puerto serie
const int N_SENSORES = 8;  // numero de AS5048A en daisy chain
// SYNC READ AND WRITE CONSTANTS
//const uint8_t DXL_ID_CNT = 4;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4};
const uint16_t user_pkt_buf_cap = 128;
const int32_t ABRIR_POS[DXL_ID_CNT] = {150/DEG_CONV, 25/DEG_CONV, 240/DEG_CONV, 120/DEG_CONV};
//const int32_t CERRAR_POS[DXL_ID_CNT] = {40/DEG_CONV, 138/DEG_CONV, 130/DEG_CONV, 230/DEG_CONV};
int32_t CERRAR_PWM[DXL_ID_CNT] = {-250, 250, -250, 250};
//const int32_t ABRIR_PWM[DXL_ID_CNT] = {100, -100, 100, -100};
const int32_t ABRIR_PWM[DXL_ID_CNT] = {60, -60, 60, -60};

// constantes para la lectura del PWM por puerto serie
String str = "";
const char separator = ',';
const int pwmdataLength = 5;
int pwmdata[pwmdataLength];

// Factores de correcion para las medidas angulares
bool FIRST = true;
 float FCa[4] = {0,0,0,0}; //{-7.43, -7.52, -5.94, -7.35};
 float FC1[4] = {0,0,0,0}; // {3.54, 2.91, 3.04, 2.15};
 float FC2[4] = {0,0,0,0}; // {-0.5, -0.5, -0.6, 0.7};

const uint16_t SR_START_ADDR =132; // <-- Present position | 126 <-- present load | 124 <-- present pwm
const uint16_t SR_ADDR_LEN = 4; // <-- tamaño del buffer pos | 2+4+4 <-- load+vel+pos | 2+2+4+4 <-- pwm+load+vel+pos
const uint16_t SW_START_ADDR = 100; // Goal PWM | 116<-- Goal position
const uint16_t SW_ADDR_LEN = 2;

// Definicion de structs
typedef struct sr_data{
  // // int16_t present_pwm;
  //int16_t present_current;
  //int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


// Variables globales
int cerrada=0;      // estado de la pinza
uint32_t tiempo,tiemposiguiente,tiempoinicial;  // Variables de control del bucle
// SYNC READ AND WRITE VARIABLES
uint8_t user_pkt_buf[user_pkt_buf_cap];
sr_data_t sr_data[DXL_ID_CNT];  // sync read
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];
sw_data_t sw_data[DXL_ID_CNT];  // sync write
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

// Prototipos de funciones
void modo(int id);
void calibracion();
void abrir();
void cerrar();
void relax();
void colorSet(uint32_t color);

// Declaracion de clases
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
AS5048A angleSensor(30);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


//This namespace is required to use Control table item names
using namespace ControlTableItem;



/***********************************************************************************
 * Funcion Setup
 ***********************************************************************************/
void setup()
{
  uint8_t i;

  //cli();
  //bitSet(UCSR1A, U2X1); // 
  bitClear(UCSR1A, U2X1); // Poner el bit U2X1 del reg. UCSR1A a 0 para mayor fiabilidad de la uart
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(500000);
  dxl.begin(1000000); // dxl.begin(2000000);

  angleSensor.init();
  // Calibrar
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
    dxl.setGoalCurrent(DXL_ID_LIST[i],150);
    dxl.setGoalPWM(DXL_ID_LIST[i],800);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID_LIST[i], CONTROL_P);    // P
    dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID_LIST[i], CONTROL_I);    // I
    dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID_LIST[i], CONTROL_D);    // D
    
    dxl.torqueOn(DXL_ID_LIST[i]);

    dxl.setGoalPosition(DXL_ID_LIST[i],90/DEG_CONV);
    
  }
  delay(2000);
  calibracion();
  //

  
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_PWM);
    dxl.setGoalPWM(DXL_ID_LIST[i],0);
    
    dxl.torqueOn(DXL_ID_LIST[i]);
  }
  
  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;


  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for(i=0; i<DXL_ID_CNT; i++){
    // Llenar el bufer de posiciones objetivo
    sw_data[i].goal_position = ABRIR_PWM[i];
  
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;


  dxl.syncWrite(&sw_infos); // enviar posiciones [ABRIR]

  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
  
  //uint32_t color = strip.Color(0,0,255); // azul
  //DEBUG_SERIAL.println(color);
  colorSet(strip.Color(0,0,250));
  
  delay(3000);
  
  tiempo=micros();
  tiempoinicial=tiempo;  
  tiemposiguiente=tiempo;
}

/***********************************************************************************
 * Funcion Loop 
 ***********************************************************************************/
void loop()
{
  uint8_t i, recv_cnt;
  char command;
  bool once = true;
  uint32_t tsr;
  String opt;

  //do {
  if(tiempo<tiemposiguiente){
    // La lectura sincrona de los motores tarda 5 ms aprox. en realizarse
    // por lo que solo se puede hacer una en el tiempo de espera (10ms)
    // Si tsr es menor que 5 ms aprox. la lectura no es valida o no ha tenido lugar
    // y se debe repetir
    tiempo=micros();  
 // } while (tiempo<tiemposiguiente);
  }else{
    tiemposiguiente+=PERIODO;
    opt = "";

  
    //Leer encoders y mandar por puerto serial
    angleSensor.read_daisyChain(N_SENSORES); 
    
    // Angulos con respecto al mismo sistema de referencia
    opt += String(angleSensor.daisyReadings[0]*RAW2DEG + FC2[0])+"\t";
    opt += String(angleSensor.daisyReadings[1]*RAW2DEG + FC1[0])+"\t";
    opt += String(-angleSensor.daisyReadings[2]*RAW2DEG + FC2[1])+"\t";
    opt += String(-angleSensor.daisyReadings[3]*RAW2DEG + FC1[1])+"\t";
    opt += String(angleSensor.daisyReadings[4]*RAW2DEG + FC2[2])+"\t";
    opt += String(angleSensor.daisyReadings[5]*RAW2DEG + FC1[2])+"\t";
    opt += String(-angleSensor.daisyReadings[6]*RAW2DEG + FC2[3])+"\t";
    opt += String(-angleSensor.daisyReadings[7]*RAW2DEG + FC1[3])+"\t"; 

    //Leer motores y mandar por puerto serial
    recv_cnt = dxl.syncRead(&sr_infos);// Esta instruccion tarda 4.8 ms aprox.

    opt += String(-sr_data[0].present_position*DEG_CONV + FCa[0])+"\t";  // 101.29 -
    opt += String(sr_data[1].present_position*DEG_CONV + FCa[1])+"\t";  // 78.85 +
    opt += String(-sr_data[2].present_position*DEG_CONV + FCa[2])+"\t";  // 191.4 -
    opt += String(sr_data[3].present_position*DEG_CONV + FCa[3])+"\t";  // 169.66 +

  
    // End of line (DESDE AQUI)
    DEBUG_SERIAL.println(opt);  
  
    if(DEBUG_SERIAL.available()){
      str = Serial.readStringUntil('\n');
      for (int i = 0; i < pwmdataLength-1 ; i++)
      {
         int index = str.indexOf(separator);
         pwmdata[i] = str.substring(0, index).toInt();
         str = str.substring(index + 1);
      }
      // PWM positivos hacen que la garra cierre, PWM negativos hacen que la garra abra
      CERRAR_PWM[0] = -1* pwmdata[0];  // Factor de correccion -1 (motor orientado en sentido inverso)
      CERRAR_PWM[1] = +1* pwmdata[1];  // Factor de correccion +1 (motor orientado en sentido directo)
      CERRAR_PWM[2] = -1* pwmdata[2];  // Factor de correccion -1 (motor orientado en sentido inverso)
      CERRAR_PWM[3] = +1* pwmdata[3];  // Factor de correccion +1 (motor orientado en sentido directo)
      cerrar();
    }
    // HASTAS AQUI tarda 1.1 ms aprox.


  //Serial.println(compresionMuelle(angleSensor.daisyReadings[1]*RAW2DEG + FC1[0], angleSensor.daisyReadings[0]*RAW2DEG + FC2[0], -sr_data[0].present_position*DEG_CONV + FCa[0]));
  } 
}


/***********************************************************************************
 * Funciones de control de los servos
 ***********************************************************************************/
void modo(int id)
{
  dxl.torqueOff(id);

  dxl.setOperatingMode(id, OP_EXTENDED_POSITION);

  dxl.setGoalCurrent(id,100);
  dxl.setGoalPWM(id,800);
 
  dxl.writeControlTableItem(POSITION_P_GAIN, id, 20);   // P
  dxl.writeControlTableItem(POSITION_I_GAIN, id, 5);    // I
  dxl.writeControlTableItem(POSITION_D_GAIN, id, 0);    // D

  dxl.torqueOn(id);
}

void calibracion(){
  angleSensor.read_daisyChain(N_SENSORES);
  
  FCa[0] = +90;
  FCa[1] = -90;
  FCa[2] = +90;
  FCa[3] = -90;

  FC1[0] = 61 -(angleSensor.daisyReadings[1]*RAW2DEG);
  FC1[1] = 61 -(-angleSensor.daisyReadings[3]*RAW2DEG);
  FC1[2] = 61 -(angleSensor.daisyReadings[5]*RAW2DEG);
  FC1[3] = 61 -(-angleSensor.daisyReadings[7]*RAW2DEG);

  FC2[0] = 0-(angleSensor.daisyReadings[0]*RAW2DEG);
  FC2[1] = 0-(-angleSensor.daisyReadings[2]*RAW2DEG);
  FC2[2] = 0-(angleSensor.daisyReadings[4]*RAW2DEG);
  FC2[3] = 0-(-angleSensor.daisyReadings[6]*RAW2DEG);
}


void abrir()
{
  sw_data[0].goal_position = ABRIR_PWM[0];
  sw_data[1].goal_position = ABRIR_PWM[1];
  sw_data[2].goal_position = ABRIR_PWM[2];
  sw_data[3].goal_position = ABRIR_PWM[3];
  sw_infos.is_info_changed = true;
  dxl.syncWrite(&sw_infos); // */ 

  cerrada=0;
}

void cerrar()
{
  sw_data[0].goal_position = CERRAR_PWM[0];
  sw_data[1].goal_position = CERRAR_PWM[1];
  sw_data[2].goal_position = CERRAR_PWM[2];
  sw_data[3].goal_position = CERRAR_PWM[3];
  sw_infos.is_info_changed = true;
  dxl.syncWrite(&sw_infos); // */
  
  cerrada=1;
}

void relax()
{
  sw_data[0].goal_position = 0;
  sw_data[1].goal_position = 0;
  sw_data[2].goal_position = 0;
  sw_data[3].goal_position = 0;
  sw_infos.is_info_changed = true;
  dxl.syncWrite(&sw_infos); // */
  
  cerrada=1;
}

float compresionMuelle(float theta1, float theta2, float thetaa){
  int a, b, c, d, e;
  float q1, q2, qa, p2x, p2y, p3x, p3y, c_prima, compresion, gamma;
  
  //longitudes de los eslabones del dedo
  a = 40; b = 20; c = 59.5; d = 50; e = 28;
  
  //calculo de los angulos en rad
  q1 = PI/180 * theta1;
  q2 = PI/180 * theta2;
  qa = PI/180 * thetaa;
  gamma = acos(15.4/e);
  
  //calculo de la compresion del muelle
  p2x = a*cos(q1) + b*cos(q1 + q2 - PI/2 );
  p2y = a*sin(q1) + b*sin(q1 + q2 - PI/2 );
  p3x = 15.4 + d*cos(qa); //p3x = e*cos(-gamma) + d*cos(qa);
  p3y = -22.4 + d*sin(qa); //p3y = e*sin(-gamma) + d*sin(qa);
  c_prima = sqrt(pow(p2x-p3x,2)+pow(p2y-p3y,2));
  /*DEBUG_SERIAL.print(p2x);
  DEBUG_SERIAL.print("  ");
  DEBUG_SERIAL.print(p2y);
  DEBUG_SERIAL.print("  ");
  DEBUG_SERIAL.print(p3x);
  DEBUG_SERIAL.print("  ");
  DEBUG_SERIAL.print(p3y);
  DEBUG_SERIAL.print("  ");*/
  compresion = c - c_prima;
  return compresion;
}


void colorSet(uint32_t color) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
  }
}

/*   ALTERNATIVA AL SWITCH CASE
 
      if(command == CLOSE_COMMAND)
        cerrar();
      else if(command == OPEN_COMMAND)
        abrir();
      else if(command == RELAX_COMMAND)
        relax();
 */
