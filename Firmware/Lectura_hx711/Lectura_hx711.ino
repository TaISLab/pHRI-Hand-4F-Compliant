//amarillo A3, naranja A2, azul A1, verde A0

#include <Q2HX711.h>

Q2HX711 scale_1(A1, A0);
Q2HX711 scale_2(A3, A2);

#define PERIODO 20 // Leemos 50 muestras por segundo de cada uno

long tiempo, tiemposiguiente,tiempoinicial;
float tara1, tara2=0;
float Read1, Read2;

#define STARTPIN 11

void setup() {
  int n;
  Serial.begin(250000);

  // Prepara señal de sincronización
  pinMode(STARTPIN, INPUT_PULLUP);

  // Espera a que el otro arduino inicie y pongala salida a nivel alto
  // delay(1000);

  // Inicializa el tiempo por si el otro ya ha empezado
    tiempo= millis();
    tiemposiguiente=tiempo;
    tiempoinicial=tiempo;

tara1=0;
tara2=0;
for (n=0; n<10;n++) {
  tara1 = tara1+(scale_1.read()/10); 
  tara2 = tara2+(scale_2.read()/10); 
  }

delay(1000);

tara1=0;
tara2=0;
for (n=0; n<10;n++) {
  tara1 = tara1+(scale_1.read()/10); 
  tara2 = tara2+(scale_2.read()/10); 
  }

  
}


float escala1=-(0.834/326000)*9.81*1.775/17*10.5;
float escala2=(0.834/326000)*9.81*1.775/17*10.5;

void loop() {

  //Sincroniza el periodo del bucle
  do {
  tiempo=millis();
  } while (tiempo<tiemposiguiente);
  tiemposiguiente+=PERIODO;
  Read1=(scale_1.read()-tara1)*escala1;
  Read2=(scale_2.read()-tara2)*escala2;

  //Serial.println(Read1+Read2,3);
  
  Serial.print(Read1, 3);
  Serial.print(" ");
  Serial.print(Read2, 3);
  Serial.write(13);
  Serial.write(10);

}
