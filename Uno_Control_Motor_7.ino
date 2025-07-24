// Dr. Sergio Miguel Delfín Prieto
// FI-UAQ
// Para la materia de Ingenieria de Control
// Ingeniería Industrial y de Manufactura

#include <TimerOne.h>
int chA = 2; // Pin de interrupción externa
int chB = 4;

int in3 = 5;  //
int in4 = 9;  // puente h
int pwm6 = 6; //

const int swSerial = 8; // Switch para enviar datos por el serial
int swState = 0;
int flagTimer = 1;

//int posicionAnterior = 0;
volatile int posicion = 0;

// Referencia
//int ref = 500;
// Referencia variable
float ref=0;
int T=600;
int i=0;

/*   |        -----------        |--------|  500
 *   |        |         |        |        |
 *   |        |         |        |        |
 *   |        |         |        |        |
 *   |--------|         |--------|        |- 0
 *   T = 10; 5 seg arriba, 5 seg abajo
 *  Amplitud: 500
 */

// Error de posición
int error = 0;
// Error de posición en el ciclo anterior
int error_1 = 0;
// Error multiplicado Kp
int errorKp=0;
// Velocidad del Error multiplicado Kd
int errorKd=0;
// Integral numérica
int errorKi = 0;
int errorKi_1 = 0;
// Control
int control = 0;
int controlPWM = 0;
// Ganancias del PID
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

void setup() {
  Serial.begin(115200);
  // Switch envio de datos por el serial
  pinMode(swSerial,INPUT);
  // Encoder
  pinMode(chA,INPUT);
  pinMode(chB,INPUT);
  // Puente H
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(pwm6,OUTPUT);
  // Servicio de interrupción
  attachInterrupt(digitalPinToInterrupt(chA),encoderISR,RISING);
  // Inicializar temporizador: 1000=1ms
  Timer1.initialize(10000); // 10,000 = 10 ms = 0.010 s
  Timer1.attachInterrupt(Muestreo);
}

void Muestreo(){
  control = errorKp + errorKi + errorKd;
  flagTimer = 1;
  i++;
}

void loop() {
  //Serial.println(posicion);
  //-- Generar la referencia --
  if(i >= 0 && i < T/2){
    ref = 500;
  }
  else if(i >= T/2 && i < T){
    ref = 0;
  }
  else{
    i = 0;
  }
  // --- Error de posición
  error = ref - posicion;
  // --- Acción proporcional
  errorKp = Kp * error;
  // --- Acción derivativa (DERIVACION NUMERICA)
  errorKd = Kd*(error-error_1)/0.01;
  // --- Acción integral   (INTEGRACION NUMERICA)
  //1. regla rectangular hacia adelante 
  errorKi = errorKi_1 + Ki*error*0.01;
  errorKi_1 = errorKi;
  // --- Guardar el ultimo valor del error
  error_1=error;
  // --- Generar el PWM
  controlPWM = abs(control);

  // --- Generar el cambio de giro del motor
  if(control >= 0){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(pwm6,controlPWM);
  }
  else{
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(pwm6,controlPWM);
  }
  // --- 
  // Enviar datos solo cuando esté el swSerial1==1 && flagTimer==1
  swState = digitalRead(swSerial);
  if(swState == HIGH && flagTimer==1){
    Serial.print(ref);
    Serial.print("\t");
    Serial.print(posicion);
    Serial.print("\t");
    Serial.println(control);
    flagTimer = 0;
  }
  // Limpia el búfer serial (de salida)
  Serial.flush();
}

void encoderISR(){
  if(digitalRead(chB) == HIGH){
    posicion ++;
  }
  else{
    posicion --;
  }
}
