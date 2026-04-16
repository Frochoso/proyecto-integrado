// Nicolas Marcos, Lucas Michielli, Javier Frochoso

#include <Servo.h>
#define SCL_Pin A5
#define SDA_Pin A4

//Definicion de los mapas de bits
unsigned char start01[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01 };
unsigned char front[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x12, 0x09, 0x12, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char back[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x48, 0x90, 0x48, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char left[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x28, 0x10, 0x44, 0x28, 0x10, 0x44, 0x28, 0x10, 0x00 };
unsigned char right[] = { 0x00, 0x10, 0x28, 0x44, 0x10, 0x28, 0x44, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char batisenal[] = { 0x00, 0x3c, 0x3e, 0x7c, 0x7c, 0x1c, 0x1c, 0x7e, 0x7e, 0x1c, 0x1c, 0x7c, 0x7c, 0x3e, 0x1c, 0x08 };  //Hemos reemplazado la señal de stop por la batseñal
unsigned char clear[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char speed_a[] = { 0x00, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0xff, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x00, 0x00 };
unsigned char speed_d[] = { 0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0xff, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00 };

//Creacion de los objetos servo
Servo lower_arm;
Servo pinza;
Servo cuello;

//Definicion de la posicion inicial de los motores del brazo
int lower_arm_position = 130, claw_position = 5;

int left_ctrl = 2;   //Definir el pin de los motores del grupo B
int left_pwm = 5;    //Definir el pin PWM de los motores del grupo B
int right_ctrl = 4;  //Definir el pin de los motores del grupo A
int right_pwm = 6;   //Definir el pin PWM de los motores del grupo A
int speeds = 110;

// Definicion de los pines del sensor siguelineas
int L_pin = 11;
int M_pin = 7;
int R_pin = 8;
int L_val, M_val, R_val;

// Definicion de los pines del sensor ultrasonido
int trigPin = 12;
int echoPin = 13;
int distance, distance_l, distance_r;
char BLE_val;  //Conexion bluetooth

void setup() {
  delay(1500);
  Serial.begin(9600);           //Definir la tasa de baudios a 9600
  pinMode(left_ctrl, OUTPUT);   //Definir el pin de los motores del grupo B a OUTPUT
  pinMode(left_pwm, OUTPUT);    //Definir el pin PWM de los motores del grupo B a OUTPUT
  pinMode(right_ctrl, OUTPUT);  //Definir el pin de los motores del grupo A a OUTPUT
  pinMode(right_pwm, OUTPUT);   //Definir el pin PWM de los motores del grupo A a OUTPUT
  delay(300);

  // Definicion de los pines del sensor siguelineas a INPUT
  pinMode(L_pin, INPUT);
  pinMode(M_pin, INPUT);
  pinMode(R_pin, INPUT);

  // Definicion de los pines del sensor ultrasonido a OUTPUT e INPUT
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(SCL_Pin, OUTPUT);  //Definir el reloj serial en output
  pinMode(SDA_Pin, OUTPUT);  //Definir el pin de datos a output
  matrix_display(clear);
  matrix_display(batisenal);  //display start01 expression pattern

  //Definir los servos a pines concretos
  lower_arm.attach(10);
  pinza.attach(9);
  cuello.attach(A0);

  //Mover los servos a una posición concreta
  cuello.write(90);
  lower_arm.write(lower_arm_position);
  pinza.write(claw_position);
}

void loop() {

  //Si recibe un mensaje por bluetooth, lo almacena e imprime por serial
  if (Serial.available() > 0) {
    BLE_val = Serial.read();
    Serial.println(BLE_val);
  }

  //Dependiendo del valor recibido, se ejecuta una funcion
  switch (BLE_val) {
    //vv Inicio control manual del robot mediante cruceta de direccion
    case 'F':
      car_front();
      matrix_display(clear);
      matrix_display(front);
      break;

    case 'B':
      car_back();
      matrix_display(clear);
      matrix_display(back);
      break;

    case 'L':
      car_left();
      matrix_display(clear);
      matrix_display(left);
      break;

    case 'R':
      car_right();
      matrix_display(clear);
      matrix_display(right);
      break;

    case 'S':
      car_Stop();
      matrix_display(clear);
      matrix_display(batisenal);
      break;
      //^^ Fin control manual del robot mediante cruceta de direccion

    case 'i':
      tracking();  // Siguelineas con evitación de obstaculos
      break;

    case 'f':
      lower_arm_fwd();  //mover brazo adelante
      break;

    case 'b':
      lower_arm_bwd();  //mover brazo atrás
      break;

    case 'Q':
      open_claw();  //Abre pinza
      break;
    case 'E':
      close_claw();  //Cierra pinza
      break;
  }
}

void car_front()  //El coche va hacia delante
{
  digitalWrite(left_ctrl, HIGH);          //Grupo B gira hacia adelante
  analogWrite(left_pwm, (255 - speeds));  // Se define una velocidad y se les pasa a los pines PWM
  digitalWrite(right_ctrl, HIGH);         //Grupo A gira hacia adelante
  analogWrite(right_pwm, (255 - speeds));
}
void car_back()  //El coche va hacia atras
{
  digitalWrite(left_ctrl, LOW);   //Grupo B gira hacia atras
  analogWrite(left_pwm, speeds);  // Se define una velocidad y se les pasa a los pines PWM
  digitalWrite(right_ctrl, LOW);  //Grupo A gira hacia atras
  analogWrite(right_pwm, speeds);
}
void car_left()  //El coche va hacia la izquierda
{
  digitalWrite(left_ctrl, LOW);  //Grupo B gira hacia atras
  analogWrite(left_pwm, speeds);
  digitalWrite(right_ctrl, HIGH);          //Grupo A gira hacia adelante
  analogWrite(right_pwm, (255 - speeds));  // Se mueve el derecho a mayor velocidad que el izquierdo
}
void car_right()  //El coche va hacia la derecha
{
  digitalWrite(left_ctrl, HIGH);          //Grupo B gira hacia adelante
  analogWrite(left_pwm, (255 - speeds));  // Se mueve el izquierdo a mayor velocidad que el derecho
  digitalWrite(right_ctrl, LOW);          //Grupo A gira hacia atras
  analogWrite(right_pwm, speeds);
}

void car_Stop()  //El coche se detiene
{
  digitalWrite(left_ctrl, LOW);  //Se le pasa el pulso LOW a los pines dcho e izdo
  analogWrite(left_pwm, 0);
  digitalWrite(right_ctrl, LOW);  //Grupo A gira hacia atras
  analogWrite(right_pwm, 0);      //No se establece ninguna velocidad para los pines PWM
}

void soft_left() {  //Movimiento suave a la hora de moverse en la linea
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, 0);  // Izquierdo parado
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, (255 - speeds));  // Derecho avanza
}

void soft_right() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, (255 - speeds));  // Izquierdo avanza
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, 0);  // Derecho parado
}

void tracking() {
  int track_flag = 1;
  agarrar();
  while (track_flag) {  //Variable que nos permite controlar el estado del seguimiento de linea
    L_val = digitalRead(L_pin);
    M_val = digitalRead(M_pin);
    R_val = digitalRead(R_pin);
    distance = get_distance();

    if (distance > 0 && distance <= 20) {  //Mientras que detecte un obstaculo entre 0cm y 20cm
      delay(30);
      int confirm_distance = get_distance();
      if (confirm_distance > 10 && confirm_distance <= 20) {  //Volvemos a confirmar si la distancia medida es correcta
        avoid();
        continue;  // Vuelve al inicio del bucle tras esquivar
      }
    } else {
      if (M_val == 1) {                         // Si detecta linea en el medio
        if (L_val == 1 && R_val == 0) {         //Detecta linea por la izquierda
          soft_left();                          // Giro suave a la izquierda
        } else if (L_val == 0 && R_val == 1) {  //Detecta linea por la derecha
          soft_right();                         // Giro suave a la derecha
        } else {                                // No encuentra nada a los lados
          car_front();                          // Continua hacia delante
        }
      } else {  //No encuentra nada en el medio
        if (L_val == 1 && R_val == 0) {
          soft_left();
        } else if (L_val == 0 && R_val == 1) {
          soft_right();
        } else {           //Fin de linea
          ciclo_fin();     //Ciclo de deposicion de carga
          track_flag = 0;  //Salimos del bucle
          break;           // Forzamos la salida del bucle
        }
      }
    }


    // Solo lee el Bluetooth si realmente hay datos
    if (Serial.available() > 0) {
      BLE_val = Serial.read();
      if (BLE_val == 'S') {  // Si recibe señal de paro desde bluetooth
        ciclo_fin();         //Ciclo de deposicion de carga
        track_flag = 0;      //Salimos del bucle
        break;               // Forzamos la salida del bucle
      }
    }
  }
}

int get_distance() {
  int distance = 0;
  digitalWrite(trigPin, LOW);  //Ponemos el pin en LOW para limpiar la lectura
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  //Realiza el disparo de ondas
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);  //Corta la señal

  // Añadimos 30000 microsegundos de espera
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    distance = 999;  // Si no hay eco (no hay obstáculo cerca), devolvemos un número alto

  } else {
    distance = duration / 58;  //Obtener centimetros a partir de los microsegundos
  }

  Serial.println(distance);
  return distance;
}

void avoid() {

  car_Stop();  //Para el coche
  matrix_display(clear);
  matrix_display(batisenal);
  delay(1000);

  cuello.write(180);  //Gira 180º
  delay(500);
  distance_l = get_distance();  //Mide la distancia a la izquierda
  delay(100);

  cuello.write(0);  //Gira a 0º
  delay(500);
  distance_r = get_distance();  //Mide la distancia a la derecha
  delay(100);

  if (distance_l > distance_r) {
    //Compara si la distancia izquierda es mayor a la derecha
    car_left();  //Gira a la izquierda
    matrix_display(clear);
    matrix_display(left);
    delay(200);
    car_Stop();
    matrix_display(clear);
    matrix_display(batisenal);

    //vv Comprobar si sigue habiendo obstaculo en la dcha
    cuello.write(0);  //Gira el cuello hasta los 0º
    delay(300);
    car_front();
    delay(200);
    while (get_distance() < 25) {  //Mientras que haya obstaculo...
      delay(50);
    }

    //Deja de haber obstaculo
    delay(300);
    car_Stop();

    cuello.write(90);  //Cuello al frente
    delay(300);

    car_right();  //Volvemos a la derecha
    delay(200);
    car_Stop();

    car_front();
    delay(600);
    car_Stop();

    car_right();
    delay(200);
    car_Stop();

    car_front();

    while (digitalRead(M_pin) == 0 && digitalRead(L_pin) == 0 && digitalRead(R_pin) == 0) {
      // Avanza hasta ver la línea negra
    }
    car_Stop();
    delay(100);

    car_left();  // Comienza a enderezarse

    while (digitalRead(M_pin) == 0) {
      // Sigue girando hasta que el sensor central pise la linea
    }
    car_Stop();  // Frena en la línea.

    // Limpia el puerto serie de comandos basura acumulados durante los delays
    while (Serial.available() > 0) {
      Serial.read();
    }
  } else {
    // Si hay más espacio a la derecha

    car_right();  //Gira a la derecha
    matrix_display(clear);
    matrix_display(left);
    delay(400);
    car_Stop();
    matrix_display(clear);
    matrix_display(batisenal);

    //Gira el cuello 180º
    cuello.write(180);
    delay(300);

    //vv Comprobar si sigue habiendo obstaculo en la izqda
    car_front();
    delay(200);
    while (get_distance() < 25) {  //Mientras que haya obstaculo...
    }

    //Ya no hay obstaculo

    delay(300);
    car_Stop();

    cuello.write(90);
    delay(300);

    car_left();
    delay(200);
    car_Stop();

    car_front();
    delay(600);
    car_Stop();

    car_left();
    delay(200);
    car_Stop();

    car_front();

    while (digitalRead(M_pin) == 0 && digitalRead(L_pin) == 0 && digitalRead(R_pin) == 0) {
      // Avanza hasta ver la linea negra
    }
    car_Stop();
    delay(100);

    car_right();  // Comienza a enderezarse

    while (digitalRead(M_pin) == 0) {
      // Sigue girando hasta que el sensor central pise la linea
    }
    car_Stop();  // Frena en la línea.

    // Limpia el puerto serie de comandos basura acumulados durante los delays
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}

void lower_arm_fwd() {                            //Inclina el brazo hacia delante
  while (lower_arm_position >= 5) {               //Limite de inclinacion a 5 grados
    lower_arm_position = lower_arm_position - 1;  //Resta un grado
    lower_arm.write(lower_arm_position);          // Actualiza la posicion
    delay(10);
  }
}

void lower_arm_bwd() {                            //Hace retroceder al brazo
  while (lower_arm_position <= 130) {             //Limite de retroceso a 130 grados
    lower_arm_position = lower_arm_position + 1;  //Suma un grado
    lower_arm.write(lower_arm_position);          //Actualiza la posicion
    delay(10);
  }
}

void open_claw() {                      //Abre la servopinza
  while (claw_position <= 110) {        //Limite de apertura a 110 grados
    claw_position = claw_position + 1;  //Suma un grado
    pinza.write(claw_position);         //Actualiza la posicion
    delay(10);
  }
}

void close_claw() {                     //Cierra la servopinza
  while (claw_position >= 0) {          //Cierra la pinza completamente
    claw_position = claw_position - 1;  //Resta un grado
    pinza.write(claw_position);         //Actualiza la posicion
    delay(10);
  }
}

/*
Funcion que ejecuta de manera secuencial la logica de la pinza para agarrar
objetos al iniciar el ciclo de seguimiento
*/
void agarrar() {
  close_claw();
  delay(300);
  lower_arm_fwd();
  delay(300);
  open_claw();
  delay(300);
  close_claw();
  delay(300);
  lower_arm_bwd();
  delay(300);
}

/*
Funcion que ejecuta de manera secuencial la logica de la pinza para soltar
objetos al acabar el ciclo de seguimiento
*/
void soltar() {
  lower_arm_fwd();
  delay(300);
  open_claw();
  delay(300);
  lower_arm_bwd();
  delay(300);
  close_claw();
  delay(300);
}

/*
Esta funcion nos permite maniobrar con el coche a la hora de soltar el objeto.
*/
void depositar() {
  car_left();
  delay(1200);
  car_Stop();
  delay(500);
  soltar();
  delay(300);
}

/*
Funcion que nos permite dar una vuelta de ~180º
*/
void dar_vuelta() {
  car_right();
  delay(1200);
  car_Stop();
  delay(300);
}

/*
Junta todas las funciones anteriores, para el coche, hace el ciclo de
deposicion, da la vuelta ~180º y rompe el bucle de seguimiento de linea
*/
void ciclo_fin() {
  car_Stop();
  delay(300);
  depositar();
  dar_vuelta();
}

void matrix_display(unsigned char matrix_value[]) {  //Funcion que nos permite dibujar imagenes por pantalla
  IIC_start();                                       //Inicia la transferencia de datos
  IIC_send(0xc0);                                    //Selecciona la direccion

  for (int i = 0; i < 16; i++)  //Los dibujos tienen un limite de 16 bytes
  {
    IIC_send(matrix_value[i]);  //Envia el dibujo
  }
  IIC_end();  //Corta la transmision
  IIC_start();
  IIC_send(0x8A);  //Dibuja el dibujo
  IIC_end();
}

void IIC_start() {
  digitalWrite(SDA_Pin, HIGH);  //Ambos pines en HIGH, la comunicacion esta en reposo
  digitalWrite(SCL_Pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, LOW);  //Se inicia la transmision
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, LOW);  //Se inicia el conteo del reloj
}
//Indicates the end of data transmission
void IIC_end() {
  digitalWrite(SCL_Pin, LOW);  //Ambos pines funcionando
  digitalWrite(SDA_Pin, LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, HIGH);  //Subimos SCL
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, HIGH);  //Si SDA sube de LOW a HIGH mientras que SCL esta en HIGH, se corta la comunicacion
  delayMicroseconds(3);
}
//transmit data
void IIC_send(unsigned char send_data) {
  for (byte mask = 0x01; mask != 0; mask <<= 1)  //Cada byte tiene 8 bits, que se comprueban de uno en uno
  {
    if (send_data & mask) {  //Define los niveles HIGH o LOW de SDA_Pin dependiendo si cada bit del byte es un 1 o un 0
      digitalWrite(SDA_Pin, HIGH);
    } else {
      digitalWrite(SDA_Pin, LOW);
    }
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, HIGH);  //Subir SCL_Pin a HIGH para cortar
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, LOW);  //Bajar SCL_Pin a LOW para cambiar la señal del SDA
  }
}
