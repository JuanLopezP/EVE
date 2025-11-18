// Control BLDC con Arduino Pro Micro
// Giro en sentido inverso

// Pines de los transistores
const int GH = 5;  // GREEN alto  (PWM)  -> MOSFET alto fase verde
const int GL = 4;  // GREEN bajo  (DIG)  -> MOSFET bajo fase verde

const int BH = 6;  // BLUE  alto  (PWM)  -> MOSFET alto fase azul
const int BL = 8;  // BLUE  bajo  (DIG)  -> MOSFET bajo fase azul

const int YH = 9;   // YELLOW alto (PWM)  -> MOSFET alto fase amarilla
const int YL = 10;  // YELLOW bajo (DIG)  -> MOSFET bajo fase amarilla


// Potenciómetro --> acelerador
const int POT = A0;

// Pines de los sensores Hall con interrupciones
const int H0_PIN = 2;  // Hall 0
const int H1_PIN = 3;  // Hall 1
const int H2_PIN = 7;  // Hall 2

// Variables globales
int PWM;                            // Para controlar el PWM del motor
//int lastHallState = -1;             // Para guardar el último estado de los Hall
unsigned long lastChangeTime = 0;   // Para verificar cuánto tiempo ha pasado
const unsigned long timeout = 200;  // Tiempo que esperamos antes de forzar el siguiente paso (200 ms)

// Secuencia de conmutación
int secuencia[6] = { 0b010, 0b011, 0b001, 0b101, 0b100, 0b110 }; // Secuencia obtenida al girar el motor en sentido directo
int indiceSecuencia = 0; //Inidce que nos permitirá recorrer la secuencia en orden en el loop

//------------------------------------------------------------------------------
// DECLARAMOS LOS PINES QUE VAMOS A USAR PARA CADA TAREA

void setup() {

  // Sensores Hall como entrada con pullup interno
  pinMode(H0_PIN, INPUT_PULLUP);
  pinMode(H1_PIN, INPUT_PULLUP);
  pinMode(H2_PIN, INPUT_PULLUP);

  // Salidas de potencia para el control del motor 
  pinMode(GH, OUTPUT);
  pinMode(GL, OUTPUT);
  pinMode(BH, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(YH, OUTPUT);
  pinMode(YL, OUTPUT);


  // Declaración de las interrupciones (cada vez que cambie cualquier hall)
  attachInterrupt(digitalPinToInterrupt(H0_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H1_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), ISR_Halls, CHANGE);


  // Ajuste de frecuencia PWM alta (~31 kHz aprox)
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // Timer1 -> pin 9,10
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  // Timer3 -> pin 5
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  // Timer4 -> pin 6,13
}

//------------------------------------------------------------------
// "OBLIGAMOS" A REALIZAR LA SECUENCIA AL MOTOR
void loop() {

  // leo el potenciómetro
  PWM = analogRead(POT);

  // Limitamos la zona de funcionamiento del motor en función de lo que reciba en el potenciómetro.
  // Marcamos el máximo al 80% y la zona muerta si el resultado del potenciometro es menor que 5%
  if (PWM < 51) {
    PWM = 0;
  } else {
    PWM = map(PWM, 51, 1023, 0, 204);
  }


  // Tratamos de arrancar el motor forzando la secuencia de arranque para coseguir el movimiento y
  // lograr que las interrupciones tomen el control.
  // Para lograr esto hemos declarado una variable que se encarga de guardar el tiempo en el que los sensores
  // cambiaron de valor por última vez.

 unsigned long ahora = millis();  // Leo el tiempo actual

  if (PWM > 0) {  // Siempre que el acelerador este activo
    if (ahora - lastChangeTime >= timeout) {  // resto este instante con la última vez que se activaron los sensores y si lleva
    // más de 200 ms parados entonces inicio mi secuencia para tratar de arrancarlo
      // Forzamos el siguiente paso de la secuencia
      int hallForzado = secuencia[indiceSecuencia];
      giroSentidoInverso(hallForzado); // Obligamos a que el rotor tenga esa secuencia

     
      indiceSecuencia++;  // Avanzamos al siguiente paso de la secuencia
      if (indiceSecuencia >= 6) {
        indiceSecuencia = 0;  // Si llegamos al final de la secuencia, volvemos al inicio
      }
      lastChangeTime = ahora;  // Reiniciamos el temporizador
    }

  } 
}

//  ---------------------------------------------------------------
// INTERRUPCIONES

void ISR_Halls() {
  //Leo los valores de los sensores Hall
  int ValDIO0 = digitalRead(H0_PIN);  //Leo el hall0
  int ValDIO1 = digitalRead(H1_PIN);  //Leo el hall1
  int ValDIO2 = digitalRead(H2_PIN);  //Leo el hall2

  int hallState = (ValDIO0 << 2) | (ValDIO1 << 1) | ValDIO2; // asigno mi máscara para poder llamar a
  // los sensores de efecto hall de forma mas clara y sencilla
  giroSentidoInverso(hallState); // Mando el resultado para continuar el funcionamiento del motor
  lastChangeTime = millis(); // Actualizo mi tiempo de forma que guardo la ultima vez que se movieron los sensores
  // Si no para de moverse se actualizará todo el rato, si no simplemente se quedará con la última. 
}

// ------------------------------------------------------------------
// IMPLEMENTACION DE LA TABLA DE VERDAD 
// Implementamos en cada case una de las configuraciones de la tabla de verdad
void giroSentidoInverso(int hallState) {
  switch (hallState) {
    case 0b000:  //nunca vamos a tener los halls en 000
      break;

    case 0b001:
      // GH = 1, YL = 1
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, LOW);
      break;

    case 0b010:
      // YH = 1, BL = 1
      digitalWrite(GH, LOW);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, LOW);
      digitalWrite(YL, HIGH);
      break;

    case 0b011:
      // GH = 1, BL = 1
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, LOW);
      digitalWrite(YL, HIGH);
      break;

    case 0b100:
      // BH = 1, GL = 1
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);

      digitalWrite(GL, LOW);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b101:
      // BH = 1, YL = 1
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, LOW);
      break;

    case 0b110:
      // YH = 1, GL = 1 
      digitalWrite(GH, LOW);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);

      digitalWrite(GL, LOW);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b111:  //nunca vamos a tener los halls en 111
      break;
  }
}
