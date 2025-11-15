// Control BLDC con Arduino Pro Micro
// Giro en sentido directo y giro sentido inverso

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

// Giro
volatile int PWM;                           // Para controlar el PWM del motor (lo usa loop e ISR)
volatile unsigned long lastChangeTime = 0;  // Para verificar cuánto tiempo ha pasado desde el último cambio de Hall
const unsigned long timeout = 200;          // Tiempo que esperamos antes de forzar el siguiente paso (200 ms)
const unsigned long cambio = 2000;          // Tiempo de giro en cada sentido (5 s)

// Secuencia de conmutación (NO TOCADA)
int secuenciaD[6] = { 0b110, 0b100, 0b101, 0b001, 0b011, 0b010 };
int indiceSecuenciaD = 0;

int secuenciaI[6] = { 0b010, 0b011, 0b001, 0b101, 0b100, 0b110 };
int indiceSecuenciaI = 0;

// Variables de control
volatile int D = 1;  // 1 = directo, 2 = inverso, 0 = parado (solo detectar Halls)


void setup() {

  // Sensores Hall como entrada con pullup interno
  pinMode(H0_PIN, INPUT_PULLUP);
  pinMode(H1_PIN, INPUT_PULLUP);
  pinMode(H2_PIN, INPUT_PULLUP);

  // Salidas de potencia
  pinMode(GH, OUTPUT);
  pinMode(GL, OUTPUT);
  pinMode(BH, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(YH, OUTPUT);
  pinMode(YL, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  // Interrupciones (cada vez que cambie cualquier hall)
  attachInterrupt(digitalPinToInterrupt(H0_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H1_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), ISR_Halls, CHANGE);

  // Ajuste de frecuencia PWM alta (~31 kHz aprox)
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // Timer1 -> pin 9,10
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  // Timer3 -> pin 5
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  // Timer4 -> pin 6,13

  lastChangeTime = millis();
}

void loop() {

  // --------- 1) GIRO EN SENTIDO DIRECTO DURANTE 5 s ---------

    int lectura = analogRead(POT);

    // lo paso a PWM limitado al 80%
    // zona muerta al 5%
    if (lectura < 51) {
      PWM = 0;
    } else {
      PWM = map(lectura, 51, 1023, 0, 204);
    }

    if (PWM > 0) {
      unsigned long ahora = millis();
      // Si el estado de los Hall no ha cambiado en 'timeout' ms, forzamos siguiente paso
      if (ahora - lastChangeTime >= timeout) {
        int hallForzado = secuenciaD[indiceSecuenciaD];
        giroSentidoDirecto(hallForzado);

        indiceSecuenciaD++;
        if (indiceSecuenciaD >= 6) {
          indiceSecuenciaD = 0;  // Volvemos al inicio de la secuencia
        }
        lastChangeTime = ahora;
      }
    }
  

  // --------- 2) ESPERAR A QUE EL MOTOR SE PARE ---------
  //
  D = 0;
  delay(10000);
  digitalWrite(20, HIGH);
  digitalWrite(21, HIGH);
  // Esperamos hasta que los Halls no hayan cambiado en 'timeout' ms
  while (millis() - lastChangeTime <= timeout) {
  }

  // --------- 3) GIRO EN SENTIDO INVERSO DURANTE 5 s ---------
  D = 2;
  inicio = millis();

  while (millis() - inicio <= cambio) {

    // leo el potenciómetro
    int lectura = analogRead(POT);

    // lo paso a PWM limitado al 80%
    // zona muerta al 5%
    if (lectura < 51) {
      PWM = 0;
    } else {
      PWM = map(lectura, 51, 1023, 0, 204);
    }

    if (PWM > 0) {
      unsigned long ahora = millis();
      // Si el estado de los Hall no ha cambiado en 'timeout' ms, forzamos siguiente paso
      if (ahora - lastChangeTime >= timeout) {
        int hallForzado = secuenciaI[indiceSecuenciaI];
        giroSentidoInverso(hallForzado);

        indiceSecuenciaI++;
        if (indiceSecuenciaI >= 6) {
          indiceSecuenciaI = 0;  // Volvemos al inicio de la secuencia
        }
        lastChangeTime = ahora;  // Reiniciamos el temporizador
      }
    }
  }

  // --------- 4) ESPERAR A QUE EL MOTOR SE PARE ---------
  D = 0;

  while (millis() - lastChangeTime <= timeout) {
    // Espera activa hasta que no haya cambios en los Halls durante 'timeout'
  }

  // Y el loop() vuelve a empezar: otra vez directo, parada, inverso, parada, ...
}


// ================== ISR DE LOS HALLS ==================

void ISR_Halls() {
  // Leo los valores de los sensores Hall
  int ValDIO0 = digitalRead(H0_PIN);  // Leo el hall0
  int ValDIO1 = digitalRead(H1_PIN);  // Leo el hall1
  int ValDIO2 = digitalRead(H2_PIN);  // Leo el hall2

  int hallState = (ValDIO0 << 2) | (ValDIO1 << 1) | ValDIO2;

  if (D == 1) {
    giroSentidoDirecto(hallState);
  }
  if (D == 2) {
    giroSentidoInverso(hallState);
  }

  // Siempre que haya un cambio de Hall, actualizamos lastChangeTime
  lastChangeTime = millis();
}

// ------------------------------------------------------------------
// GIRO DIRECTO (NO TOCADO)

void giroSentidoDirecto(int hallState) {

  switch (hallState) {
    case 0b000:  // nunca vamos a tener los halls en 000 y por tanto no se va a dar
      break;

    case 0b001:
      // GL = 1, YH = 1
      digitalWrite(GH, LOW);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);

      digitalWrite(GL, LOW);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b010:
      // BH = 1, YL = 1
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, LOW);
      break;

    case 0b011:
      // GL = 1, BH = 1
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);

      digitalWrite(GL, LOW);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);

      break;

    case 0b100:
      // GH = 1, BL = 1
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, LOW);
      digitalWrite(YL, HIGH);
      break;

    case 0b101:
      // BL = 1, YH = 1
      digitalWrite(GH, LOW);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, LOW);
      digitalWrite(YL, HIGH);
      break;

    case 0b110:
      // GH = 1, YL = 1
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, LOW);
      break;

    case 0b111:  // nunca vamos a tener los halls en 111 no van a estar todos encendidos a la vez
      break;
  }
}

// ------------------------------------------------------------------
// GIRO INVERSO (NO TOCADO)

void giroSentidoInverso(int hallState) {

  switch (hallState) {
    case 0b000:  // nunca vamos a tener los halls en 000
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
      // BL = 1, YH = 1
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
      // GL = 1, BH = 1
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
      // GL = 1, YH = 1
      digitalWrite(GH, LOW);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);

      digitalWrite(GL, LOW);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b111:  // nunca vamos a tener los halls en 111
      break;
  }
}
