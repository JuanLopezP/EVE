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

// Pin de selección de sentido (conector J15, D20)
const int DIR_PIN = 20;  // HIGH = marcha adelante, LOW = marcha atrás

// Giro / control PWM
volatile int PWM = 0;                        // Para controlar el PWM del motor (lo usa loop e ISR)
volatile unsigned long lastChangeTime = 0;   // Tiempo desde el último cambio de Hall
const unsigned long timeout = 200;           // Tiempo para forzar el siguiente paso (ms)

// Tiempo para considerar que el motor se ha parado (sin cambios de Hall)
const unsigned long stopTimeout = 1000;      // ms sin cambios de Hall => motor parado

// Secuencia de conmutación (NO TOCADA)
int secuenciaD[6] = { 0b110, 0b100, 0b101, 0b001, 0b011, 0b010 };
int indiceSecuenciaD = 0;

int secuenciaI[6] = { 0b010, 0b011, 0b001, 0b101, 0b100, 0b110 };
int indiceSecuenciaI = 0;

// Constantes de sentido
const int SENTIDO_PARADO    = 0;
const int SENTIDO_ADELANTE  = 1;
const int SENTIDO_ATRAS     = 2;

// Variables de control de sentido
volatile int D = SENTIDO_PARADO;  // Sentido actual que usa la ISR
int dirCommand      = SENTIDO_ADELANTE;  // Sentido ordenado por el pin DIR_PIN
int lastDirCommand  = SENTIDO_ADELANTE;  // Para detectar cambios de orden
bool esperandoParada = false;            // true cuando hemos ordenado cambio de sentido y esperamos a que se pare


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

  // Selector de sentido
  pinMode(DIR_PIN, INPUT_PULLUP);  // Interruptor o jumper hacia GND = marcha atrás

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
  unsigned long ahora = millis();

  // 1) Leer selector de sentido (HIGH = adelante, LOW = atrás)
  if (digitalRead(DIR_PIN) == HIGH) {
    dirCommand = SENTIDO_ADELANTE;
  } else {
    dirCommand = SENTIDO_ATRAS;
  }

  // 2) Leer potenciómetro
  int lectura = analogRead(POT);

  // 3) Calcular PWM deseado según sentido ORDENADO (no el actual)
  int pwmMax;
  if (dirCommand == SENTIDO_ATRAS) {
    // Máx. 20% en marcha atrás (255 * 0.2 ≈ 51)
    pwmMax = 51;
  } else {
    // Adelante como antes: máx ~80% (255 * 0.8 ≈ 204)
    pwmMax = 204;
  }

  int pwmDeseado = 0;
  // Zona muerta al 5% del potenciómetro (51 ≈ 0.05 * 1023)
  if (lectura >= 51) {
    pwmDeseado = map(lectura, 51, 1023, 0, pwmMax);
  }

  // 4) Gestión de cambio de sentido
  // Si la orden de sentido cambia mientras el motor está siendo alimentado, hay que pararlo totalmente
  if (dirCommand != lastDirCommand) {
    // Ha cambiado la orden de sentido
    if (D != SENTIDO_PARADO) {
      // El motor estaba siendo conmutado: paramos y esperamos a que se detenga
      D = SENTIDO_PARADO;   // La ISR deja de conmutar (solo lee Halls)
      PWM = 0;              // Quitamos par motor (coasting/freno según hardware)
      esperandoParada = true;
      // lastChangeTime se seguirá actualizando mientras los Halls se muevan
    } else {
      // Ya está parado: podemos cambiar directamente de sentido
      D = dirCommand;
      esperandoParada = false;
    }
    lastDirCommand = dirCommand;
  }

  // 5) Si estamos esperando a que se pare (después de ordenar un cambio de sentido)
  if (esperandoParada) {
    // Nos aseguramos de que no haya par aplicado
    PWM = 0;

    // Consideramos que el motor está parado cuando no hay cambios de Hall
    // durante stopTimeout ms
    if (ahora - lastChangeTime >= stopTimeout) {
      // Motor parado -> activamos el nuevo sentido
      D = dirCommand;
      esperandoParada = false;
    }
  } else {
    // No estamos en fase de parada: aplicamos el PWM calculado
    PWM = pwmDeseado;
  }

  // 6) Arranque / avance forzado si no hay cambios de Hall
  // (solo si hay PWM > 0 y un sentido activo)
  if (PWM > 0 && (D == SENTIDO_ADELANTE || D == SENTIDO_ATRAS)) {
    if (ahora - lastChangeTime >= timeout) {
      if (D == SENTIDO_ADELANTE) {
        int hallForzado = secuenciaD[indiceSecuenciaD];
        giroSentidoDirecto(hallForzado);
        indiceSecuenciaD++;
        if (indiceSecuenciaD >= 6) indiceSecuenciaD = 0;
      } else if (D == SENTIDO_ATRAS) {
        int hallForzado = secuenciaI[indiceSecuenciaI];
        giroSentidoInverso(hallForzado);
        indiceSecuenciaI++;
        if (indiceSecuenciaI >= 6) indiceSecuenciaI = 0;
      }
      lastChangeTime = ahora;
    }
  }

  // No hay while ni delay: todo es no bloqueante.
}



// ================== ISR DE LOS HALLS ==================

void ISR_Halls() {
  // Leo los valores de los sensores Hall
  int ValDIO0 = digitalRead(H0_PIN);  // Leo el hall0
  int ValDIO1 = digitalRead(H1_PIN);  // Leo el hall1
  int ValDIO2 = digitalRead(H2_PIN);  // Leo el hall2

  int hallState = (ValDIO0 << 2) | (ValDIO1 << 1) | ValDIO2;

  if (D == SENTIDO_ADELANTE) {
    giroSentidoDirecto(hallState);
  } else if (D == SENTIDO_ATRAS) {
    giroSentidoInverso(hallState);
  }
  // Si D == SENTIDO_PARADO, no conmutamos (motor libre / freno según hardware)

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
