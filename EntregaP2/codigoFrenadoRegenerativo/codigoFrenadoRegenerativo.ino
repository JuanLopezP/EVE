// ESTE CÓDIGO NO SE HA PROBADO EN LABORATORIO POR TANTO PUEDE CONTENER ERRORES

// Control BLDC con Arduino Pro Micro
// Giro en sentido directo, inverso y frenado regenerativo 

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

// Pin de selección de sentido (D20 o D21)
const int DIR_PIN = 20;   // Sentido de giro (HIGH = directo, LOW = inverso)
// Pin de habilitación de frenado regenerativo
const int REGEN_PIN = 21; // Habilita frenado regenerativo cuando está en HIGH

// Constantes de sentido
const int SENTIDO_DIRECTO = 0;
const int SENTIDO_INVERSO = 1;

// Límites de PWM
const int PWM_MAX_DIRECTO = 204;  // ~80% de 255
const int PWM_MAX_INVERSO = 51;   // ~20% de 255
const int PWM_REGEN       = 77;   // ~30% de 255

// Variables globales
int PWM;                          // Para controlar el PWM del motor
unsigned long lastChangeTime = 0; // Momento del último cambio de los Hall
const unsigned long timeout = 200;  // Tiempo para forzar siguiente paso en tracción (200 ms)

// Tiempo para considerar que el motor se ha parado en modo regenerativo
const unsigned long tiempoParadaRegen = 600;  // ms aprox

// Sentidos
int sentidoSeleccionado = SENTIDO_DIRECTO;
int sentidoActual       = SENTIDO_DIRECTO;  // Sentido con el que está conmutando el puente

// Regenerativo
bool regenHabilitado = false;            // Estado de la entrada REGEN_PIN
bool enRegen         = false;            // Indica si estamos frenando regenerativamente
int sentidoPendiente = SENTIDO_DIRECTO;  // Sentido al que cambiaremos después de frenar

// Secuencias de conmutación en tracción
// Directo
int secuenciaDirecta[6] = { 0b110, 0b100, 0b101, 0b001, 0b011, 0b010 };
// Inverso
int secuenciaInversa[6] = { 0b010, 0b011, 0b001, 0b101, 0b100, 0b110 };
int indiceSecuencia = 0;

// Función auxiliar: comprueba si el motor sigue girando
bool motorGirando(unsigned long ahora) {
  // Considero que sigue girando si ha habido cambios de Hall hace poco
  return (ahora - lastChangeTime) < tiempoParadaRegen;
}

//------------------------------------------------------------------------------

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

  // Entradas de selección de sentido y regenerativo
  pinMode(DIR_PIN, INPUT_PULLUP);    // Asumo interruptor a GND
  pinMode(REGEN_PIN, INPUT_PULLUP);  // Asumo interruptor a GND

  // Interrupciones (cada vez que cambie cualquier sensor de efecto Hall)
  attachInterrupt(digitalPinToInterrupt(H0_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H1_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), ISR_Halls, CHANGE);

  // Ajuste de frecuencia PWM alta (~31 kHz aprox)
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // Timer1 -> pin 9,10
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  // Timer3 -> pin 5
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  // Timer4 -> pin 6,13
}

//------------------------------------------------------------------

void loop() {
  unsigned long ahora = millis();  // Leo el tiempo actual

  // 1) Leo el sentido solicitado
  int dirPinValue = digitalRead(DIR_PIN);
  // HIGH = directo, LOW = inverso
  if (dirPinValue == HIGH) {
    sentidoSeleccionado = SENTIDO_DIRECTO;
  } else {
    sentidoSeleccionado = SENTIDO_INVERSO;
  }

  // 2) Leo si el frenado regenerativo está habilitado
  int regenPinValue = digitalRead(REGEN_PIN);
  // HIGH = regenerativo habilitado, LOW = deshabilitado
  regenHabilitado = (regenPinValue == HIGH);

  // 3) Leo el potenciómetro
  int potValue = analogRead(POT);
  bool aceleradorActivo = (potValue >= 51);  // Por encima del 5%

  // 4) Cálculo del PWM en función del modo actual
  if (!enRegen) {
    // Modo tracción (normal)
    if (!aceleradorActivo) {
      PWM = 0;
    } else {
      if (sentidoActual == SENTIDO_DIRECTO) {
        PWM = map(potValue, 51, 1023, 0, PWM_MAX_DIRECTO);
      } else {
        PWM = map(potValue, 51, 1023, 0, PWM_MAX_INVERSO);
      }
    }
  } else {
    // Modo frenado regenerativo: PWM fijo al 30%
    PWM = PWM_REGEN;
  }

  // 5) Activar frenado regenerativo al soltar el acelerador (solo en directo)
  if (regenHabilitado && !enRegen && sentidoActual == SENTIDO_DIRECTO) {
    if (!aceleradorActivo && motorGirando(ahora)) {
      // Entramos en modo regenerativo manteniendo el mismo sentido
      enRegen = true;
      sentidoPendiente = sentidoActual;  // No hay cambio de sentido pendiente
      PWM = PWM_REGEN;
    }
  }

  // 6) Gestión del cambio de sentido
  if (sentidoSeleccionado != sentidoActual) {

    if (regenHabilitado && !enRegen && sentidoActual == SENTIDO_DIRECTO) {
      // Se ha pedido cambio de sentido y hay regenerativo:
      // primero frenamos regenerando en sentido directo
      enRegen = true;
      sentidoPendiente = sentidoSeleccionado;  // Sentido al que queremos ir después
      PWM = PWM_REGEN;
    } else {
      // Sin regenerativo (o venimos de sentido inverso): parada "normal"
      if (PWM == 0) {
        apagarPuente();
        sentidoActual = sentidoSeleccionado;
        indiceSecuencia = 0;
        lastChangeTime = ahora;
      }
    }
  }

  // 7) Salir del modo regenerativo cuando el motor ya se ha parado
  if (enRegen) {
    if (!motorGirando(ahora)) {
      // Considero que el motor ya está parado
      enRegen = false;
      PWM = 0;
      apagarPuente();

      // Si había un cambio de sentido pendiente, lo aplico ahora
      if (sentidoPendiente != sentidoActual) {
        sentidoActual = sentidoPendiente;
        indiceSecuencia = 0;
        lastChangeTime = ahora;
      }
    }
  }

  // 8) Arranque / mantenimiento si los Halls no cambian (solo en tracción)
  if (PWM > 0 && !enRegen) {
    if (ahora - lastChangeTime >= timeout) {
      int hallForzado;

      if (sentidoActual == SENTIDO_DIRECTO) {
        hallForzado = secuenciaDirecta[indiceSecuencia];
        giroSentidoDirecto(hallForzado);
      } else {
        hallForzado = secuenciaInversa[indiceSecuencia];
        giroSentidoInverso(hallForzado);
      }

      // Avanzo al siguiente paso de la secuencia
      indiceSecuencia++;
      if (indiceSecuencia >= 6) {
        indiceSecuencia = 0;  // Si llego al final de la secuencia, vuelvo al inicio
      }
      lastChangeTime = ahora;  // Reinicio el temporizador
    }
  }
}

//  ---------------------------------------------------------------
// Interrupciones de los sensores Hall

void ISR_Halls() {
  // Leo los valores de los sensores Hall
  int ValDIO0 = digitalRead(H0_PIN);  // Leo el Hall0
  int ValDIO1 = digitalRead(H1_PIN);  // Leo el Hall1
  int ValDIO2 = digitalRead(H2_PIN);  // Leo el Hall2

  int hallState = (ValDIO0 << 2) | (ValDIO1 << 1) | ValDIO2;  // Máscara: D0 D1 D2

  if (PWM > 0) {
    if (enRegen && sentidoActual == SENTIDO_DIRECTO) {
      // Frenado regenerativo en sentido directo
      giroRegenDirecto(hallState);
    } else {
      // Tracción normal
      if (sentidoActual == SENTIDO_DIRECTO) {
        giroSentidoDirecto(hallState);
      } else {
        giroSentidoInverso(hallState);
      }
    }
  } else {
    // Sin acelerador ni regen --> puente apagado
    apagarPuente();
  }

  lastChangeTime = millis();
}

// ------------------------------------------------------------------
// Función para apagar todos los MOSFET (seguridad en cambios de sentido)

void apagarPuente() {
  analogWrite(GH, LOW);
  analogWrite(BH, LOW);
  analogWrite(YH, LOW);

  // MOSFET bajos: HIGH = apagado (según driver)
  digitalWrite(GL, HIGH);
  digitalWrite(BL, HIGH);
  digitalWrite(YL, HIGH);
}

// ------------------------------------------------------------------
// Tracción en sentido DIRECTO

void giroSentidoDirecto(int hallState) {

  switch (hallState) {
    case 0b000:  // Nunca vamos a tener los Halls en 000
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

    case 0b111:  // Nunca vamos a tener los Halls en 111
      break;
  }
}

// ------------------------------------------------------------------
// Tracción en sentido INVERSO

void giroSentidoInverso(int hallState) {

  switch (hallState) {
    case 0b000:  // Nunca vamos a tener los Halls en 000
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

    case 0b111:  // Nunca vamos a tener los Halls en 111
      break;
  }
}

// ------------------------------------------------------------------
// Frenado regenerativo en sentido DIRECTO 

void giroRegenDirecto(int hallState) {

  switch (hallState) {

    
    case 0b000: // Nunca vamos a tener los Halls en 000
    case 0b111: // Nunca vamos a tener los Halls en 111
      apagarPuente();
      break;

    case 0b001:
      // GH = 1, YH = 1
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b010:
      // BH = 1, YH = 1
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      analogWrite(YH, PWM);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b011:
      // GH = 1, BH = 1
      analogWrite(GH, PWM);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b100:
      // GH = 1, BH = 1 
      analogWrite(GH, PWM);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b101:
      // BH = 1, YH = 1
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      analogWrite(YH, PWM);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b110:
      // GH = 1, YH = 1
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);

      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;
  }
}
