// Control BLDC con Arduino Pro Micro
// Giro en sentido directo

// Pines de los transistores
const int GH = 5;
const int GL = 4;
const int BH = 6;
const int BL = 8;
const int YH = 9;
const int YL = 10;

// Potenciómetro --> acelerador
const int POT = A0;

// Pines de los sensores Hall con interrupciones
const int H0_PIN = 2;
const int H1_PIN = 3;
const int H2_PIN = 7;

int PWM;  // Variable para controlar el PWM del motor

// -------------------- ESTADOS --------------------
enum EstadoMotor {
  Estado_Aling,
  Estado_Ramp,
  Estado_Run
};

EstadoMotor estado = Estado_Aling;
unsigned long tiempoAnterior = 0;

// Secuencia de conmutación que entiende tu switch (la típica)
int secuencia[6] = { 0b110, 0b100, 0b101, 0b001, 0b011, 0b010 };

int indiceSecuencia = 0;
int pasosHechos = 0;

// Cuántos pasos queremos dar en la rampa de arranque (24 = 4 vueltas)
const int pasosParaArrancar = 24;   

// Para guardar el hall que había justo al arrancar
int hallInicial = 0;

// *** AÑADIDO *** Variable de control para saber si el motor ya está girando
bool motorEnMarcha = false;

// *** AÑADIDO *** Función auxiliar para leer los halls reales
int leerHallActual() {
  int h0 = digitalRead(H0_PIN);
  int h1 = digitalRead(H1_PIN);
  int h2 = digitalRead(H2_PIN);
  return (h0 << 2) | (h1 << 1) | h2;
}

// *** AÑADIDO *** Función para buscar el índice de la secuencia
int buscarIndicePorHall(int hall) {
  for (int i = 0; i < 6; i++) {
    if (secuencia[i] == hall) {
      return i;
    }
  }
  return 0;  // Por si llega algo raro
}

void setup() {
  pinMode(H0_PIN, INPUT_PULLUP);
  pinMode(H1_PIN, INPUT_PULLUP);
  pinMode(H2_PIN, INPUT_PULLUP);

  pinMode(GH, OUTPUT);
  pinMode(GL, OUTPUT);
  pinMode(BH, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(YH, OUTPUT);
  pinMode(YL, OUTPUT);

  // Interrupciones (cada vez que cambie cualquier sensor de efecto hall)
  attachInterrupt(digitalPinToInterrupt(H0_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H1_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), ISR_Halls, CHANGE);

  // Ajuste de frecuencia PWM alta (~31 kHz aprox)
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // Timer1 -> pin 9,10
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  // Timer3 -> pin 5
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  // Timer4 -> pin 6,13

  leerHalls();   // Alineamos al principio
}

void loop() {

  // Leo el potenciómetro
  PWM = analogRead(POT);

  // Zona muerta y limitación
  if (PWM < 51) {
    PWM = 0;
  } else {
    PWM = map(PWM, 51, 1023, 0, 204);
  }

  // *** AÑADIDO: Aseguramos que PWM sea suficientemente alto en el arranque ***
  if (PWM < 140) {
    PWM = 140;  // Establecer un mínimo para asegurar que el motor tenga suficiente par
  }

  unsigned long ahora = millis();

  if (!motorEnMarcha) {
    // Si el motor no está en marcha, forzamos la secuencia de arranque
    switch (estado) {

      case Estado_Aling:
        if (ahora - tiempoAnterior >= 120) {  // Damos tiempo para alineado
          indiceSecuencia = buscarIndicePorHall(hallInicial);
          pasosHechos = 0;
          tiempoAnterior = ahora;
          estado = Estado_Ramp;
        }
        break;

      case Estado_Ramp:
        // Rampa más lenta para permitir que el motor siga el campo
        if (ahora - tiempoAnterior >= 6) {  // 6 ms entre pasos
          tiempoAnterior = ahora;

          int hallForzado = secuencia[indiceSecuencia];

          // *** AÑADIDO: Empujón más fuerte durante la rampa ***
          int pwmOriginal = PWM;
          if (PWM < 160) PWM = 160;  // Aumentamos el PWM mínimo
          giroSentidoDirecto(hallForzado);
          PWM = pwmOriginal;
          // *** FIN AÑADIDO ***

          indiceSecuencia++;
          if (indiceSecuencia >= 6) {
            indiceSecuencia = 0;
          }

          pasosHechos++;
          if (pasosHechos >= pasosParaArrancar) {
            // *** AÑADIDO: Verificamos que el rotor ya ha comenzado a moverse
            int hallReal = leerHallActual();
            giroSentidoDirecto(hallReal);
            motorEnMarcha = true;  // El motor ya está girando
            estado = Estado_Run;
          }
        }
        break;
    }
  }

  // En Estado_Run, ya dejamos que las interrupciones controlen el motor
  // Si el motor ya está en marcha, no forzamos nada más.
}

// -------------------------------------------------------------------
// leer posición inicial y alinear
void leerHalls() {
  hallInicial = leerHallActual();

  // *** AÑADIDO: Alineamos el motor al inicio con el hall inicial ***
  int pwmOriginal = PWM;
  if (pwmOriginal < 140) PWM = 140;  // Aseguramos un mínimo PWM
  giroSentidoDirecto(hallInicial);
  PWM = pwmOriginal;

  tiempoAnterior = millis();
}

// -------------------------------------------------------------
// Interrupciones:
void ISR_Halls() {
  int ValDIO0 = digitalRead(H0_PIN);
  int ValDIO1 = digitalRead(H1_PIN);
  int ValDIO2 = digitalRead(H2_PIN);

  int hallState = (ValDIO0 << 2) | (ValDIO1 << 1) | ValDIO2;

  // Solo actuamos cuando estamos en Estado_Run
  if (motorEnMarcha) {  // Solo actuamos cuando el motor ya está en marcha
    giroSentidoDirecto(hallState);
  }
}

// ------------------------------------------------------------------
void giroSentidoDirecto(int hallState) {

  switch (hallState) {
    case 0b001:
      digitalWrite(GH, LOW);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);
      digitalWrite(GL, LOW);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b010:
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);
      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, LOW);
      break;

    case 0b011:
      digitalWrite(GH, LOW);
      analogWrite(BH, PWM);
      digitalWrite(YH, LOW);
      digitalWrite(GL, LOW);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, HIGH);
      break;

    case 0b100:
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      digitalWrite(YH, LOW);
      digitalWrite(GL, HIGH);
      digitalWrite(BL, LOW);
      digitalWrite(YL, HIGH);
      break;

    case 0b101:
      digitalWrite(GH, LOW);
      digitalWrite(BH, LOW);
      analogWrite(YH, PWM);
      digitalWrite(GL, HIGH);
      digitalWrite(BL, LOW);
      digitalWrite(YL, HIGH);
      break;

    case 0b110:
      analogWrite(GH, PWM);
      digitalWrite(BH, LOW);
      digitalWrite(YH, LOW);
      digitalWrite(GL, HIGH);
      digitalWrite(BL, HIGH);
      digitalWrite(YL, LOW);
      break;

    default:
      break;
  }
}
