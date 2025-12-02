// ESTE CÓDIGO NO SE HA PROBADO EN LABORATORIO Y POR TANTO PUEDE CONTENER ERRORES

// Control BLDC con Arduino Pro Micro
// Giro en sentido directo e inverso con arranque suave

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

// Creo una nueva entrada digital para
// HIGH -> marcha adelante (sentido directo)
// LOW  -> marcha atrás  (sentido inverso)
const int DIR_PIN = 20;   // puedes usar 21 si lo prefieres

// Variables globales
int PWM;                          // valor de PWM que aplicamos al motor
unsigned long lastChangeTime = 0; // última vez que cambiaron los sensores Hall
const unsigned long timeout = 200;           // tiempo para forzar la secuencia de arranque suave
const unsigned long tiempoParadoCambio = 1000; // tiempo que esperamos sin cambios en los halls para considerar el motor parado

// Secuencia de conmutación para arranque en sentido directo
int secuencia[6]    = { 0b110, 0b100, 0b101, 0b001, 0b011, 0b010 };
// Secuencia de conmutación para arranque en sentido inverso
int secuenciaInv[6] = { 0b010, 0b011, 0b001, 0b101, 0b100, 0b110 };

int indiceSecuencia = 0;  // índice que nos permitirá recorrer la secuencia en orden en el loop

// 0 -> sentido directo
// 1 -> sentido inverso
int sentidoActual   = 0;  // sentido con el que está funcionando el motor ahora mismo
int cambiandoSentido = 0; // 0 -> normal, 1 -> estamos esperando a que se pare para cambiar de sentido

// -----------------------------------------------------------------------------
// DECLARAMOS LOS PINES

void setup() {
  // Sensores Hall como entrada con pullup interno
  pinMode(H0_PIN, INPUT_PULLUP);
  pinMode(H1_PIN, INPUT_PULLUP);
  pinMode(H2_PIN, INPUT_PULLUP);

  // Entrada de selección de sentido con pullup
  pinMode(DIR_PIN, INPUT_PULLUP);

  // Salidas de potencia para el control del motor
  pinMode(GH, OUTPUT);
  pinMode(GL, OUTPUT);
  pinMode(BH, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(YH, OUTPUT);
  pinMode(YL, OUTPUT);

  // Al inicio apagamos el motor
  PWM = 0;
  apagarMotor(); // Subprograma que me pone todos los transistores desconectados para parar el motor

  // Declaración de las interrupciones (cada vez que cambie cualquier sensor de efecto hall)
  attachInterrupt(digitalPinToInterrupt(H0_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H1_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), ISR_Halls, CHANGE);

  // Ajuste de frecuencia PWM alta (~31 kHz aprox) para que no se oiga el zumbido
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // Timer1 -> pin 9,10
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  // Timer3 -> pin 5
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  // Timer4 -> pin 6,13

  // Guardamos el tiempo actual
  lastChangeTime = millis();
}

// -----------------------------------------------------------------------------
// BUCLE PRINCIPAL

void loop() {

  // leo el potenciómetro
  PWM = analogRead(POT);

  // lo paso a PWM limitado al 80%
  // zona muerta al 5%
  if (PWM < 51) {
    PWM = 0;
  } else {
    PWM = map(PWM, 51, 1023, 0, 204);
  }

  unsigned long ahora = millis();  // leo el tiempo actual

  // leo el sentido pedido en la entrada digital
  int valorDir = digitalRead(DIR_PIN);  // HIGH -> directo, LOW -> inverso

  int sentidoDeseado;
  if (valorDir == HIGH) {
    sentidoDeseado = 0;   // sentido directo
  } else {
    sentidoDeseado = 1;   // sentido inverso
  }

  // Si el sentido seleccionado es marcha atrás, el ciclo de trabajo máximo debe ser del 20%
  // PWM ahora está entre 0 y 204 entonces ahora lo mapeo a 0..51 (~20% de 255)
  if (sentidoDeseado == 1) {
    PWM = map(PWM, 0, 204, 0, 51);
  }

  // Planteo el cambio de sentido:
  // Si estamos cambiando de sentido, forzamos motor parado hasta que no haya cambios en los halls
  if (cambiandoSentido == 1) {
    // Forzamos motor parado: sin PWM y todas las fases apagadas
    PWM = 0;
    apagarMotor();

    // Si ha pasado el tiempo suficiente desde el último cambio de los sensores hall,
    // consideramos que el motor ya está totalmente parado
    if ((ahora - lastChangeTime) >= tiempoParadoCambio) {
      sentidoActual = sentidoDeseado;  // aplicamos el nuevo sentido
      cambiandoSentido = 0;            // ya podemos volver a arrancar
      indiceSecuencia = 0;             // reiniciamos la secuencia de arranque suave
    }
  } else {
    // No estamos cambiando de sentido
    // Si se pide un sentido distinto mientras hay PWM, iniciamos el proceso de parada
    if ((sentidoDeseado != sentidoActual) && (PWM > 0)) {
      // se ha cambiado la entrada de sentido con el motor girando
      // primero debemos parar el motor
      cambiandoSentido = 1;
      PWM = 0;
      apagarMotor();
    }
  }

  // Si el motor puede girar (no estamos cambiando de sentido) y hay PWM,
  // utilizamos la secuencia de arranque suave como en el código original.
  if ((cambiandoSentido == 0) && (PWM > 0)) {

    
    if (ahora - lastChangeTime >= timeout) {  // Comprobamos si ha pasado el tiempo límite para forzar la secuencia

      int hallForzado;

      if (sentidoActual == 0) {
       
        hallForzado = secuencia[indiceSecuencia];  // Forzamos el siguiente paso de la secuencia en sentido directo
        giroSentidoDirecto(hallForzado);  // Obligamos a que el rotor tenga esa secuencia
      } else {
        
        hallForzado = secuenciaInv[indiceSecuencia];  // Forzamos el siguiente paso de la secuencia en sentido inverso
        giroSentidoInverso(hallForzado);  // Obligamos a que el rotor tenga esa secuencia
      }

      indiceSecuencia++;                // Avanzamos al siguiente paso de la secuencia
      if (indiceSecuencia >= 6) {
        indiceSecuencia = 0;  // Si llegamos al final de la secuencia, volvemos al inicio
      }

      lastChangeTime = ahora;  // Reiniciamos el temporizador
    }
  }
}

// -----------------------------------------------------------------------------
// INTERRUPCIONES

void ISR_Halls() {
  // Leo los valores de los sensores Hall
  int ValDIO0 = digitalRead(H0_PIN);  // Leo el hall0
  int ValDIO1 = digitalRead(H1_PIN);  // Leo el hall1
  int ValDIO2 = digitalRead(H2_PIN);  // Leo el hall2

  int hallState = (ValDIO0 << 2) | (ValDIO1 << 1) | ValDIO2;
  // asigno mi máscara para poder llamar a los sensores de efecto hall de forma
  // más clara y sencilla

  // Si no hay PWM, apagamos el motor
  if (PWM == 0) {
    apagarMotor();
  } else {
    // Según el sentido actual, llamamos a la tabla de verdad correspondiente
    if (sentidoActual == 0) {
      giroSentidoDirecto(hallState);
    } else {
      giroSentidoInverso(hallState);
    }
  }

  // Actualizo mi tiempo de forma que guardo la última vez que se movieron los sensores.
  lastChangeTime = millis();
  // Si no para de moverse se actualizará todo el rato, si no simplemente se quedará con la última.
}

// ------------------------------------------------------------------
// IMPLEMENTACIÓN DE LA TABLA DE VERDAD: SENTIDO DIRECTO

void giroSentidoDirecto(int hallState) {
  switch (hallState) {
    case 0b000:  // nunca vamos a tener los halls en 000
      break;

    case 0b001:
      // YH = 1, GL = 1
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
      // BH = 1, GL = 1
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
      // YH = 1, BL = 1
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

    case 0b111:  //nunca vamos a tener los halls en 111 no van a estar todos encendidos a la vez
      break;
  }
}

// ------------------------------------------------------------------
// IMPLEMENTACIÓN DE LA TABLA DE VERDAD: SENTIDO INVERSO

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

// ------------------------------------------------------------------
// Función para apagar completamente el puente de potencia

void apagarMotor() {
  analogWrite(GH, 0);
  analogWrite(BH, 0);
  analogWrite(YH, 0);

  digitalWrite(GL, HIGH);
  digitalWrite(BL, HIGH);
  digitalWrite(YL, HIGH);
}
