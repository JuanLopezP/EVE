// Código para controlar un motor BLDC y hacerlo girar

// Pines de los transistores
const int GH = 5;   // GREEN HIGH  (PWM)
const int BH = 6;   // BLUE HIGH   (PWM)
const int YH = 9;   // YELLOW HIGH (PWM)

const int GL = 4;   // GREEN LOW   (digital)
const int BL = 8;  // BLUE LOW    (digital)
const int YL = 13;  // YELLOW LOW  (digital)

// Potenciómetro --> acelerador
const int POT = A0;

// Pines de los sensores Hall con interrup
const int H0_PIN = 2;
const int H1_PIN = 3;
const int H2_PIN = 10;

// Variables globales
bool sentidoHorario = true;  // booleano para elegir sentido de giro 
volatile int dutyGlobal = 0; // variable para el duty que usarán las salidas PWM
volatile int H0 = 0;
volatile int H1 = 0;
volatile int H2 = 0;

// ---------------------------------------------------------------------------

void setup() {

  Serial.begin(9600);

  // Pines de los sensores Hall como entrada
  
  pinMode(H0_PIN, INPUT_PULLUP);
  pinMode(H1_PIN, INPUT_PULLUP);
  pinMode(H2_PIN, INPUT_PULLUP);


  pinMode(GH, OUTPUT);
  pinMode(BH, OUTPUT);
  pinMode(YH, OUTPUT);


  pinMode(GL, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(YL, OUTPUT);

  // Entrada analógica del potenciómetro
  pinMode(POT, INPUT);

  // Ajuste de frecuencia PWM alta (~31 kHz aprox)
  // (para que no suene el motor)
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // frecuencia  pines 9 y 10
  TCCR3B = TCCR3B & 0b11111000 | 0x01; // frecuencia  pin 5
  TCCR4B = TCCR4B & 0b11111000 | 0x01; // frecuencia  pines 6 y 13

  // Leemos una primera vez los halls para no arrancar a ciegas
  H0 = digitalRead(H0_PIN);
  H1 = digitalRead(H1_PIN);
  H2 = digitalRead(H2_PIN);

  // Configuro las interrupciones de los 3 halls.
  // CHANGE = cuando cambie (suba o baje) cualquiera.
  attachInterrupt(digitalPinToInterrupt(H0_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H1_PIN), ISR_Halls, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), ISR_Halls, CHANGE);

  // Arrancamos con duty = 0 (parado)
  dutyGlobal = 0;
  
}


// ---------------------------------------------------------------------------


void loop() {


  int valorPot = analogRead(POT);

  // Convierto a PWM (0 a 204) --> esto es el 80% de 255 aprox
  int valorPotDig = map(valorPot, 0, 1023, 0, 204);

  // Si está por debajo del 5% lo paramos todo sí o sí

  if (valorPotDig <= 13) {

    // Poner duty a cero
    dutyGlobal = 0;

    // Apagar transistores altos
    //analogWrite(GH, 0);
    //analogWrite(BH, 0);
    //analogWrite(YH, 0);

    // Apagar transistores bajos
    //digitalWrite(GL, HIGH);
    //digitalWrite(BL, HIGH);
    //digitalWrite(YL, HIGH);

    // Mensaje por serie
   // Serial.print("STOP | pot = ");
    //Serial.print(valorPot);
   // Serial.print("   duty = ");
    //Serial.println(valorPotDig);

 
    return; // importante: saltamos el resto del loop pq no me interesa seguir adelante
  }


  dutyGlobal = valorPotDig;  // guardo el duty en la global que usa la ISR

 
  

  // Mensajes para ver qué pasa
  //Serial.print("RUN  | pot = ");
  //Serial.print(valorPot);
  //Serial.print("   duty = ");
  //Serial.print(valorPotDig);

  // También vemos el estado hall actual
  //Serial.print("   H0=");
  //Serial.print(H0);
  //Serial.print(" H1=");
  //Serial.print(H1);
  //Serial.print(" H2=");
  //Serial.println(H2);

}




// ---------------------------------------------------------------------------

// Interrupción: cada vez que cambia alguno de los sensores Hall
void ISR_Halls() {
  // leemos los halls
  H0 = digitalRead(H0_PIN);
  H1 = digitalRead(H1_PIN);
  H2 = digitalRead(H2_PIN);

 
   // ecuaciones lógicas para cada transistor (EC = ecuación)


  if (sentidoHorario) {

    int GH_EC = H0 && !H2;
    int BH_EC = !H0 && H1;
    int YH_EC = !H1 && H2;

    int GL_EC = H0 && !H2;
    int BL_EC = !H0 && H1;
    int YL_EC = !H1 && H2;

  } else {

  }

  // Ahora activamos los transistores según las ecuaciones

  // Transistores altos -> PWM con el duty que toque
  if (GH_EC) {
    analogWrite(GH, dutyGlobal);

  } else {
    analogWrite(GH, 0);
    
  }

  if (BH_EC) {
    analogWrite(BH, dutyGlobal);
  } else {
    analogWrite(BH, 0);
  }

  if (YH_EC) {
    analogWrite(YH, dutyGlobal);
  } else {
    analogWrite(YH, 0);
  }

  // Transistores bajos --> digital normal

  digitalWrite(GL, GL_EC ? LOW : HIGH);
  digitalWrite(BL, BL_EC ? LOW : HIGH);
  digitalWrite(YL, YL_EC ? LOW : HIGH);
}


//-----------------------------------------------------------------------

// Esta función actualiza las salidas del puente según los sensores Hall

/*void actualizarPuente() {

  // ecuaciones lógicas para cada transistor (EC = ecuación)
  int GH_EC;
  int BH_EC;
  int YH_EC;

  int GL_EC;
  int BL_EC;
  int YL_EC;

  if (sentidoHorario) {

    GH_EC =  (H0 && !H2);
    BH_EC =  (!H0 && H1);
    Serial.println(BH_EC);
    YH_EC =  (!H1 && H2);

    GL_EC =  (!H0 && H2);
    BL_EC =  (H0 && !H1);
    YL_EC =  (H1 && !H2);

  } else {
    // giro inverso
    GH_EC =  (H1 && (!H2));
    BH_EC =  ((!H1) && H0);
    YH_EC =  ((!H0) && H2);

    GL_EC =  ((!H1) && H2);
    BL_EC =  (H1 && (!H0));
    YL_EC =  (H0 && (!H2));
  }

  // Ahora activamos los transistores según las ecuaciones

  // Transistores altos -> PWM con el duty que toque
  if (GH_EC) {
    analogWrite(GH, dutyGlobal);
  } else {
    analogWrite(GH, 0);
  }

  if (BH_EC) {
    analogWrite(BH, dutyGlobal);
  } else {
    analogWrite(BH, 0);
  }

  if (YH_EC) {
    analogWrite(YH, dutyGlobal);
  } else {
    analogWrite(YH, 0);
  }

  // Transistores bajos --> digital normal

  digitalWrite(GL, GL_EC ? HIGH : LOW);
  digitalWrite(BL, BL_EC ? HIGH : LOW);
  digitalWrite(YL, YL_EC ? HIGH : LOW);
}
*/