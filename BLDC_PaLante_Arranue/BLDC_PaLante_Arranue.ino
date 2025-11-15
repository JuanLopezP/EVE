volatile bool flagCambioHall = false;  // Flag para indicar cambio en sensores
int hallA, hallB, hallC;               // Variables de sensores
int acel, PWM; 
int pinsentido=20;
int valorActual, valorAnterior;



void setup() {
TCCR1B = TCCR1B & 0b11111000 | 0x01; // frecuencia en los pines 9 y 10: aproximadamente 31250 Hz
TCCR3B = TCCR3B & 0b11111000 | 0x01; // frecuencia en el pin 5: aproximadamente 31250 Hz
TCCR4B = TCCR4B & 0b11111000 | 0x01; // frecuencia en los pines 6 y 13: aproximadamente 31250 Hz

  pinMode(5, OUTPUT);     // GA HING   PWm
  pinMode(4, OUTPUT);     // GB LING 
  pinMode(6, OUTPUT);     // BA HINB PWM 
  pinMode(8, OUTPUT);     // BB LIMB
  pinMode(9, OUTPUT);     // YA HINY PWM
  pinMode(10, OUTPUT);    // YB LINY

   // EFecto HALL 
  pinMode(2, INPUT_PULLUP);    // HVerde D2 
  pinMode(3, INPUT_PULLUP);    // HAZUL D3 
  pinMode(7, INPUT_PULLUP);    // HA D7
  pinMode(pinsentido, INPUT_PULLUP);  // Pin que indica el sentido
  // Analógica 
  pinMode(A0, INPUT);   
  
  // --- Configurar interrupciones en los sensores Hall ---
//  attachInterrupt(digitalPinToInterrupt(2), cambioHall, CHANGE); // HVerde
//  attachInterrupt(digitalPinToInterrupt(3), cambioHall, CHANGE); // HAzul
//  attachInterrupt(digitalPinToInterrupt(7), cambioHall, CHANGE);
}


void loop() {
  // Leer acelerador
  acel = analogRead(A0);
  valorActual = digitalRead(pinsentido);   // Leer el pin (0 o 1)
//  Serial.print("Valor digital: ");
//  Serial.println(valor);


  // Escalar el valor del acelerador (0–1023) a PWM 0–80% (≈204)
  if (acel >= 1023 * 0.05) {
    PWM = map(acel, 1023 * 0.05, 1023, 0, 255 * 0.8);
    if (valorAnterior != valorActual) {
      PWM = 0; 
      delay(1000); 
      }
      cambioHall();
    } 
  else  {
    PWM = 0;
        }
  valorAnterior = valorActual; 
}

// --- Rutina de interrupción (compartida por los 3 sensores Hall) ---


  
// PWM        LOW HIGH    PWM     HIGH LOW     PWM              HIGH LOW
// G A 5,     G B 4,      B A 6,   B B 8,       Y A 9 ,            Y B  10 
void cambioHall() {


  hallA = digitalRead(2);  // Verde
  hallB = digitalRead(3);  // Azul
  hallC = digitalRead(7);  // Amarillo

if ( valorActual==0){
  hallA = !hallA;  // Verde
  hallB = !hallB;    // Azul
  hallC = !hallC;    // Amarillo
}
 if (hallA == LOW && hallB == LOW && hallC == HIGH) {
    analogWrite(5, 0);     digitalWrite(4, LOW);
    analogWrite(6, 0);   digitalWrite(8, HIGH);
    analogWrite(9, PWM);     digitalWrite(10, HIGH);
  }

  else if (hallA == LOW && hallB == HIGH && hallC == HIGH) {
    analogWrite(5, 0);     digitalWrite(4, LOW);
    analogWrite(6, PWM);     digitalWrite(8, HIGH);
    analogWrite(9, 0);   digitalWrite(10, HIGH);
  }

  else if (hallA == LOW && hallB == HIGH && hallC == LOW) {
    analogWrite(5, 0);     digitalWrite(4, HIGH);
    analogWrite(6, PWM);     digitalWrite(8, HIGH);
    analogWrite(9,0);   digitalWrite(10, LOW);
  }

  else if (hallA == HIGH && hallB == HIGH && hallC == LOW) {
    analogWrite(5, PWM);   digitalWrite(4, HIGH);
    analogWrite(6, 0);     digitalWrite(8, HIGH);
    analogWrite(9, 0);     digitalWrite(10, LOW);
  }

  else if (hallA == HIGH && hallB == LOW && hallC == LOW) {
    analogWrite(5, PWM);   digitalWrite(4, HIGH);
    analogWrite(6, 0);     digitalWrite(8, LOW);
    analogWrite(9, 0);     digitalWrite(10, HIGH);
  }

  else if (hallA == HIGH && hallB == LOW && hallC == HIGH) {
    analogWrite(5, 0);     digitalWrite(4, HIGH);
    analogWrite(6, 0);   digitalWrite(8, LOW);
    analogWrite(9, PWM);     digitalWrite(10, HIGH);
  }
}

