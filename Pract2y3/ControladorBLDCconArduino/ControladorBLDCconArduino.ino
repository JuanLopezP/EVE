// Código para controlar un motor y hacer que gire



const int GH = 4;  // GREEN HIGH  (PWM)
const int BH = 5;  // BLUE HIGH  (PWM)
const int YH = 8;  // YELLOW HIGH (PWM)

const int GL = 9;  // GREEN LOW  (digital)
const int BL = 10;  // BLUE LOW  (digital)
const int YL = 11; // YELLOW LOW  (digital)

const int POT = A0;

bool sentidoHorario = true;


-------------------------------------->

void setup() {
  // Inicializo puerto serie
  Serial.begin(9600);
// Entradas digitales para los sensores de efecto Hall 

  pinMode(2,INPUT);
  pinMode(3, INPUT);
  pinMode(7, INPUT);

// Salida digital para los transistores 
// Transistores altos --> deben tener capacidad para generar señales PWM
  pinMode(GH,OUTPUT);
  pinMode(BH,OUTPUT);
  pinMode(YH,OUTPUT);
// Transistores bajos
  pinMode(GL,OUTPUT);
  pinMode(BL,OUTPUT);
  pinMode(YL,OUTPUT);

// Entrada analógica 
  pinMode(POT,INPUT); // para controlar el potenciómetro 
}

------------------------------------------------------------------------------>
void loop() {
  // put your main code here, to run repeatedly:

  int valorPot = analogRead(POT); // lee un valor de (0-1023)
  int valorPotDig = map(valorPot,0,1023,0,204); // lo convierto a digital para las señales y limito el 80%

// Leo los sensores de efecto Hall
  int H0 = digitalRead(2);  // 0 o 1
  int H1 = digitalRead(3);
  int H2 = digitalRead(7);  


  Serial.print("Valor del potenciómetro: ");
  Serial.print(valorPot);
  Serial.print("         Valor de salida digital: ");
  Serial.println(valorPotDig);

// Esto es para hacer que si estan por debajo del 5% se apaguen todos si o si
  if (valorPotDig <= (255*0.05) ) {
    analogWrite(GH,0);
    analogWrite(BH,0);
    analogWrite(YH,0);

    digitalWrite(GL,LOW);
    digitalWrite(BL,LOW);
    digitalWrite(YL,LOW);

  // Info por si acaso
    Serial.print("STOP | pot =");
    Serial.print(valorPot);
    Serial.print(" duty cycle =");
    Serial.println(valorPotDig);

    delay(50);
    return;  // MUY IMPORTANTE: no seguimos encendiendo nada más abajo
  } 

// Ahora toca implementar para conseguir movimiento con el motor

// ECUACIONES

if (sentidoHorario) {
int GH_EC = H0 && !H2;
int BH_EC = !H0 && H1;
int YH_EC = !H1 && H2;

int GL_EC = !H0 && H2;
int BL_EC = H0 && !H1;
int YL_EC = H1 && !H2;
else {
  // Ecuaciones inversas (orden invertido)
  GH_EC =  H1 && (!H2) ;
  BH_EC =  (!H1) && H0 ;
  YH_EC =  (!H0) && H2 ;

  GL_EC =  (!H1) && H2 ;
  BL_EC =  H1 && (!H0) ;
  YL_EC =  H0 && (!H2) ;
}

// Giro del motor cuando la ecuación sea 1

if (GH_EC) analogWrite(GH, valorPotDig); else analogWrite(GH, 0);
if (BH_EC) analogWrite(BH, valorPotDig); else analogWrite(BH, 0);
if (YH_EC) analogWrite(YH, valorPotDig); else analogWrite(YH, 0);

digitalWrite(GL, GL_EC ? HIGH : LOW);
digitalWrite(BL, BL_EC ? HIGH : LOW);
digitalWrite(YL, YL_EC ? HIGH : LOW);

  delay(50);

}
