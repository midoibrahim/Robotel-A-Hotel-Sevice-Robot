#include <PID_v1.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SimpleKalmanFilter.h>

// Please insert your motor encoder output pulse per rotation
#define ENCODEROUTPUT 396

#define RST_PIN   46
#define SS_PIN    53

#define HALLSENA 2 // Hall sensor A connected to pin 2 (external interrupt)
#define HALLSENB 3 // Hall sensor A connected to pin 3 (external interrupt)

#define PWMA 6 // PWM connected to pin 6
#define PWMB 7 // PWM connected to pin 7

#define T 9 // trig
#define E 10 // echo

#define DIRA1 24 // DIR1 connected to pin 24
#define DIRA2 22 // DIR2 connected to pin 22

#define DIRB1 23 // DIR1 connected to pin 24
#define DIRB2 25 // DIR2 connected to pin 25

#define us 31 // DIR1 connected to pin 31

boolean graph = !true; //for debug

// defines variables
double distance = 0;
double pdistance = 0;


//Define Variables we'll be connecting to
double SetpointA, rpmA, motorPwmA;
//Define Variables we'll be connecting to
double SetpointB, rpmB, motorPwmB;
double error,delta;

long encoderValueA;
long encoderValueB;

int timer1_counter; //for timer

double revA;
double revB;
double dis=10000;
double Seterr=0;
//double prpmA;
//double prpmB;

int speedrob=50;
int c=0;


long distU;
int uc;
boolean sent1=0;
boolean sent2=0;

double aggKp=0.7, aggKi=2.0, aggKd=0.05;

SimpleKalmanFilter KalmanA(1, 1, 0.01);
SimpleKalmanFilter KalmanB(1, 1, 0.01);
SimpleKalmanFilter KalmanR(20, 0.01, 0.01);

PID myPIDA(&rpmA, &motorPwmA, &SetpointA, aggKp, aggKi, aggKd, DIRECT);
PID myPIDB(&rpmB, &motorPwmB, &SetpointB, aggKp, aggKi, aggKd, DIRECT);
PID myPIDR(&error,&delta,&Seterr,         0.3,0.3,0.1, DIRECT);

//RFID and LCD
byte readCard[4];
String myTags[] = {"67CB3CD7","4EE1A7E9","F71FF8D7","7692AC8B"};
boolean succ = false;
boolean turn = false;
boolean go = false;

boolean gflag=false;
boolean tflag=false;

String tagID = "";
int target=2;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Create instances
MFRC522 mfrc522(SS_PIN, RST_PIN);


void setup()
{
  Serial.begin(115200); // Initialize serial with 115200 baudrate
  
  pinMode(HALLSENA, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSENB, INPUT_PULLUP); // Set hall sensor A as input pullup
  
  pinMode(PWMA, OUTPUT); // Set PWM pin as output
  pinMode(DIRA1, OUTPUT); // Set DIR1 pin as output
  pinMode(DIRA2, OUTPUT); // Set DIR2 pin as output

  
  pinMode(PWMB, OUTPUT); // Set PWM pin as output
  pinMode(DIRB1, OUTPUT); // Set DIR1 pin as output
  pinMode(DIRB2, OUTPUT); // Set DIR2 pin as output
  pinMode(us, INPUT); // Set DIR2 pin as output

  pinMode(us, OUTPUT);
  pinMode(T,OUTPUT); // Set trig pin as output
  pinMode(E,INPUT_PULLUP); // Set echo pin as input
  digitalWrite(us, LOW);
  // Attach interrupt at hall sensor A on each rising signal
  
  attachInterrupt(digitalPinToInterrupt(HALLSENA), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSENB), updateEncoderB, RISING);
    
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
  //--------------------------timer setup
  
  digitalWrite(DIRA2,HIGH);
  digitalWrite(DIRA1,LOW);
  digitalWrite(DIRB2,HIGH);
  digitalWrite(DIRB1,LOW);
    
  SetpointA=0;
  SetpointB=0;

  encoderValueA=0;
  encoderValueB=0;
  rpmA = 0;
  rpmB = 0;
  revA=0;
  revB=0;
  motorPwmA = 0;
  motorPwmB = 0;

  //turn the PID on
  myPIDA.SetMode(AUTOMATIC);
  myPIDB.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);
  myPIDA.SetSampleTime(100);
  myPIDB.SetSampleTime(100);
  myPIDR.SetSampleTime(100);
  myPIDA.SetOutputLimits(0, 255);
  myPIDB.SetOutputLimits(0, 255);
  myPIDR.SetOutputLimits(-50,50);

  SPI.begin();        // SPI bus
  mfrc522.PCD_Init(); //  MFRC522
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  // Prints the initial message
  printNormalModeMessage(); 
}
boolean correct=false;
void RFid()
{
    if( ! mfrc522.PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
      return;
    }
    if ( ! mfrc522.PICC_ReadCardSerial()) {   //Since a PICC placed get Serial and continue
      return;
    }
    tagID = "";
    // The MIFARE PICCs that we use have 4 byte UID
    for ( uint8_t i = 0; i < 4; i++) {  //
      readCard[i] = mfrc522.uid.uidByte[i];
      tagID.concat(String(mfrc522.uid.uidByte[i], HEX)); // Adds the 4 bytes in a single String variable
    }
    tagID.toUpperCase();
    mfrc522.PICC_HaltA(); // Stop reading
 
    //Serial.println(tagID);
    // Checks whether the scanned tag is the target tag
    if (tagID == myTags[target]) { 
      //Serial.println("rfid");
      lcd.clear();
      lcd.print("Pick Up package");
      lcd.setCursor(0, 1);
      lcd.print("   Enjoy!! :)  ");
      succ = true;
      s();
      if(correct)
        {
      Serial.println('w');    
      correct=false;
      delay(2000);
      if (target%2==0){ p();}
      else if (target%2==1) {n();}
       }
    correct=true;
    }
    else
    {
      lcd.clear();
      lcd.print("Package for Room");
      lcd.setCursor(8, 1);
      lcd.print(target);
      succ = false;
    }
}

void printNormalModeMessage() {
  delay(500);
  lcd.clear();
  lcd.print("   -Robotel-    ");
  lcd.setCursor(0, 1);
  lcd.print("Ready 2 Deliever");
}

char s0;

void serialEvent() {
  //if(digitalRead(us)){ Serial.flush();}
  while (Serial.available())
  {
   s0=Serial.read();
   //Serial.println(a);
   if(isDigit(s0))
   {
    String p = Serial.readString();
    p=s0+p;
    dis=p.toDouble();
    //Serial.println(p);
   }
    switch(s0)
  {
    case 'f':
    f(); 
    break;
    case 'r':
    r(); 
    break;
    case 'l':
    l(); 
    break;
    case 's':
    s();
    break;
    case 'n':
    n();
    break;
    case 'p':
    p(); 
    break;
    case 't':
    t(); 
    break;
  }
 }
}
void checkU()
{
  // Clears the trigPin
  digitalWrite(T, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(T, HIGH);
  delayMicroseconds(10);
  digitalWrite(T, LOW);
  distU = SonarSensor(T,E);
  //Serial.println(distU);
  if (distU < 60) {
    uc++;
    if(uc > 3){digitalWrite(us, HIGH);}
    }
  else{uc=0; digitalWrite(us, LOW);}
}
  long SonarSensor(int trigPin,int echoPin)
{
  delay(200);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration/2) / 29.1;
  if (distance >= 500 || distance <= 0){
    distance=100;
    }
  //Serial.println(distance);
  return distance;
}   

void loop()
{
  //noInterrupts();
  //Serial.println(digitalRead(us));
  checkU();
  if(go && digitalRead(us)) {pdistance = distance; s(); go=true; distance=pdistance;}
  if(go && !digitalRead(us)) {delay(1000); f();}
  RFid();
}

void updateEncoderA()
{
  // Add encoderValue by 1, each time it detects rising signal from hall sensor A
  encoderValueA++;
  //Serial.println(encoderValueA);
}

void updateEncoderB()
{
  // Add encoderValue by 1, each time it detects rising signal from hall sensor A
  encoderValueB++;
  //Serial.println(encoderValueB);
}

void f()
{
  //Serial.println('f');
  if (!digitalRead(us))
  {
     turn = false;
     go=true;
     digitalWrite(DIRA2,HIGH);
     digitalWrite(DIRA1,LOW);
     digitalWrite(DIRB2,HIGH);
     digitalWrite(DIRB1,LOW);
     SetpointA=SetpointB=speedrob;
     Seterr=0; 
  }    
}
void t()
{
  while (!Serial.available()) {}
   s0=Serial.read();
   //Serial.println(a);
   if(isDigit(s0))
   {
    String p = Serial.readString();
    p=s0+p;
    target=p.toInt();
    //Serial.println(target);
    s();
   }
}
  void p()
{
     s();
     turn = true;
     digitalWrite(DIRA2,HIGH);
     digitalWrite(DIRA1,LOW);
     digitalWrite(DIRB1,HIGH);
     digitalWrite(DIRB2,LOW);
     SetpointA=SetpointB=50;
   
}
void n()
{    
     s(); 
     turn = true;
     digitalWrite(DIRA1,HIGH);
     digitalWrite(DIRA2,LOW);
     digitalWrite(DIRB2,HIGH);
     digitalWrite(DIRB1,LOW);
     SetpointA=SetpointB=50;
}
void r()
{
     s(); 
     digitalWrite(DIRA2,HIGH);
     digitalWrite(DIRA1,LOW);
     digitalWrite(DIRB1,HIGH);
     digitalWrite(DIRB2,LOW);
     SetpointA=speedrob;
     SetpointB=0;
     analogWrite(PWMA , 190);
     analogWrite(PWMB , 190);
     delay(50);
     s();
}
void l()
{    s();
     digitalWrite(DIRA1,HIGH);
     digitalWrite(DIRA2,LOW);
     digitalWrite(DIRB2,HIGH);
     digitalWrite(DIRB1,LOW);
     SetpointA=0;
     SetpointB=speedrob;
     analogWrite(PWMA , 190);
     analogWrite(PWMB , 190);
     delay(50);
     s();
}
void s()
{    
     turn = false;
     go=false;
     analogWrite(PWMA , 0);
     analogWrite(PWMB , 0);
     digitalWrite(DIRA1,HIGH);
     digitalWrite(DIRA2,HIGH);
     digitalWrite(DIRB2,HIGH);
     digitalWrite(DIRB1,HIGH);
     //Serial.println('s');
     //reset every variable and PID
     myPIDA.outputSum=0;
     myPIDB.outputSum=0;
     myPIDR.outputSum=0;
     motorPwmA=0;
     motorPwmB=0;
     delta=0;
     SetpointA=0;
     SetpointB=0;
     rpmA=0;
     rpmB=0;
     encoderValueA=0;
     encoderValueB=0;
     revA=0;
     revB=0;
     error=0;
     Seterr=0;
     distance=0;
     error=0;
     KalmanA.setEstimateError(1);
     KalmanB.setEstimateError(1);
     KalmanR.setEstimateError(1);
}
int tr=12;

ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
{
  if(go || turn)
  { 
    rpmA = (double)(encoderValueA*60/(ENCODEROUTPUT*0.1));
    rpmB = (double)(encoderValueB*60/(ENCODEROUTPUT*0.1));

    rpmA=KalmanA.updateEstimate(rpmA);
    rpmB=KalmanB.updateEstimate(rpmB);

    revA=revA+(double)(encoderValueA*1.0/(ENCODEROUTPUT*1.0));
    revB=revB+(double)(encoderValueB*1.0/(ENCODEROUTPUT*1.0));
    error=(revA-revB)*100;
    error = KalmanR.updateEstimate(error);
    encoderValueA=0;
    encoderValueB=0;
    if(!turn)
    {
    distance = ((revA+revB)/2)*39.27;
    if(distance > dis-22) 
    {
      s();
      if (target%2==0){p();}
      else if (target%2==1) {n();}
      if(succ) 
      {
      if (target%2==0){tr=24; p();}
      else if (target%2==1) {tr=24; n();}
      succ=false;
      }
    }
    }
    if(turn)
    {
      distance = ((revA+revB)/4)*39.27;
      if(distance > tr){s(); tr=12; 
      if(!succ){ 
        Serial.println('a');
        succ = true;}
    }
    }
    if(rpmA==0 && rpmB==0 && SetpointA==0 && SetpointB==0)
     { 
     motorPwmA=0;
     motorPwmB=0;
     delta=0;
     digitalWrite(DIRA1,HIGH);
     digitalWrite(DIRA2,HIGH);
     digitalWrite(DIRB2,HIGH);
     digitalWrite(DIRB1,HIGH);
     }
     else
     {
      myPIDA.Compute();
      myPIDB.Compute();
      if(go && !turn) myPIDR.Compute();
      analogWrite(PWMA , motorPwmA+delta*1);
      analogWrite(PWMB , motorPwmB-delta*1);
     }
  }

  TCNT1 = timer1_counter;   // set timer
  
  if(graph)
  {
  //Serial.print(SetpointB);
  //Serial.print(',');
  //Serial.print(rpmA);
  //Serial.print(',');
  //Serial.println(rpmB);
  //Serial.print(error);
  //Serial.print(',');
  //Serial.print(distance);
  //Serial.print(',');
  //Serial.print(motorPwmA+delta);
  //Serial.print(',');
  //Serial.print(motorPwmB-delta);
  //Serial.print(',');
  //Serial.print(0);
  //Serial.print(',');
  //Serial.println(error);
  }
}
