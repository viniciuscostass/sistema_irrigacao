#include <Servo.h>

const int sensor3 = 11;
const int sensor2 = 12;
const int umiditySensor = A0;
const int ldr = A1;

const int sensor1 = A2;
const int redLed = A5;
const int yellowLed = A4;
const int greenLed = A3;
const int whiteLed = 6;

const int powerBumpButton = 8;
const int button1 = 2;
const int servoButton = 4;
const int pwmWaterBump = 3;
const int buzzer = 7;



Servo waterValveServo;

int valueSensor1;
int valueSensor2;
int valueSensor3;
int valuePwmBump;
int valueLdr;
int valuePowerBumpButton;
int valueUmiditySensor;
int valueServoButton;
int redLedSignal;
int yellowLedSignal;
int greenLedSignal;
int buzzerSignal;
int servoSignal;
int openTime;
boolean timerOn;

int opCode;

ISR(TIMER2_OVF_vect) {
  TCNT2 = 130;
  if(timerOn){
    openTime += 1;
    if(openTime == 10000){
      Serial.println("entrou na isr");
      timerOn = false; 
    }
  }
}


void change_label(){
  if(opCode == 1){
    opCode = 0;
  }else if(opCode == 0){
    opCode = 1;
    valuePwmBump = 0;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(umiditySensor, INPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(servoButton, INPUT);
  pinMode(powerBumpButton, INPUT);
  pinMode(ldr, INPUT);
  pinMode(pwmWaterBump, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(whiteLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  waterValveServo.attach(9);

  opCode = 0;

  redLedSignal = 0;
  yellowLedSignal = 0;
  greenLedSignal = 0;
  buzzerSignal = 0;
  
  attachInterrupt(digitalPinToInterrupt(button1), change_label, RISING);

  // Configuração do timer2 
  TCCR2A = 0;                        //confira timer para operação normal pinos OC2A e OC2B desconectados
  TCCR2B = 0;                        //limpa registrador
  TCCR2B |= (1<<CS10)|(1 << CS12);   // configura prescaler para 1024: CS12 = 1 e CS10 = 1
 
  TCNT2 = 130;                    // incia timer com valor para que estouro ocorra em 0.001 segundo
  
  TIMSK2 |= (1 << TOIE2);           // habilita a interrupção do TIMER2

}

void loop() {
 
  valueUmiditySensor = digitalRead(umiditySensor);
  valueLdr = analogRead(ldr);
//  Serial.println(valueLdr);
  valueSensor1 = digitalRead(sensor1);
  valueSensor2 = digitalRead(sensor2);
  valueSensor3 = digitalRead(sensor3);
  valuePowerBumpButton = digitalRead(powerBumpButton);
  valueServoButton = digitalRead(servoButton);
  servoSignal = 0;

  yellowLedSignal = 0;
  greenLedSignal = 0;
  buzzerSignal = 0;
  valuePwmBump = 0;


  if(valueUmiditySensor == 1 && valueLdr > 50){
    servoSignal = 180;
  }
  if(valueSensor1 == 1){
    redLedSignal = 0;
    if(valueSensor3 ==  0){
      greenLedSignal = 0;
      yellowLedSignal = 1;
      if(valueSensor2 == 1){
        redLedSignal = 1;
        valuePwmBump = 255;
      }else{
        valuePwmBump = 190;
      }
    }else{
      greenLedSignal = 1;
      valuePwmBump = 0;
    }
  }else{
    redLedSignal = 1;
    buzzerSignal = 1;
    valuePwmBump = 0;
  }

  if(opCode == 0){
  
  }else if(opCode == 1){
    if(valueSensor1 == 0 || valueSensor3 == 1) {
      valuePwmBump = 0;
    } else {
      if(valuePowerBumpButton == 1){
        valuePwmBump = 255;
      } else {
        valuePwmBump = 0;
      }
    }

    if(!timerOn){
      if(valueServoButton == 1){
        timerOn = true;
        openTime = 0;
        servoSignal = 180;
      }else{
        servoSignal = 0;
      }
    }
  }
  Serial.println(timerOn);
  digitalWrite(redLed, redLedSignal);
  digitalWrite(yellowLed, yellowLedSignal);
  digitalWrite(greenLed, greenLedSignal);
  digitalWrite(whiteLed, opCode);
  digitalWrite(buzzer, buzzerSignal);
  analogWrite(pwmWaterBump, valuePwmBump);
  waterValveServo.write(servoSignal);

  
}
