#include <BluetoothSerial.h>
//--------- Flag structure --------------------------------------
typedef struct _vFlag
{
  uint8_t BTFlag = 0;
  uint8_t L298NFlag = 0;
  uint8_t CANFlag = 0;
  uint8_t I2C_Flag = 0;
  uint8_t BMP180Flag = 0;
  uint8_t DS18B20Flag = 0;
  uint8_t JSONFlag = 0;
  uint8_t LEDFlag = 1;
  uint8_t back_light_Flag = 0;
  uint8_t front_light_Flag = 0;
  uint8_t sensor1_Flag = 0;
  uint8_t initial_Flag = 0;
  uint8_t Tone_Flag = -1;
  uint8_t IR_RECV_Flag=0;
  uint8_t IR_SEND_Flag=0;
  uint8_t FunctionFlag = 3;
  uint8_t SendFlag = 0;
  uint8_t BMPCnt = 0;
} vFlag;
vFlag *flag_Ptr;
vFlag flag;

//----------uart--------------
#define LINE_BUFFER_LENGTH 64
//--------- uart structure --------------------------------------
typedef struct _vUart
{
  char c;
  int lineIndex = 0;
  int line1Index = 0;
  int BTlineIndex = 0;
  bool lineIsComment; 
  bool lineSemiColon;
  char line[128];
  char BTline[20];
  String inputString;
  String BTinputString;
  String S1inputString;
  int V[16];
  char ctemp[30];
  char I2C_Data[80];
  int DC_Spped = 50;
  float Voltage[16];
  int Buffer[128];
  int StartCnt = 0;
  int ReadCnt = 0;
  int sensorValue = 0;
} vUart;
vUart *Uart_Ptr;
vUart Uart;
//---------BT--------------------
BluetoothSerial SerialBT;
//------LED------------------
#define LED_BUILTIN 2
//-------------L298---------------------------------------------------
// motor 1 settings
#define CHA 0
#define ENA 4 // this pin must be PWM enabled pin if Arduino board is used
#define IN_1 16 //rx2
#define IN_2 17 //tx2
// motor 2 settings
#define IN_3 18 //d18
#define IN_4 19 //d19
#define ENB 5// this pin must be PWM enabled pin if Arduino board is used
#define CHB 1

#define timeSeconds 1  //pir timer


const int CCW = 2; // do not change
const int CW  = 1; // do not change

#define motor1 1 // do not change
#define motor2 2 // do not change


//------Bluetooth RC Controller Define ----
#define back_light 21 //D21
#define front_light 22 //D22
#define horn_Buzz 26 //D26    

// Set GPIOs for LED and PIR Motion Sensor
//const int led = 22;
const int motionSensor = 27;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;
boolean motion = false;


// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  digitalWrite(front_light, HIGH);
  digitalWrite(horn_Buzz, HIGH);
  startTimer = true;
  lastTrigger = millis();
}

String data="X";
char value;


void Forward(){ 

      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
     // analogWrite(ENA, speedCar);
Serial.println("forward");
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      //analogWrite(ENB, speedCar);
  }

void Reverse(){ 

      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
     // analogWrite(ENA, speedCar);
Serial.println("back");
      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
     // analogWrite(ENB, speedCar);
  }

void Right(){ 

      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
     // analogWrite(ENA, speedCar);
Serial.println("right");
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
     // analogWrite(ENB, speedCar);
  }

void Left(){

      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
     // analogWrite(ENA, speedCar);
Serial.println("left");
      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
    //  analogWrite(ENB, speedCar);
  }

void goAheadRight(){
      
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      //analogWrite(ENA, speedCar/speed_Coeff);
 Serial.println("ahead right");
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
     // analogWrite(ENB, speedCar);
   }



void goAheadLeft(){
      
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
     // analogWrite(ENA, speedCar);
Serial.println("ahead left");
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
     // analogWrite(ENB, speedCar/speed_Coeff);
  }


void Stop(){  

      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      //analogWrite(ENA, speedCar);
       Serial.println("stop");
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, LOW);
    //  analogWrite(ENB, speedCar);
 }
//-------------------------------------------------
void setup() 
{
  Serial.begin(115200);
  Serial.println(F("init"));
  SerialBT.begin("BT_MSKCAR02");
  Serial.println("Bot Started .......");
// PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT); 

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(back_light, OUTPUT);
  pinMode(front_light, OUTPUT);
  pinMode(horn_Buzz, OUTPUT);

   // Set LED to LOW
  //pinMode(led, OUTPUT);
  digitalWrite(front_light, LOW);
  digitalWrite(horn_Buzz, LOW);
  
}
//-----------------------------------------
void loop() 
{
  // Current time
  now = millis();
  if((digitalRead(front_light) == HIGH) && (motion == false)) {
    Serial.println("MOTION DETECTED!!!");
    
     Stop();
    
    motion = true;
  }
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("Motion stopped...");
    digitalWrite(front_light, LOW);
    digitalWrite(horn_Buzz, LOW);
    Serial.println("MOTION Stop");
    
    startTimer = false;
    motion = false;
  }
  
  
    if(SerialBT.available()>0)
    {
      //flag.L298NFlag=1;
     
      value = SerialBT.read();
     // data = SerialBT.readString();
     // Stop();
      
      Serial.println(data);
     // Serial.println(value);
    }  
      if (value =='F' || value =='U')
  {
    Serial.println("Forward");
    Forward();
  }
  else if (value == 'L')
  {
    Serial.println("Left");
    Left();
  }
  else if (value == 'R')
  {
    Serial.println("Right");
    Right();
  }
  else if (value =='B' || value =='D')
  {
    Serial.println("Reverse");
    Reverse();
  }
  else if (value == 'X' || value =='S')
  {
    Serial.println("Stop");
    Stop();
    flag.back_light_Flag=2;
  }
  else if (value == 'x' || value =='S')
  {
    Serial.println("Stop");
    Stop();
    flag.back_light_Flag=0;
  }
  //else if (value == 'U')
  else if (value == '1')
  {
    //backlight
    digitalWrite(back_light, HIGH);  
    flag.back_light_Flag=1;
    Serial.println("light");
  }
  else if (value == 'u')
  {

    digitalWrite(back_light, LOW);
    flag.back_light_Flag=0;
    Serial.println("lightoff");
  }
  else if (value == 'W')
  {
    digitalWrite(front_light, HIGH);  
    flag.front_light_Flag=1;
    //Serial.println("light");
  }
  else if (value == 'w')
  {
    digitalWrite(front_light, LOW);
    flag.front_light_Flag=0;
    Serial.println("lightoff");
  }
  else if (value == 'V')
  {
    digitalWrite(horn_Buzz, HIGH);
    Serial.println("horn");
  }
  else if (value == 'v')
  {
    digitalWrite(horn_Buzz, LOW);
    Serial.println("horn");
  }
  
    

}
////-------------------BT-------------
//void BTprocessCommand(String data)
//{
//     if (data =="FS")
//  {
//    Serial.println("Forward");
//    Forward();
//  }
//  else if (data == "LS")
//  {
//    Serial.println("Left");
//    Left();
//  }
//  else if (data == "RS")
//  {
//    Serial.println("Right");
//    Right();
//  }
//  else if (data == "BS")
//  {
//    Serial.println("Reverse");
//    Reverse();
//  }
//  else if (data == "X")
//  {
//    Serial.println("Stop");
//    Stop();
//    flag.back_light_Flag=2;
//  }
//  else if (data == "x")
//  {
//    Serial.println("Stop");
//    Stop();
//    flag.back_light_Flag=0;
//  }
//  else if (data == "U")
//  {
//    //backlight
//    digitalWrite(back_light, HIGH);  
//    flag.back_light_Flag=1;
//    Serial.println("light");
//  }
//  else if (data == "u")
//  {
//
//    digitalWrite(back_light, LOW);
//    flag.back_light_Flag=0;
//    Serial.println("lightoff");
//  }
//  else if (data == "W")
//  {
//    digitalWrite(front_light, HIGH);  
//    flag.front_light_Flag=1;
//    //Serial.println("light");
//  }
//  else if (data == "w")
//  {
//    digitalWrite(front_light, LOW);
//    flag.front_light_Flag=0;
//    Serial.println("lightoff");
//  }
//  else if (data == "V")
//  {
//    digitalWrite(horn_Buzz, HIGH);
//    Serial.println("horn");
//  }
//  else if (data == "v")
//  {
//    digitalWrite(horn_Buzz, LOW);
//    Serial.println("horn");
//  }}
  
  
