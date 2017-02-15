#include <Servo.h>

//Servos
Servo servos[6];
//Pseudo read pins
byte pinInsPS[6] = {A8,A9,A10,A7,A11};
//Sensor input pins
byte pinIns[6] = {A0,A1,A2,A3,A4,A5}; // analog pin used to connect the potentiometer
//Servo output pins
byte pinOuts[6] = {10,9,8,7,6,5};
//Value reading
int vals[6]; // array to record the readings from the analog pins
int outVals[6];
int outVals_old[1];
int curr_outVals[6];
int pre_outVals[6];

//Safety limits
int vals_max[6] = {795,1003,795,1019,795,890};
int vals_min[6] = {265,150,0,282,0,675};
//Loop Control
byte enterFlag = 0;//at the beginning it's 0, and then changes to 1

//Modes
byte modeFlag = 0;
byte buttonCount = 0;
byte buttonPressed = 0;

const byte mode_manual = 0;
const byte mode_learn = 1;
const byte mode_replay = 2;

//Switch & Button pin numbers:
const byte switchPin_M = 41;
const byte switchPin_L = 43;
const byte buttonPin = 39;
const byte graspPin = 38;    

//Times
long _previousMillis = 0;
long _currentMillis = 0;
byte _waitMillis = 30;

//Mems
const int memLimit = 667;
byte memArray[6][680];
int memCount = 0;
int rplyCount = 0;
//

void setup() {
  Serial.begin(115200);
  // initialize the switch & button pins
  pinMode(buttonPin, INPUT);
  pinMode(graspPin, INPUT);
  pinMode(switchPin_M, INPUT);
  pinMode(switchPin_L, INPUT);

  //attach servos
  servos[0].attach(pinOuts[0], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  servos[1].attach(pinOuts[1], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  servos[2].attach(pinOuts[2], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  servos[3].attach(pinOuts[3], MIN_PULSE_WIDTH, 2600);//MAX_PULSE_WIDTH);
  servos[4].attach(pinOuts[4], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  servos[5].attach(pinOuts[5], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

bool flag_ctrl = false;

void loop() {
  _currentMillis = millis();
  
  if(_currentMillis-_previousMillis > _waitMillis){    
    if(readSerial()==0){
      _previousMillis = _currentMillis;
    }
  }
//  flag_ctrl = false;
  if(!flag_ctrl){
    readControls();
    if(modeFlag == mode_manual){
      //Serial.println(0);
      if(_currentMillis-_previousMillis > _waitMillis){
        readSensors();
        moveServos();
        _previousMillis = _currentMillis;
      }
    }else if(modeFlag == mode_learn){
      if(buttonCount == 0){//learning starts
        //Serial.println(1);
        if(_currentMillis-_previousMillis > _waitMillis && memCount < memLimit){    
          //Serial.println(memCount); 
          readSensors();
          writeToArray();//also moves servos
          memCount++;
          _previousMillis = _currentMillis;
        }
      }else if(buttonCount == 1){
        //Serial.println(2);
        //do nothing
      }else{
        modeFlag = mode_replay;
        buttonCount = 0;
        rplyCount = 0;
      }
    }else if(modeFlag == mode_replay){
      if(rplyCount < memCount){
        //Serial.println(3);
        if(_currentMillis-_previousMillis > _waitMillis){
          readArray();
          moveServos_angle();
          rplyCount++;
          _previousMillis = _currentMillis;
        }
      }else{
        //Serial.println(4);
        rplyCount = 0;
        delay(2000);
      }
    }
  }
}

void readControls(){
  if(digitalRead(switchPin_M)==HIGH){
    modeFlag = 0;
    buttonCount = 0;
    memCount = 0;
  }else if(digitalRead(switchPin_L)==HIGH){
    if(modeFlag != mode_replay){
      modeFlag = 1;
      //memCount = 0;
    }
  }

  if(modeFlag == mode_learn){
    if(digitalRead(buttonPin)==HIGH){
      buttonPressed = 1;
    }else{    
      if(buttonPressed){
        buttonCount++;
        buttonPressed = 0;
      }
    }
  }
}

void readArray(){
  for(int i = 0; i<6; i++){
    outVals[i] = memArray[i][rplyCount];
  }  
}

void writeToArray(){
  memArray[0][memCount] = moveServo(0,1);
  memArray[1][memCount] = moveServo(1,-1);
  memArray[2][memCount] = moveServo(2,-1);
  memArray[3][memCount] = moveServo(3,-1);
  memArray[4][memCount] = moveServo(4,-1);
  memArray[5][memCount] = moveServo(5,-1);  

//  Serial.print(memArray[0][memCount]);
//  Serial.print("  ");
//  Serial.print(memArray[1][memCount]);
//  Serial.print("  ");
//  Serial.print(memArray[2][memCount]);
//  Serial.print("  ");
//  Serial.print(memArray[3][memCount]);
//  Serial.print("  ");
//  Serial.print(memArray[4][memCount]);
//  Serial.print("  ");
//  Serial.print(memArray[5][memCount]);
//  Serial.println();
}

void moveServos(){
  moveServo(0,1);
  moveServo(1,-1);
  moveServo(2,-1);
  moveServo(3,-1);
  moveServo(4,-1);
  moveServo(5,-1);

  
  Serial.print(outVals[0]);
  Serial.print(" ");
  Serial.print(outVals[1]);
  Serial.print(" ");
  Serial.print(outVals[2]);
  Serial.print(" ");
  Serial.print(outVals[3]);
  Serial.print(" ");
  Serial.print(outVals[4]);
  Serial.print(" ");
  Serial.print(outVals[5]);
  Serial.print("\n");
}

void moveServos_angle(){
  for(int i = 0; i<6; i++){
    servos[i].write(outVals[i]);
  }
}

int angles[6] = {90,66,0,118,92,118};
bool doneReading = false;
int ct_angle = 0;
int ct_num_o_angles = 0;
char str_buffer_angle [4];

int readSerial(){
  if (!Serial.available()){
    if(!flag_ctrl){
      return 1;
    }
  }else{
    
  }
  while(Serial.available()){          
    char ch = Serial.read();
    if(ch == 'c'){
      flag_ctrl = true;
      return 1;
    }else if(ch == 'd'){
      flag_ctrl = false;
      return 1;
    }
    if(flag_ctrl){
      if(ch == ';'){
        char str_angle_temp[4]={'\0','\0','\0','\0'}; 
        for (int i = 0; i< ct_angle; i++){
          str_angle_temp[i] = str_buffer_angle[i];
        }
        angles[ct_num_o_angles++] = atoi(str_angle_temp);
        //Serial.println(ct_angle);//In Control mode this has to be commented out otherwise it will introduce a delay after running for a while
        //Serial.println(str_angle_temp);
        ct_angle = 0;
        ct_num_o_angles = 0;
        doneReading = true;
        break;
      }
      else if(ch != ','){
        str_buffer_angle[ct_angle++] = ch;
      }else{
        char str_angle_temp[4]={'\0','\0','\0','\0'}; 
        for (int i = 0; i< ct_angle; i++){
          str_angle_temp[i] = str_buffer_angle[i];
        }
        angles[ct_num_o_angles++] = atoi(str_angle_temp);
        //Serial.println(ct_angle);
        //Serial.println(str_angle_temp);
        ct_angle = 0;
      }
    }
  }
  if(flag_ctrl){
    if(doneReading){//update angles according to serial data
      for(int i = 0; i<5; i++){
        vals[i] = angles[i];
      }
  
      servos[0].write(angles[0]);
      servos[1].write(angles[1]);
      servos[2].write(angles[2]);
      servos[3].write(angles[3]);
      servos[4].write(angles[4]);
    }
  
    servos[0].write(angles[0]);
    servos[1].write(angles[1]);
    servos[2].write(angles[2]);
    servos[3].write(angles[3]);
    servos[4].write(angles[4]);
    servos[5].write(angles[5]);
  
    return 0;
  }
  
//  for(int i = 0; i<5; i++){
//    vals[i] = analogRead(pinIns[i]);
//  }  
//
//  if(digitalRead(graspPin) ==LOW){
//    if(vals[5]<1024){
//      if((vals[5]+51)<1024){
//        vals[5] = vals[5] + 51;
//      }else{
//        vals[5] = 1023;
//      }
//    }
//  }else{
//    if(vals[5]>=0){
//      if((vals[5]-51)>=0){
//        vals[5] = vals[5] - 51;
//      }else{
//        vals[5] = 0;
//      }
//    }
//  }  
}

void readSensors(){
  analogRead(pinInsPS[2]);  //Pseudo Reads to stablize the signals
  analogRead(pinInsPS[1]);  //Pseudo Reads
  analogRead(pinInsPS[0]);  //Pseudo Reads
  
  for(int i = 0; i<5; i++){
    vals[i] = analogRead(pinIns[i]);
  }  

  if(digitalRead(graspPin) ==LOW){
    if(vals[5]<1024){
      if((vals[5]+51)<1024){
        vals[5] = vals[5] + 51;
      }else{
        vals[5] = 1023;
      }
    }
  }else{
    if(vals[5]>=0){
      if((vals[5]-51)>=0){
        vals[5] = vals[5] - 51;
      }else{
        vals[5] = 0;
      }
    }
  }
  
  analogRead(pinInsPS[3]); //Pseudo Reads
  analogRead(pinInsPS[4]); //Pseudo Reads
}

int moveServo(int n, int p){
  if(p==1&&n!=5){
    if (vals[n] > vals_max[n]) {
      curr_outVals[n] = map(vals_max[n], vals_min[n], vals_max[n], 0, 360);
      if(abs(curr_outVals[n]-pre_outVals[n])>=2){
        outVals[n] = curr_outVals[n]/2;
        pre_outVals[n] = curr_outVals[n];
      }
//      outVals[n] = map(vals_max[n], vals_min[n], vals_max[n], 0, 180);
    } else if (vals[n] < vals_min[n]) {
      curr_outVals[n] = map(vals_min[n], vals_min[n], vals_max[n], 0, 360);
      if(abs(curr_outVals[n]-pre_outVals[n])>=2){
        outVals[n] = curr_outVals[n]/2;
        pre_outVals[n] = curr_outVals[n];
      }
    } else {
      curr_outVals[n] = map(vals[n], vals_min[n], vals_max[n], 0, 360);
      if(abs(curr_outVals[n]-pre_outVals[n])>=2){
        outVals[n] = curr_outVals[n]/2;
        pre_outVals[n] = curr_outVals[n];
      }
    }      
  }else if(p==-1&&n!=5){//flip max-min to map servo with sensor
    if (vals[n] > vals_max[n]) {
      curr_outVals[n] = map(vals_max[n], vals_max[n], vals_min[n], 0, 360);
      if(abs(curr_outVals[n]-pre_outVals[n])>=2){
        outVals[n] = curr_outVals[n]/2;
        pre_outVals[n] = curr_outVals[n];
      }
    } else if (vals[n] < vals_min[n]) {
      curr_outVals[n] = map(vals_min[n], vals_max[n], vals_min[n], 0, 360);
      if(abs(curr_outVals[n]-pre_outVals[n])>=2){
        outVals[n] = curr_outVals[n]/2;
        pre_outVals[n] = curr_outVals[n];
      }
    } else {
      curr_outVals[n] = map(vals[n], vals_max[n], vals_min[n], 0, 360);
      if(abs(curr_outVals[n]-pre_outVals[n])>=2){
        outVals[n] = curr_outVals[n]/2;
        pre_outVals[n] = curr_outVals[n];
      }
    }         
  }else if(n==5){
      if (vals[n] > vals_max[n]) {
        curr_outVals[n] = map(vals_max[n], 0, 1023, 0, 360);
        if(abs(curr_outVals[n]-pre_outVals[n])>=2){
          outVals[n] = curr_outVals[n]/2;
          pre_outVals[n] = curr_outVals[n];
        }
        
      } else if (vals[n] < vals_min[n]) {
        curr_outVals[n] = map(vals_min[n], 0, 1023, 0, 360);
        if(abs(curr_outVals[n]-pre_outVals[n])>=2){
          outVals[n] = curr_outVals[n]/2;
          pre_outVals[n] = curr_outVals[n];
        }
      } else {
        curr_outVals[n] = map(vals[n], 0, 1023, 0, 360);
        if(abs(curr_outVals[n]-pre_outVals[n])>=2){
          outVals[n] = curr_outVals[n]/2;
          pre_outVals[n] = curr_outVals[n];
        }
      }
  }
  
  
  servos[n].write(outVals[n]);

  
  
//  if(n==0){
//    if(enterFlag==0){
//      enterFlag++;
//      outVals_old[n] = outVals[n];
//      servos[n].write(outVals[n]);        
//    }    
//    if(abs(outVals_old[n]-outVals[n])>1){//Compensate for analogread() errors
//      servos[n].write(outVals[n]);
//      outVals_old[n] = outVals[n];
//    } 
//  }else{
//    servos[n].write(outVals[n]);
//  } 
  return(outVals[n]);
}

int lowResBy8(int val) {
  val /= 8;
  val *= 8;
  return val;
}

int lowResBy16(int val) {
  val /= 16;
  val *= 16;
  return val;
}

int lowResBy100(int val) {
  val /= 100;
  val *= 100;
  return val;
}

