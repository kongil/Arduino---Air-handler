                  
int accelbutton = 0;                        
int breakbutton = 1;
int boosterbutton = 2;
int RX_input = 0;
int TX_output = 1;
int Keyboard_Right = 215, Keyboard_Left = 216, Keyboard_Up = 218, Keyboard_Down = 128;
int reverse_int = 0;
int timer = 0;
int button = 0;
void setup(){ 
  Serial.begin(9600);
  Keyboard.begin();
  pinMode(2,INPUT);
  pinMode(3,OUTPUT);
}
void read_handle();
void read_pedal(int accel_, int break_, int booster_);
void reverse();
void loop(){
  int LorR;
  int accel_stat, break_stat, booster_stat;
  long random_;
  read_handle();
  accel_stat = analogRead(accelbutton);
  break_stat = analogRead(breakbutton);
  booster_stat = analogRead(boosterbutton);
  read_pedal(accel_stat, break_stat, booster_stat);
  button = digitalRead(2);
  if(button == 1){
  timer = timer + 100;
  if(timer > 5000 && timer < 10000){
    random_ = random(1,100);
    if(random_ < 10 || timer == 10000){
      reverse();
    }
  }
  }
  if(button == 0){
    digitalWrite(3,LOW);
    reverse_int = 0;
  }
  if(reverse_int == 1){
    digitalWrite(3,HIGH);
  }
  if(reverse_int == 0){
    digitalWrite(3,LOW);
  }
  Serial.println(reverse_int);
  delay(100);
}                                                              
void read_handle(){
  int right = Keyboard_Right;
  int left = Keyboard_Left;
  if (reverse_int == 1){
    right = right + 1;
    left = left - 1;
  }
  if(analogRead(3) > 500){
    Keyboard.press(right);
  }
  else {
    Keyboard.release(right);
  }
  if(analogRead(4) > 500){
    Keyboard.press(left);
  }
  else{
    Keyboard.release(left);
  }
}             
void read_pedal(int a, int b, int c){
  int up = Keyboard_Up;
  int down = Keyboard_Down;
  if(reverse_int == 1){
    up = up - 90;
    down = down + 90;
  }
  if(a > 30) {
    Serial.println(a);    
    Keyboard.press(Keyboard_Up);
    //Serial.print("Accel");
  }
  else{
    Keyboard.release(Keyboard_Up);
  }                                                                                 
  if(b > 30){
    Keyboard.release(Keyboard_Up);
    Keyboard.press(Keyboard_Down);
    //Serial.print("break!");
  }
  else{
    Keyboard.release(Keyboard_Down);
  }
  if(c > 30){
    Keyboard.press(' ');
    Keyboard.release(Keyboard_Down);
    //Serial.print("Let's Boost!");
  }
  else{
    Keyboard.release(' ');
  }
}
void reverse(){
  if(reverse_int == 0){
    reverse_int = 1;
    return;
  }
  if(reverse_int == 1){
    reverse_int = 0;
  }
  timer = 0;
}
  
