void setup(){
  Serial.begin(9600);
}

void loop(){
  int a;
  a = Serial.read();
  if (a ==1){
    Serial.print("OKAY!!!");
  }
  else{
    Serial.print("NOOO!!!");
  }
  delay(1000);
}
