void setup()
{
  Serial.begin(9600);
  pinMode(2,OUTPUT); 
  pinMode(3,OUTPUT);
}
int button = 0;
int a, b, ran;
void loop()
{
    a = analogRead(0);
    ran = analogRead(1) ; 
    int b;
    
    b = Serial.read();                                                                                      

    Serial.println(ran);
  if (a>30 && ran < 500){
    Serial.write(1);
  }
  if (a < 30 && ran > 500 ){
    Serial.write(2);
  }
  if (a > 30 && ran > 500){
    Serial.write(2);
  }
  
    if(b == -1){
      digitalWrite(2,HIGH);
      digitalWrite(3,LOW);
    }
    if(b == 1){
      digitalWrite(3,HIGH);
      digitalWrite(2,LOW);
    }
    else{
      digitalWrite(2,LOW);
      digitalWrite(3,LOW);
    }  
  delay(100);
 }
