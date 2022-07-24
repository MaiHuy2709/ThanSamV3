// Motor Variables
//int ENA = 6;              //Motor phải
int motorInput1 = 5;
int motorInput2 = 4;
int motorInput3 = 6;
int motorInput4 = 7;
//int ENB = 5;             //Motor trái

void setup()
{
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  //pinMode(ENA, OUTPUT);
  //pinMode(ENB, OUTPUT);

  
  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(500);
}
void loop()
{
  analogWrite(motorInput1, 100);  //tiến
  digitalWrite(motorInput2, LOW);
  analogWrite(motorInput3, 80);
  digitalWrite(motorInput4, HIGH);
  delay(2000);
 // analogWrite(motorInput1, 150);
  //digitalWrite(motorInput2, HIGH);//lùi
  //analogWrite(motorInput3, 150);
  digitalWrite(motorInput4, HIGH);
  delay(2000);
}
