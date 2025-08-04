void setup() 
{
  Serial.begin(9600);


}

void loop() 
{
  int data = random(0, 1000);

  Serial.pritn("data = ");
  Serial.println(data);

  delay(2500);
}
 