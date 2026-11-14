const int ledPin = 7;

void setup()
{
	pinMode(ledPin, OUTPUT);
}

void loop()
{
	digitalWrite(ledPin, HIGH);
  	delay(1000); // 1 second
  
  	digitalWrite(ledPin, LOW);
  	delay(1000); // 1 second
}