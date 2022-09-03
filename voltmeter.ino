int analogInput = 1;
int analogVal = 0;
float voltage = 0.00;
int voltageFreq = 1000;

void setup() {
  pinMode(analogInput, INPUT); //assigning the input port
  Serial.begin(9600); //BaudRate
}

void loop() {
  analogVal = analogRead(analogInput);//reads the analog input
  voltage = (analogVal * 5.00) / 1024.00; // formula for calculating voltage out i.e. V+, here 5.00
  
  Serial.print("voltage:");
  Serial.println(voltage);
  
  delay(voltageFreq);
}
