int analogInput = 1;
float voltage = 0.00;
int analogVal = 0;
int voltageFreq = 1000;

String state1 = "volFreq";

void setup() {
  pinMode(analogInput, INPUT); //assigning the input port
  Serial.begin(9600); //BaudRate
}

void serialEvent() {
  Serial.flush();

  String dataFromRP = Serial.readStringUntil('\n');

  int index = dataFromRP.indexOf(':');
  String dataFromRP_topic = dataFromRP.substring(0, index);

  if (state1.equals(dataFromRP_topic)) {
    int dataFromRP_length = dataFromRP.length();
    if (10 < dataFromRP.substring(index+1, dataFromRP_length).toInt()) {
      voltageFreq = dataFromRP.substring(index+1, dataFromRP_length).toInt(); 
    } else {
      Serial.println("Requested frequency of publiching voltage is too small (<10ms)");
    }
  } 
}

void loop() {
  
  analogVal = analogRead(analogInput);//reads the analog input
  voltage = (analogVal * 5.00) / 1024.00; // formula for calculating voltage out i.e. V+, here 5.00
  
  Serial.print("voltage:");
  Serial.println(voltage);
  
  delay(voltageFreq);
}
