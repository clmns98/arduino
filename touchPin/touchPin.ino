//https://randomnerdtutorials.com/esp32-touch-pins-arduino-ide/

#define THEPIN 32
int temp=0;
int counter=0;

void setup() {
    Serial.begin(115200);
    Serial.println("Touch sensor tutorial ESP32");

}

void loop() {
        
    Serial.print("Touch value is = ");
    for(int i=0; i<41; i++)
    {
      temp += touchRead(THEPIN);
    }
    temp /= 40;
    Serial.println(temp);
    if(temp<30)
    {
      counter++;
    }
    if(counter == 4)
    {
      Serial.println("SUCCESS!!!");
      counter = 0;
      delay(5000);
    }
    
    delay(500);

}
