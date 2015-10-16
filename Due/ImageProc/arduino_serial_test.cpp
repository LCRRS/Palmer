void setup() {
  Serial.begin(9600);

}

void loop() {

    if(Serial.available()){
        int new_val = Serial.parseInt();
        Serial.println(new_val);
    }
}