void setup() {
  Serial.begin(9600);

}

int pi_data[4];

void loop() {

    int x_axis = 0;
    int y_axis = 0;
    
    if (Serial.find("s")) {
      x_axis = Serial.parseInt(); // parses numeric characters before the comma
      y_axis = Serial.parseInt();// parses numeric characters after the comma
      
      // print the results back to the sender:
      Serial.print("x_axis: " );
      Serial.print(x_axis);
      Serial.print(" at y_axis");
      Serial.println(y_axis);
    }
}