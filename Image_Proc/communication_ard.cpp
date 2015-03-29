
float pi_data[5];

void setup() {
  Serial.begin(115200);
}

void serial_read() {
    if (Serial.parseInt() == 314159265)
    {
        for (int i = 0; i < 4; i++)
        {
            float new_val = (float(Serial.parseInt()))/100.0;
            pi_data[i] = new_val;

        }
    }
}

void loop() {

    serial_read();

    Serial.print("  Horizontal:   ");
    Serial.print(pi_data[1]);
    Serial.print("  Vertical:     ");
    Serial.print(pi_data[2]);
    Serial.print("  Distance:     ");
    Serial.print(pi_data[3]);
    Serial.print("  Mode:     ");
    Serial.println(pi_data[4]);

}
