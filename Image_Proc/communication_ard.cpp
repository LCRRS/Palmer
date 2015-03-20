int led1 = 8;
int led2 = 9;
int led3 = 10;
int led4 = 11;
int led5 = 12;
int new_val;

void setup() {
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  Serial.begin(9600);
}

int pi_data[3];

void serial_read() {
    if (Serial.parseInt() == 31415)
    {
        for (int i; i < 3; i++)
        {
            int new_val = Serial.parseInt();
            pi_data[i] = new_val;

        }
    }

    Serial.print("New set:  ");
    for (int z; z < 3; z++)
    {
        Serial.print(pi_data[z]);
        Serial.print("    ");
    }
    Serial.println();
}

void loop() {
    serial_read();


    if (pi_data[1] == 0)
    {
        digitalWrite(led1,LOW);
        digitalWrite(led2,LOW);
        digitalWrite(led3,LOW);
        digitalWrite(led4,LOW);
        digitalWrite(led5,LOW);
    }
    if (pi_data[1] != 0)
    {
        if (pi_data[1] >= 512)
        {
            digitalWrite(led1, HIGH);
            digitalWrite(led2,LOW);
            digitalWrite(led3,LOW);
            digitalWrite(led4,LOW);
            digitalWrite(led5,LOW);
        }
        if (pi_data[1] >= 384 && pi_data[1] < 512)
        {
            digitalWrite(led2, HIGH);
            digitalWrite(led1,LOW);
            digitalWrite(led3,LOW);
            digitalWrite(led4,LOW);
            digitalWrite(led5,LOW);
        }
        if (pi_data[1] >= 256 && pi_data[1] < 384)
        {
            digitalWrite(led3, HIGH);
            digitalWrite(led2,LOW);
            digitalWrite(led1,LOW);
            digitalWrite(led4,LOW);
            digitalWrite(led5,LOW);
        }
        if (pi_data[1] >= 128 && pi_data[1] < 256)
        {
            digitalWrite(led4, HIGH);
            digitalWrite(led2,LOW);
            digitalWrite(led3,LOW);
            digitalWrite(led1,LOW);
            digitalWrite(led5,LOW);
        }
        if (pi_data[1] < 128)
        {
            digitalWrite(led5, HIGH);
            digitalWrite(led2,LOW);
            digitalWrite(led3,LOW);
            digitalWrite(led4,LOW);
            digitalWrite(led1,LOW);
        }
    }

}
