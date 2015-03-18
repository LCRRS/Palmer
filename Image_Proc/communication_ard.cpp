int led1 = 8;
int led2 = 9;
int led3 = 10;
int led4 = 11;
int led5 = 12;
int new_val;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
    String added_val;
    if (Serial.available() > 0)
    {
        int new_val = Serial.parseInt();
        Serial.println(new_val);

            if (new_val == 0)
            {
                digitalWrite(led1,LOW);
                digitalWrite(led2,LOW);
                digitalWrite(led3,LOW);
                digitalWrite(led4,LOW);
                digitalWrite(led5,LOW);
            }
            if (new_val != 0)
            {
                if (new_val >= 512)
                {
                    digitalWrite(led1, HIGH);
                    digitalWrite(led2,LOW);
                    digitalWrite(led3,LOW);
                    digitalWrite(led4,LOW);
                    digitalWrite(led5,LOW);
                }
                if (new_val >= 384 && new_val < 512)
                {
                    digitalWrite(led2, HIGH);
                    digitalWrite(led1,LOW);
                    digitalWrite(led3,LOW);
                    digitalWrite(led4,LOW);
                    digitalWrite(led5,LOW);
                }
                if (new_val >= 256 && new_val < 384)
                {
                    digitalWrite(led3, HIGH);
                    digitalWrite(led2,LOW);
                    digitalWrite(led1,LOW);
                    digitalWrite(led4,LOW);
                    digitalWrite(led5,LOW);
                }
                if (new_val >= 128 && new_val < 256)
                {
                    digitalWrite(led4, HIGH);
                    digitalWrite(led2,LOW);
                    digitalWrite(led3,LOW);
                    digitalWrite(led1,LOW);
                    digitalWrite(led5,LOW);
                }
                if (new_val < 128)
                {
                    digitalWrite(led5, HIGH);
                    digitalWrite(led2,LOW);
                    digitalWrite(led3,LOW);
                    digitalWrite(led4,LOW);
                    digitalWrite(led1,LOW);
                }
            }

                // wait for a second
    }
}
