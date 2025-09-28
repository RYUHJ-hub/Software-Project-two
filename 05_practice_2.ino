#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); //Initialize serial port
  while (!Serial) {
    ; //wait for serial port to connect.
  }
  count=toggle=0;
  digitalWrite(PIN_LED, HIGH); 
}

void loop() {
  toggle=1;
  digitalWrite(PIN_LED, toggle ? LOW : HIGH);
  delay(1000);

  for (int i=0; i<5; i++) {
    toggle=toggle_state(toggle); //toggle LED value.
    digitalWrite(PIN_LED, toggle ? LOW : HIGH); //update LED status.
    delay(100);
    toggle=toggle_state(toggle);
    digitalWrite(PIN_LED, toggle ? LOW : HIGH);
    delay(100);
  }
  
  digitalWrite(PIN_LED, HIGH);
  toggle=0;
  while(1) {
    ;
  }
}

int toggle_state(int state) {
  return !state;
}