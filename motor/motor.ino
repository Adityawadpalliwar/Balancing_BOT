const int inputPin1  = A2;    
const int inputPin2  = A3;    
const int buzz       = A1;    
const int inputPin3  = 9;     
const int inputPin4  = 4;     
int EN1 = 6;                  
int EN2 = 5;                  

void setup() {
   pinMode(buzz, OUTPUT); 
   pinMode(EN1, OUTPUT);   
   pinMode(EN2, OUTPUT);   
   pinMode(inputPin1, OUTPUT);
   pinMode(inputPin2, OUTPUT);
   pinMode(inputPin3, OUTPUT);
   pinMode(inputPin4, OUTPUT);

   analogWrite(EN1, 100);      
   analogWrite(EN2, 100);
}

void loop() {
   delay(500);
   tone(buzz, 2000);
   delay(100);
   noTone(buzz);

   digitalWrite(inputPin1, HIGH);
   digitalWrite(inputPin2, LOW);
   digitalWrite(inputPin3, HIGH);
   digitalWrite(inputPin4, LOW);
   delay(200);

   digitalWrite(inputPin1, LOW);
   digitalWrite(inputPin2, HIGH);
   digitalWrite(inputPin3, LOW);
   digitalWrite(inputPin4, HIGH); 
   delay(200);
}
