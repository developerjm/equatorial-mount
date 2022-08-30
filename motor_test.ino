// motor control pin
const int motorDirPin = 10; // L298 Input 1
const int motorPWMPin = 9; // L298 Input 2

// encoder pin
const int encoderPinA = 2;
const int encoderPinB = 3;

long elapsedTime = 0;

long encoderPos = 0;
const float ratio = 360./600./26.;

// P control
float Kp = 10;
float Ki = 10;
float targetDeg = 0;

void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}

void doMotor(bool dir, int vel){
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, dir?(255 - vel):vel);
}

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT);

  delay(5000);

  Serial.begin(115200);
}

long previousTime = millis();
float Ierror = 0;

void loop() {
  
  elapsedTime = millis() - previousTime;
  previousTime = millis();
//  targetDeg = targetDeg + 0.004167*0.001*elapsedTime;
  targetDeg = targetDeg + 0.1*0.001*elapsedTime;

  float motorDeg = float(encoderPos)*ratio;
  
  float Perror = targetDeg - motorDeg;
  Ierror = Ierror + Perror*elapsedTime*0.001;
  float control = Kp*Perror + Ki*Ierror;

  doMotor( (control>=0)?HIGH:LOW, min(abs(control), 255));

  Serial.print("encoderPos : ");
  Serial.print(encoderPos);
  Serial.print("   TargetDeg : ");
  Serial.print(targetDeg);
  Serial.print("   motorDeg : ");
  Serial.print(float(encoderPos)*ratio);
  Serial.print("   error : ");
  Serial.print(Perror);
  Serial.print("    control : ");
  Serial.print(control);
  Serial.print("    motorVel : ");
  Serial.println(min(abs(control), 255));

  
}
