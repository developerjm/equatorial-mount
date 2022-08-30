
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// motor control pin
const int motorDirPin = 10; // L298 Input 1
const int motorPWMPin = 9; // L298 Input 2


#define trig 13
#define echo 12

#define led_motor 4
#define led_time 2
#define led_manual 8
#define sw 7


// encoder pin
const int encoderPinA = 2;
const int encoderPinB = 3;

int encoderPos = 0;
const float ratio = 360./600./26.;

unsigned long ret_time;
unsigned long time_interval;

int flag_sonic = 0;
int flag_motor = 0;
int flag_mode = 0;
int flag_joystick = 0;

int motor_pwm = 0;

int runningtime = 0;

int joystick = 0;

// P control
float Kp = 30;
float targetDeg = 720;

void doEncoderA(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}

void doMotor(bool dir, int vel){
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, dir?(255 - vel):vel);
}

void setup() {
  lcd.init();

  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("LCD Init");

  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT);
  
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(sw, INPUT);
  pinMode(led_motor, OUTPUT);
  pinMode(led_time, OUTPUT);
  pinMode(led_manual, OUTPUT);
 
  Serial.begin(115200);
}

void loop() {
  long duration, distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration*17/1000;

  joystick = analogRead(A0);

  if(digitalRead(sw) == HIGH) {
    flag_mode = !flag_mode;
  }

  Serial.println(duration ); //초음파가 반사되어 돌아오는 시간을 보여줍니다.

  Serial.print("\nDIstance : ");

  Serial.print(distance); //측정된 물체로부터 거리값(cm값)을 보여줍니다.

  Serial.println(" Cm");
  
  Serial.print("x val: ");
  Serial.println(joystick);

  if (flag_mode == 0) {
    Serial.println("switch val: time mode");
    lcd.setCursor(0,0);
    lcd.print("Set Running Time:           ");

//    digitalWrite(led_time, HIGH);
//    digitalWrite(led_manual, LOW);
    if(joystick > 980 && flag_joystick == 0) {
      flag_joystick = 1;
      runningtime += 20;
      Serial.print("running time : ");
      Serial.println(runningtime);

    }
    if(joystick < 20 && flag_joystick == 0) {
      flag_joystick = 1;
      runningtime -= 20;
      Serial.print("running time : ");
      Serial.println(runningtime);
    }

    if(runningtime < 0) {
      runningtime = 0;
    }
    
    lcd.setCursor(0,1);
    lcd.print(String(runningtime) + "min         ");
    
    flag_joystick = 0;
    
  }else {
    Serial.println("switch val: manual mode");
    lcd.setCursor(0,0);
    lcd.print("Manual Mode:    ");
//    digitalWrite(led_time, LOW);
//    digitalWrite(led_manual, HIGH);
    motor_pwm = (joystick - 504)/2;
    Serial.print("motor pwm : ");
    Serial.println(motor_pwm);
    lcd.setCursor(0,1);
    lcd.print("Rot Speed: " + String(int((motor_pwm*100)/255)) + "%    ");
    if( motor_pwm < 10 && motor_pwm > -10) {
      motor_pwm = 0;
    }
    if( motor_pwm > 0 ){
      doMotor(HIGH, motor_pwm);
      delay(100);
    } else {
      doMotor(LOW, (-1)*motor_pwm);
      delay(100);
    }
  }

  if (flag_sonic==0 && distance<=5) {
    ret_time=millis();
    flag_sonic=1;
    }
  if (distance>8) {
    flag_sonic=0;
    }
  time_interval=millis()-ret_time;
  if (time_interval>=3000&&flag_sonic==1) {
    flag_motor = 1;
  }
  delay(200);
  
  
  
  if (flag_motor == 1) {
    digitalWrite(led_motor, HIGH);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Star Tracking...      ");
    digitalWrite(led_time, LOW);
    digitalWrite(led_manual, LOW);
    long runningSec = runningtime * 60;
    long remTime = 0;
    long autoStartTime = millis();
    long remMin = 0;
    long remSec = 0;

    while (true) {
      float motorDeg = float(encoderPos)*ratio;
     
      float error = targetDeg - motorDeg;
      float control = Kp*error;
    
      doMotor( (control>=0)?HIGH:LOW, min(abs(control), 255));
    
      Serial.print("encoderPos : ");
      Serial.print(encoderPos);
      Serial.print("   motorDeg : ");
      Serial.print(float(encoderPos)*ratio);
      Serial.print("   error : ");
      Serial.print(error);
      Serial.print("    control : ");
      Serial.print(control);
      Serial.print("    motorVel : ");
      Serial.println(min(abs(control), 255));
      
//      remTime = runningSec - (millis() - autoStartTime)/1000;
//      remMin = remTime/60;
//      remSec = remTime - remMin*60;
//      lcd.setCursor(0,1);
//      lcd.print(String(remMin) + "min " + String(remSec) + "sec      ");
      }
   }
}
