#include <Wire.h>
#include <AS5600.h>
#include <Servo.h>
#include <Average.h>

Average<float> ave(5);

Servo myservo;
AMS_5600 ams5600;

int r_en = 9;
int l_en = 10;
int r_pwm = 13;
int l_pwm = 12;
int ams_dir_pin = 19;
int amp_pwr_pin = 6;

int current_power = 0;
int requested_power = 0;

float angle = 0;
float last_angle = 0;
float absolute_angle = 0;
int revolutions = 0;

float elapsed_time = 0;
float time = 0;
float last_time = 0;
float rad_to_deg = 180 / 3.141592654;

float PID = 0;
float error = 0;
float previous_error = 0;
float desired_angle = 180;

float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

// double kp = 5.5; //3  // 2.1
// double ki = 0.000008; //0.002 // 0.00008
// double kd = 1.12; //0.4 // 0.8

// works perfectly for pwm test board (code_robot drive)
double p_val = 2.9;
double i_val = 0.006;
double d_val = 0.6;

/*
  double kp = 4.1;
  double ki = 0.0005;
  double kd = 1.1;
*/

void setup() {
  pinMode(r_en, OUTPUT);
  pinMode(l_en, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(amp_pwr_pin, OUTPUT);
  pinMode(ams_dir_pin, OUTPUT);
  digitalWrite(ams_dir_pin, HIGH);
  digitalWrite(amp_pwr_pin, HIGH);
  myservo.attach(3);

  Serial.begin(115200);
  Serial.println("servo_error , desired_servo_angle, actual_servo_angle;");

  Wire.begin();
  if (ams5600.detectMagnet() == 0 ) {
    while (1) {
      if (ams5600.detectMagnet() == 1 ) {
        // Serial.print("Current Magnitude: ");
        // Serial.println(ams5600.getMagnitude());
        break;
      }
      else {
        Serial.println("Can not detect magnet");
      }
      delay(1000);
    }
  }

  time = millis();
  enable();
}

void loop() {
  time = millis();
  elapsed_time = (time - last_time) / 1000;

  angle = ams5600.getRawAngle() * 0.087;
  if (last_angle - angle > 300 ) {
    revolutions += 1;
  }
  if (last_angle - angle < -300 ) {
    revolutions -= 1;
  }
  last_angle = angle;
  absolute_angle = angle + (revolutions * 360);

  
  int actual_servo_pos = analogRead(A0);
  ave.push(actual_servo_pos);
  
  /*
  Serial.print(absolute_angle);
  Serial.print(",");
  Serial.print(ave.mean());
  Serial.print(",");
  */
  int desired_servo_angle = map(absolute_angle,-62,170, 160, -10);
  int actual_servo_angle = map(actual_servo_pos,162,548, 145, 35);
  int servo_error = (desired_servo_angle - actual_servo_angle);


  
  drive(servo_error *2 * -1);
  
  myservo.write(desired_servo_angle);

  
  Serial.print(servo_error);
  Serial.print(",");
  Serial.print(desired_servo_angle);
  Serial.print(",");
  Serial.print(actual_servo_angle);
  
  Serial.println();
  

  previous_error = error;
  last_time = time;
}

void disable() {
  digitalWrite(l_en, LOW);
  digitalWrite(r_en, LOW);
}

void enable() {
  digitalWrite(l_en, HIGH);
  digitalWrite(r_en, HIGH);
}

void drive(int requested_power) {
  requested_power = constrain(requested_power, -100, 100) * -1 ;

  if (requested_power > current_power) {
    current_power += 1;
  }
  if (requested_power < current_power) {
    current_power -= 1;
  }
  if (current_power > 0) {
    digitalWrite(r_pwm, LOW);
    analogWrite(l_pwm, map(current_power, 0, 100, 0, 255));
  }
  else if (current_power < 0) {
    digitalWrite(l_pwm, LOW);
    analogWrite(r_pwm, map(current_power, 0, -100, 0, 255));
  }
  else {
    digitalWrite(l_pwm, LOW);
    digitalWrite(r_pwm, LOW);
  }
  if (requested_power == current_power) {
    return;
  }
  //  delay(1);
  delayMicroseconds(500); // ramp dampening for motor and driver protection
}
