/*
  MIT License

  Copyright (c) [2018] [John Peter Davalos]

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


//PIN configurations
const byte pin_a = 2;   // Encoder A
const byte pin_b = 3;   // Encoder B
const byte pin_fwd = 4; // IN1 of the Motor Driver, enables forward motor
const byte pin_bwd = 5; // IN2 of the Motor Driver, enables backward motor
const byte pin_pwm = 6; // ENA of the Motor Driver, motor speed pin

//PID algorithm variables
float target_speed = 0.0;
float current_speed = 0.0;

float p_value = 0.0;
float i_value = 0.0;
float d_value = 0.0;

float p_correction = 0.0;
float i_correction = 0.0;
float d_correction = 0.0;

float error = 0.0;
float lastError = 0.0;
float final_correction = 0.0;

float accumulative_error = 0.0;
float slope = 0.0;

//Other parameters
short int pulse = 0;
bool motor_check = false;
bool motor_start = false;


void setup()
{
  //Start serial port using 19200 baud rate - must match with Matlab code
  Serial.begin(19200);
  delay(100);

  //Input Pins
  pinMode(pin_a, INPUT_PULLUP);
  pinMode(pin_b, INPUT_PULLUP);
  delay(100);

  //Output Pins
  pinMode(pin_fwd, OUTPUT);
  pinMode(pin_bwd, OUTPUT);
  pinMode(pin_pwm, OUTPUT);
  delay(100);

  //Attach Interupt
  attachInterrupt(digitalPinToInterrupt(pin_a), set_point, RISING);
  delay(100);

  //Optimize timing speed
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 59000; //for 0.1 sec

  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  interrupts();

  //Initialize Motor speed and Position
  analogWrite(pin_pwm, 0);
  digitalWrite(pin_fwd, 0);
  digitalWrite(pin_bwd, 0);

}

void loop() {
  // Motor check will come from Matlab (SEND)

  if (motor_check == true) {
    digitalWrite(pin_fwd, HIGH);     //run motor run forward
    digitalWrite(pin_bwd, LOW);
    motor_start = true;
  }
  else if (motor_check == false) {
    digitalWrite(pin_fwd, LOW);
    digitalWrite(pin_bwd, LOW);     //stop motor
    motor_start = false;
  }

}

// Optimization
ISR(TIMER1_OVF_vect)
{
  TCNT1 = 59000; //for 0.1 sec
  current_speed = (pulse); //current speed
  pulse = 0;

  //print out speed
  if (Serial.available() <= 0)
  {
    Serial.println(current_speed);
  }

  //PID algorithm starts here
  if (motor_start)
  {
    //Calculate current error..
    error = target_speed - current_speed;

    //P Correction..
    p_correction = p_value * error;

    //I Correction..
    accumulative_error += error;
    i_correction = i_value * accumulative_error;

    //D Correction..
    slope = error - lastError;
    d_correction = d_value * slope;
    lastError = error;

    //Final correction
    final_correction = p_correction + i_correction + d_correction;

    //Fixes
    if (accumulative_error > 1000) accumulative_error = 1000;
    if (accumulative_error < -1000) accumulative_error = -1000;
  }

  //If motor = stop, reset values
  else
  {
    error = 0.0;
    lastError = 0.0;
    accumulative_error = 0.0;
    final_correction = 0.0;
  }

  //Update the speed
  if (final_correction <= 255 && final_correction > 0)
  {
    analogWrite(pin_pwm, final_correction); //set motor speed
  }
  else if (final_correction > 255)
  {
    analogWrite(pin_pwm, 255); //set max speed
  }
  else if (final_correction < 0)
  {
    analogWrite(pin_pwm, 0); //set to 0
  }

}

void set_point()
{
  pulse = pulse + 1; // Pulse per loop
}

//Event for receiving Serial data from Matlab GUI
void serialEvent()
{

  /*
    Received data sample: 1,100,0.18,0.36,0.00225

    1st number = motor start , 1 if start, 0 if stop
    2nd number = motor speed
    3rd number = Kp
    4th number = Ki
    5th number = Kd

  */

  motor_check = Serial.parseInt(); // variable if start or stop
  target_speed = Serial.parseInt(); // target speed set from Matlab GUI
  p_value = Serial.parseFloat(); // P value from Matlab GUI
  i_value = Serial.parseFloat(); // I value from Matlab GUI
  d_value = Serial.parseFloat();// D value from Matlab GUI
}

