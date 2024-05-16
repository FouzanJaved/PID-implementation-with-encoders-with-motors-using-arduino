 #include <TimerOne.h>

//set the desired speed here
unsigned int desired_speed=5; 
unsigned int desired_speed2=5;

//variables for the PID controller
unsigned int measured_speed=0; 
unsigned int measured_speed2=0;
unsigned int total_pulses=0; 
unsigned int total_pulses2=0;

unsigned int set_power=0;
unsigned int set_power2=0;
unsigned int maximum_power=150;
unsigned int maximum_power2=150;
unsigned int minimum_power=20;
unsigned int minimum_power2=20;

//tune these PID coefficients
float Kp=10;
float Ki=0.7;
float Kd=0.1;

float Kp2=10;
float Ki2=0.7;
float Kd2=0.1;

int error=0;
int prev_error=0;
int integral=0;
int diff=0;

int error2=0;
int prev_error2=0;
int integral2=0;
int diff2=0;

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 6;
int in1 = 13;
int in2 = 12;
// motor two
int enB = 5;
int in4 = 7;
int in3 = 8;

void docount()  // counts from the speed sensor
{
  measured_speed++;  // increase +1 the measured_speed value
  total_pulses += measured_speed;
} 
void docount2()  // counts from the speed sensor
{
  measured_speed2++;  // increase +1 the measured_speed value
  total_pulses2 += measured_speed2;
}  
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  
  error = measured_speed - desired_speed;
  Serial.println("error1=");
  Serial.println(error);
  Serial.println("\n");
  integral += (error + prev_error)* Ki;
 // diff = (error - prev_error)* Kd;

  if (set_power < maximum_power)
  {
   // set_power -= ((error * Kp) + integral + diff);
    // set_power -= ((error * Kp) + integral);
    set_power -= ((error * Kp));
    Serial.print("Pwm1=");
    Serial.print(set_power,DEC);
    Serial.print("\n");
  }
  else {set_power =minimum_power; integral =0;};

  error2 = measured_speed2 - desired_speed2;
  integral2 += (error2 + prev_error2)* Ki;
  diff2 = (error2 - prev_error2)* Kd;

    Serial.println("error2=");
  Serial.println(error2);
  Serial.println("\n");

  if (set_power2 < maximum_power2)
  {
    
    //set_power2 -= ((error2 * Kp) + integral2 + diff);
    //set_power2 -= ((error2 * Kp) + integral2);
    set_power2 -= ((error2 * Kp));
    Serial.print("Pwm2=");
    Serial.print(set_power2,DEC);
    Serial.print("\n");
  }
  
  else {set_power2 = minimum_power2; integral2 = 0;};

  prev_error = error;
  prev_error2 = error2;

  int temp = (measured_speed); 
  Serial.print("Motor1="); 
  Serial.print(temp,DEC);  
  
  temp = (measured_speed2); 
  Serial.print("Motor2="); 
  Serial.print(temp,DEC);
  
  Serial.println(); 

/*

  Serial.print(" measured_speed:"); 
  int temp = (measured_speed); 
  Serial.print(temp,DEC);  
  
  Serial.print(" measured_speed2:"); 
  temp = (measured_speed2); 
  Serial.print(temp,DEC);  
  
  Serial.print(" set_power:"); 
  temp = (set_power);
  Serial.print(temp,DEC);  
  
  Serial.print(" set_power2:"); 
  temp = (set_power2); 
  Serial.print(temp,DEC);  

  Serial.print(" total_pulses:"); 
  temp = (total_pulses);
  Serial.print(temp,DEC);  
  
  Serial.print(" total_pulses2:"); 
  temp = (total_pulses2); 
  Serial.print(temp,DEC);  

  Serial.print(" error:"); 
  temp = (error);
  Serial.print(temp,DEC);  
  
  Serial.print(" error2:"); 
  temp = (error2); 
  Serial.print(temp,DEC); 

  Serial.print(" integral:"); 
  temp = (integral);
  Serial.print(temp,DEC);  
  
  Serial.print(" integral2:"); 
  temp = (integral2); 
  Serial.print(temp,DEC);  

  Serial.print(" diff:"); 
  temp = (diff);
  Serial.print(temp,DEC);  
  
  Serial.print(" diff2:"); 
  temp = (diff2); 
  Serial.print(temp,DEC);  
  
  Serial.println(" End"); 
  
*/
  
  measured_speed=0;  //  reset measured_speed to zero
  measured_speed2=0;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

void setup() {
  // put your setup code here, to run once:
// set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  //Serial.begin(9600);
  Serial.begin(9600);
  enable_interrupt();  
}
void enable_interrupt()
{
  Timer1.initialize(1000000); // set timer for 0.1 sec
  attachInterrupt(digitalPinToInterrupt(2), docount, RISING);  // increase measured_speed when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(3), docount2, RISING);  // increase measured_speed when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}


//
//void disable_interrupt()
//{
//  detachInterrupt(digitalPinToInterrupt(2));  // increase measured_speed when speed sensor pin goes High
//  detachInterrupt(digitalPinToInterrupt(3));  // increase measured_speed when speed sensor pin goes High
//  Timer1.detachInterrupt(); // enable the timer
//}


void demoStraightPID()
{
// this function will run the motors in a straight line at a fixed speed

//set the desired speed here
  desired_speed=13; 
  desired_speed2=13;

  unsigned int lpower = (set_power);
  unsigned int lpower2 = (set_power2); 
  analogWrite(enA, lpower);
  analogWrite(enB, lpower2);
  
// turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
// turn on motor B: faulty, cannot reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
}

//void demoStop()
//{
// // now turn off motors
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, LOW);  
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW); 
//}

//void demoOne()
//{
//// this function will run the motors in a rectangle at a fixed speed
//// set speed to 200 out of possible range 0~255
//    analogWrite(enA, 70);
//    analogWrite(enB, 70);
//
//  // run straight
//  // turn on motor A
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);  
//  // turn on motor B: faulty, cannot reverse
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, HIGH); 
//  delay(1000);
//  // turn left
//   // turn on motor A
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);  
//  // turn on motor B: faulty, cannot reverse
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW); 
//  delay(500);  
// // now turn off motors
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, LOW);  
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW);  
//  delay(2000);
//}

//void demoTwo()
//{
//  // this function will run the motors across the range of possible speeds
//  // note that maximum speed is determined by the motor itself and the operating voltage
//  // the PWM values sent by analogWrite() are fractions of the maximum speed possible 
//  // by your hardware
//  // turn on motors
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);  
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, HIGH); 
//  // accelerate from zero to maximum speed
//  for (int i = 0; i < 256; i++)
//  {
//    analogWrite(enA, i);
//    analogWrite(enB, i);
//    delay(20);
//  } 
//  // decelerate from maximum speed to zero
//  for (int i = 255; i >= 0; --i)
//  {
//    analogWrite(enA, i);
//    analogWrite(enB, i);
//    delay(20);
//  } 
//  // now turn off motors
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, LOW);  
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW);  
//}

void loop() {
  // put your main code here, to run repeatedly:
  //disable_interrupt();
  //demoOne();
//  demoTwo();
  demoStraightPID();
 // delay(5000);

}