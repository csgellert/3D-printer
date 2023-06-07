#include "Dynamics.h"

Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];

/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
  Serial.print("Pause: ");
  Serial.println(ms);
}

/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  nfr = nfr*STEPS_PER_MM/60; 
  Serial.print("feedrate: ");
  Serial.println(nfr);
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = MAX_FEEDRATE/nfr;
  fr=nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy,float npz, float npe) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
  pz=npz;
  pe=npe;
}

/**
 * Supports movement with both styles of Motor Shield
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void onestep(int motor) {
#ifdef VERBOSE
  char *letter="XYZE";
  Serial.print(letter[]);
#endif
  
  digitalWrite(motors[motor].step_pin,HIGH);
  digitalWrite(motors[motor].step_pin,LOW);
}

/*
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 *
 */
void line(float newx,float newy,float newz, float newe) {
  a[0].delta = (newx-px)*STEPS_PER_MM_X;
  a[1].delta = (newy-py)*STEPS_PER_MM_Y;
  a[2].delta = (newz-pz)*STEPS_PER_MM_Z;
  a[3].delta = (newe-pe)*STEPS_PER_MM_E;
  
  long i,j,maxsteps=0;

  for(i=0;i<NUM_AXIES;++i) {
    a[i].absdelta = abs(a[i].delta);
    a[i].over=0;
    if( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;
    // set the direction once per movement
    digitalWrite(motors[i].dir_pin,a[i].delta>0?HIGH:LOW);
  }
  
  long dt = MAX_FEEDRATE/5000;
  long accel = 1;
  long steps_to_accel = dt - step_delay;
  if(steps_to_accel > maxsteps/2 ) 
    steps_to_accel = maxsteps/2;
    
  long steps_to_decel = maxsteps - steps_to_accel;

  Serial.print("START ");
  Serial.println(dt);
  Serial.print("STOP ");
  Serial.println(step_delay);
  
  Serial.print("accel until ");
  Serial.println(steps_to_accel);  
  Serial.print("decel after ");
  Serial.println(steps_to_decel);  
  Serial.print("total ");
  Serial.println(maxsteps);  
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  for( i=0; i<maxsteps; ++i ) {
    for(j=0;j<NUM_AXIES;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= maxsteps) {
        a[j].over -= maxsteps;
        
        digitalWrite(motors[j].step_pin,HIGH);
        digitalWrite(motors[j].step_pin,LOW);
      }
    }

    if(i<steps_to_accel) {
      dt -= accel;
    }
    if(i>=steps_to_decel) {
      dt += accel;
    }
    delayMicroseconds(dt);
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif
  if (intr_hap == 0){
    position(newx,newy,newz,newe);
  }
  else if(intr_hap ==1){
    for(int i=0;i<1;++i) {  
    // set the motor pin & scale
    //motor_calibration(i);
    }
  }
  intr_hap = 0;

  where();
}

// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}

/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.print(val);
  Serial.print(" ");
}

/*

 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("Z",pz);
  output("E",pe);
  output("F",fr/STEPS_PER_MM*60);
  Serial.println(mode_abs?"ABS":"REL");
  Serial.println();
} 

/**
 * set up the pins for each motor
 * Pins fits a CNCshieldV3.xx
 */
void motor_setup() {
  motors[0].step_pin=X_STEP_PIN;
  motors[0].dir_pin=X_DIR_PIN;
  motors[0].enable_pin=X_ENABLE_PIN;
  motors[0].max_pin = X_MAX_PIN;
  motors[0].min_pin = X_MIN_PIN;

  motors[1].step_pin=Y_STEP_PIN;
  motors[1].dir_pin=Y_DIR_PIN;
  motors[1].enable_pin=Y_ENABLE_PIN;
  motors[1].max_pin = Z_MAX_PIN;
  motors[1].min_pin = Z_MIN_PIN;

  motors[2].step_pin=Z_STEP_PIN;
  motors[2].dir_pin=Z_DIR_PIN;
  motors[2].enable_pin=Z_ENABLE_PIN;
  motors[2].max_pin = Y_MAX_PIN;
  motors[2].min_pin = Y_MIN_PIN;

  motors[3].step_pin=E1_STEP_PIN;
  motors[3].dir_pin=E1_DIR_PIN;
  motors[3].enable_pin=E1_ENABLE_PIN;
  //motors[3].max_pin = Y_MAX_PIN;
  //motors[3].min_pin = Y_MIN_PIN;

  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);
    pinMode(motors[i].max_pin, INPUT_PULLUP);
    pinMode(motors[i].min_pin, INPUT_PULLUP);
  }

  for(int i=0;i<1;++i) {  
    // set the motor pin & scale
    //motor_calibration(i);
  }
  //AccelStepper motors[0] = AccelStepper(motorInterfaceType, X_STEP_PIN, X_DIR_PIN);

  attachInterrupt(digitalPinToInterrupt(X_MIN_PIN),Stop_Xmotor_Min,FALLING);
  attachInterrupt(digitalPinToInterrupt(X_MAX_PIN),Stop_Xmotor_Max,FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MIN_PIN),Stop_Ymotor_Min,FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MAX_PIN),Stop_Ymotor_Max,FALLING);
  //attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN),Stop_motors,FALLING);
  //attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN),Stop_motors,FALLING);
}

void motor_calibration(int motor){
  noInterrupts();
  Serial.println("Calibration start");
  digitalWrite(motors[motor].dir_pin,LOW);
  int loop = digitalRead(motors[motor].min_pin);
  while(loop != 0){
    digitalWrite(motors[motor].step_pin,HIGH);
    digitalWrite(motors[motor].step_pin,LOW);
    loop = digitalRead(motors[motor].min_pin);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  
  Serial.println("Calibration done");
  interrupts(); 
}

void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,LOW);
  }
  Serial.println("Motors enable");
}

void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,HIGH);
  }
  Serial.println("Motors disable");
}

void Stop_Xmotor_Max(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[0].max_pin) == 0){
    digitalWrite(motors[0].step_pin,HIGH);
    digitalWrite(motors[0].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}

void Stop_Xmotor_Min(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[0].min_pin) == 0){
    digitalWrite(motors[0].step_pin,HIGH);
    digitalWrite(motors[0].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}

void Stop_Ymotor_Max(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[1].max_pin) == 0){
    digitalWrite(motors[1].step_pin,HIGH);
    digitalWrite(motors[1].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}

void Stop_Ymotor_Min(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[1].min_pin) == 0){
    digitalWrite(motors[1].step_pin,HIGH);
    digitalWrite(motors[1].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}