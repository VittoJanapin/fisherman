#include <Arduino.h>
#include <cmath>

/**************************************************************************

Adjust These parameters

***************************************************************************/
//PID Constants
static const float K_p = 10;
static const float K_i = 20;
static const float K_d = 1;
// Anti Deadband System
static const float MIN_SPEED = 2.0; //speed is encoder positions/dt determine this by experiment
static const float STICTION_KICK = 40; //raise this if its not budging (ranges from 0-255)
// HARDCODED POSITIONS (SEE README FOR CONVERSION GUIDE)
static const int RELEASE = -56;
static const int EQUILIBRIUM = 0; //adjust during calibration see readme
static const int PICKUP = 2260;




//INPUT PINS --- refer to ESPWROOM32 - 38 PIN pinout
const int motorOne = 32;
const int motorTwo = 33;
const int encA = 19;
const int encB = 18;
const int inputSignal = 17;


//position globals
volatile int pos;
volatile int prevPos;
volatile uint8_t prevState;
void IRAM_ATTR handle(){
    
    static const int8_t stateTransitionTable[4][4] = {
        /*0*/ { 0, +1, -1,  0 },
        /*1*/ { -1, 0,  0, +1 },
        /*3*/ { 0, -1, +1,  0 },
        /*2*/ { +1, 0,  0, -1 }
    };
    uint8_t a = digitalRead(encA);
    uint8_t b = digitalRead(encB);
    
    uint8_t curr = ((a << 1) | b );
    pos += stateTransitionTable[prevState][curr];      
    prevState = curr;
}

/*******
Globals
*******/
unsigned long lastTime;
float error_integral;
float prevError;
int allow = 0;
float speed;
float PID(int error, unsigned long currentTime, bool & dir){
    //handle only positive errors 
    
    unsigned long delta = currentTime - lastTime;
    lastTime = currentTime; 
    float dt = delta/1000000.0;
    float e = error;
    //calculate the mf
    
    float P = K_p * e;
    error_integral += e*allow; //accumulative from time 0
    float I = K_i * (error_integral*dt);
    float D = K_d * (e-prevError)/dt;
    prevError = error; 
    
    
    float output = P + I + D;

    float speed = (pos-prevPos)/dt;
    prevPos = pos;
    if(abs(output) > 5 && abs(speed) < MIN_SPEED){
        if(output > 0){
            output += STICTION_KICK;
        } 
        else{
            output -= STICTION_KICK;
        }
    }

    Serial.print(P);
    Serial.print(" ");
    Serial.print(I);
    Serial.print(" ");
    Serial.print(D);
    Serial.print(" ");
    Serial.print(e-prevError);
    Serial.print(" ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(dt);
    Serial.print("     |      ");
    int control = round(output);
    
    dir = (control < 0);//negative is true
    return control;
    
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    
    pinMode(inputSignal, INPUT);
    ledcSetup(0, 20000, 8);
    ledcSetup(1, 20000, 8); //channel freq, resolution
    ledcAttachPin(motorOne, 0); //channel 0
    ledcAttachPin(motorTwo, 1); //channel 1
    pinMode(encA, INPUT);
    pinMode(encB, INPUT);
    pos = 0;
    prevPos = 0;
    speed = 0;
    prevState = 0;
    prevError = 0;
    error_integral = 0;
    attachInterrupt(digitalPinToInterrupt(encA), handle, CHANGE); //pin, function to call 
}

int desiredPosition = 0;
void loop() {
    // put your main code here, to run repeatedly:
    unsigned long current = micros();
    int speedOne,speedTwo;
    
    unsigned long pulse = pulseIn(inputSignal, HIGH, 25000);

    // 232 per rotation
    if (pulse > 800 && pulse <= 1333){
        desiredPosition = PICKUP;
    }
    else if(pulse > 1333 && pulse <= 1666){
        desiredPosition = EQUILIBRIUM;
    }
    else if(pulse > 1666){
        desiredPosition = RELEASE;
    }
    else{
        desiredPosition = EQUILIBRIUM;
    }


    int error = desiredPosition - pos;
    bool dir;
    int controlSignal = PID(error, current, dir);

    //pwm clamper
    if(controlSignal > 255){
        controlSignal = 255;
    }
    else if(controlSignal < -255){
        controlSignal = -255;
    }
    
    if(controlSignal > 0){
        //go ccw
        speedOne = 255;
        speedTwo = 255-controlSignal;
    }
    else{
        speedOne = 255+controlSignal;
        speedTwo = 255;
    }
    
    

    /** for a serial plotter  **/
    // Serial.println(pos);
    Serial.print(pulse);
    Serial.print(" ");
    Serial.print(desiredPosition);
    Serial.print(" ");
    Serial.print(pos);
    Serial.print(" ");
    Serial.print(controlSignal);
    Serial.println();
        // move to next lin

    ledcWrite(0, speedOne);
    ledcWrite(1, speedTwo);

}
