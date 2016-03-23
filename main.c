
#include "Header.h"
#include "Steppers.h"
#include "Config.h"
#include "IRSensors.h"

_FOSCSEL(FNOSC_FRC & SOSCSRC_DIG) // 8 MHz Declare oscillator and Turn off Secondary oscillator on Pins 9 and 10
_FICD(ICS_PGx1) //Set Debug Pins
_FOSC(OSCIOFNC_OFF)//Turn off CLK output on Pin 8
        
// Global Variables
int gtime = 0;
int step = 0;
int step_target = 0;
int start_button = 0;

//State Variables
typedef enum{start, FindHome, Reorient, tohome, reload, tocenter, findgoal, launch,  test, test2} statedef;
statedef state = start;
int goal = -1; //Active Goal
typedef enum{forward,left,right,backwards,none} orientdef;
orientdef orientation = none; //Where front is pointing: 1=forward, 2=left, 3=right, 4=backwards



void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    _OC1IF = 0;
    step++;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    // Clear Timer1 interrupt flag so that the program doesn't just jump
    // back to this function when it returns to the while(1) loop.
    _T1IF = 0;

    // Increment Global Time
    gtime++;
    
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    // Clear Timer1 interrupt flag so that the program doesn't just jump
    // back to this function when it returns to the while(1) loop.
    _T2IF = 0;

    // Increase step by 1 

    //step = step + 1;
    
}

void StepperStop(){
    stepper_out(1, 0, 0);
}

int StepFinished(){
    if (step > step_target) {
        step = 0;
        StepperStop();
        return 1;
    }
    return 0;
}

int TurnRobot(double angle, char direction, int speed, int step_size){
    static int turnstate = 0;
    static int turntime = 0;
    switch (turnstate) {
        case 0:
            step = 0;
            TMR2 = 0;
            
            //Validate step size input
            if (step_size != 1 && step_size != 2 && step_size != 8 && step_size != 16)
                step_size = 1; //default to whole step

            //Validate speed
            if (speed < 0 || speed > 4)
                speed = 2; // Default to half speed 

            angle = angle * PI / 180.0; //convert to radians
            step_target = R_base * angle*step_size; // calculate number of steps
            stepper_out(step_size, direction, speed);
            
            turnstate = 1;
            break;
        case 1: //Turning
            if (StepFinished()){ // Finished
                turnstate = 2;
                turntime = gtime;
            }          
            break;
        case 2: //Holding
            if (gtime-turntime > 500){
                turnstate = 0;
                _LATB0 = 0; // Sleep Motor
                return 1;
            }
    }
    return 0;
}

int DriveRobot(double distance, char direction, int speed, int step_size) {
    static int drivestate = 0;
    static int holdtime = 0;
    switch (drivestate){
        case 0: //Start
            step = 0;
            TMR2 = 0;
            //Validate step size input
            if (step_size != 1 && step_size != 2 && step_size != 8 && step_size != 16)
                step_size = 1; //default to whole step

            //Validate speed
            if (speed < 0 || speed > 4)
                speed = 2; // Default to half speed 

            step_target = distance*step_size; // calculate number of steps
            stepper_out(step_size, direction, speed);
            
            drivestate = 1;
            break;
        case 1: //Driving
            if (StepFinished()){ // Finished
                drivestate = 2;
                holdtime = gtime;
            }          
            break;
        case 2: //Holding
            if (gtime-holdtime > 500){
                drivestate = 0;
                _LATB0 = 0; // Sleep Motor
                return 1;
            }
            
    }
    return 0;
}

int RampTurn(double angle, char direction, int step_size) {
    static int turnstate = 0;
    static double ramp_speed = 20;
    static int ramp_time = 0;
    int ramp_rate = 2;
    int delay = 2;
    int speed = 300;
    int crashspeed = 50;
    
    switch (turnstate){
        case 0: //Start
            step = 0;
            TMR2 = 0;
            
            //Validate step size input
            if (step_size != 1 && step_size != 2 && step_size != 8 && step_size != 16)
                step_size = 1; //default to whole step

            angle = angle * PI / 180.0; //convert to radians
            step_target = R_base * angle*step_size; // calculate number of steps
            ramp_time = gtime;
            
            turnstate = 1;
            break;
        case 1: //Ramp up Speed
            if (ramp_speed < speed) {
                if (gtime-ramp_time > delay){
                    ramp_speed += ramp_rate;
                    ramp_time = gtime;
                    
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            else {
                stepper_out_ramp(step_size, direction, speed);
                turnstate = 2;
            }
            break;
        case 2: // Max Speed
            if (step > step_target-40*step_size){
                if (state == tohome)
                    turnstate = 5;
                else
                    turnstate = 3;
            }
            break;
        case 3: // Ramp down to stop
            if (StepFinished()){ // Stop
                ramp_time = gtime;
                turnstate = 4;
            }    
            else { 
                if (gtime-ramp_time > delay && ramp_speed > 30) {
                    ramp_speed -= ramp_rate+1;
                    ramp_time = gtime;
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            break;
        case 4: //Hold
            if (gtime-ramp_time > 200){
                turnstate = 0;
                ramp_time = 0;
                ramp_speed = 20;
                _LATB0 = 0; // Sleep Motor
                return 1;
            }
            break;
        case 5: //Run into wall
            if (gtime-ramp_time > delay && ramp_speed > crashspeed){
                    ramp_speed -= ramp_rate+1;
                    ramp_time = gtime;
                    stepper_out_ramp(step_size, direction, ramp_speed);
            }
            if (WallContact()){
                turnstate = 4;
            }
            if (step > step_target + 500*step_size){ // Failsafe
                turnstate = 4;
            }    
            break;
    }
    return 0;
}

int RampDrive(double distance, char direction, int step_size) {
    static int drivestate = 0;
    static double ramp_speed = 20;
    static int ramp_time = 0;
    int ramp_rate = 2;
    int delay = 2;
    int speed = 300;
    int crashspeed = 50;
    
    switch (drivestate){
        case 0: //Start
            step = 0;
            TMR2 = 0;
            //Validate step size input
            if (step_size != 1 && step_size != 2 && step_size != 8 && step_size != 16)
                step_size = 1; //default to whole step

            step_target = distance*step_size; // calculate number of steps
            ramp_time = gtime;
            
            drivestate = 1;
            break;
        case 1: //Ramp up Speed
            if (ramp_speed < speed) {
                if (gtime-ramp_time > delay){
                    ramp_speed += ramp_rate;
                    ramp_time = gtime;
                    
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            else {
                stepper_out_ramp(step_size, direction, speed);
                drivestate = 2;
            }
            break;
        case 2: // Max Speed
            if (step > step_target-40*step_size){
                if (state == tohome)
                    drivestate = 5;
                else
                    drivestate = 3;
            }
            break;
        case 3: // Ramp down to stop
            if (StepFinished()){ // Stop
                ramp_time = gtime;
                drivestate = 4;
            }    
            else { 
                if (gtime-ramp_time > delay && ramp_speed > 30) {
                    ramp_speed -= ramp_rate+1;
                    ramp_time = gtime;
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            break;
        case 4: //Hold
            if (gtime-ramp_time > 200){
                drivestate = 0;
                ramp_time = 0;
                ramp_speed = 20;
                _LATB0 = 0; // Sleep Motor
                return 1;
            }
            break;
        case 5: //Run into wall
            if (gtime-ramp_time > delay && ramp_speed > crashspeed){
                    ramp_speed -= ramp_rate+1;
                    ramp_time = gtime;
                    stepper_out_ramp(step_size, direction, ramp_speed);
            }
            if (WallContact()){
                drivestate = 4;
            }
            if (step > step_target + 500*step_size){ // Failsafe
                drivestate = 4;
            }    
            break;
    }
    return 0;
}

int WallContact(){
    int LBump = _RB15;
    int RBump = _RB14;
    if (LBump && RBump) //Both buttons pressed
        return 1;
    return 0;
}

int Start_Check() {
    int value = 0;
    if (_RB14 == 1) {
        if (start_button == 0) { // Button Pressed 
            value = 1;
        }
        start_button = 1;
    }
    else {
        start_button = 0;
    }
    return value;
    
 }

int Solenoid(double timeON, double timeOFF, int repeat){
    static int SolState = 0;
    static int startTime = 0;
    static int repeat_count = 1;
    switch (SolState){
        case 0: //New
            startTime = gtime;
            SolState = 1;
            _LATB4 = 1;
            break;
        case 1: // Activated
            if (gtime - startTime > timeON){ //On time finished
                _LATB4 = 0; // Turn off
                SolState = 2;
            }
            break;
        case 2: //Off
            if (gtime - startTime > timeON + timeOFF) {
                if (repeat_count >= repeat){
                    SolState = 3; //Exit
                }
                else {
                    SolState = 0; // Repeat
                }
                repeat_count++;
            }
            break;
        case 3: // End
            SolState = 0;
            repeat_count = 1;
            return 1;
    }
    return 0;
}

int Launch(){
    static int stime = 0;
    static  launchstate = 0;
    
    switch (launchstate){
        case 0:
            stime = gtime;
            _LATB1 = 1; // Turn on Motor
            launchstate = 1;
            break;
        case 1:
            if (gtime-stime > 3000){
                _LATB1 = 0; // Turn off motor
                launchstate = 0;
                stime = 0;
                return 1;
            }
            
    }
    return 0;
}

int ReadIR() {
    static char IRState = 'H'; // H = Home, S = Search for IR, F = Found IR and hone in
    float IRThreshold = 1.0;
    float VoltageFront = 0; //(ADC1BUF0 / 4095.0) * 3.3;
    float VoltageLeft = (ADC1BUF1 / 4095.0) * 3.3;
    float VoltageRight = (ADC1BUF4 / 4095.0) * 3.3;
    
    // Front IR
    if (VoltageFront >= IRThreshold) {
        // Don't turn
        return 0;
    }
    
    // Left IR
    if (VoltageLeft >= IRThreshold) {
        // turn toward LED
        return 1;
    }
    
    // Right IR
    if (VoltageRight >= IRThreshold) {
        //turn toward LED
        return 2;
    }
    
    return -1;
}


int main () {
    TMR1_Config();
    config_ad();
    config_IR();
    config_step1();
    
    _ANSA0 = 0; // Set up AN0 (Pin 2) as digital
    _TRISA0 = 0; // Set up Pin 2 as output
    _ANSA1 = 0; // Set up AN1 (Pin 3) as digital
    _TRISA1 = 0; // Set up Pin 3 as output
    _ANSB14 = 0; // Set up pin 17 as digital
    _TRISB14 = 1; // Set up pin 17 as input for push button start
    
    //Configure LAUNCH Pin on Pin 9
    _ANSB4 = 0; // Set Pin 9 as digital
    _TRISB4 = 0; // Set up Pin 9 as output
    
    //Configure Left Button on Pin 18
    _ANSB15 = 0; // Set Pin 18 as digital
    _TRISB15 = 1; // Set Pin 18 as input
    
    _TRISB8 = 0;
    
    
    int counter = 0;
    
    
    state = test;
    
    int speed = 3;
    
    
    if (0){
        int i = 20;
        int time = gtime;
        while(i<400) {
            if (gtime - time > 15){
                stepper_out_ramp(2, 'F', i);
                time = gtime;
                i++;
            }
        }
    }
    
    
    _LATA0 = 0;
    _LATA1 = 0;
    
    _LATA0 = 1;
    _LATB1 = 1;
    _LATB8 = 1;
    while(1) {
        switch (state) {
            case test:
                if (Start_Check()){
                    state = Reorient;
                }
                else {
                    
                }
                break;
            case Reorient:
                if (RampTurn(90,'L',8)){
                    state = test;
                }
                break;
            case tohome:
                if (RampDrive(550,'B',8)){
                    state = test;
                }
                break;
            case findgoal:
                switch (goal){
                    case -1: //Forward
                        state = test;
                        break;
                    case 1: //Left Goal
                        if (TurnRobot(90, 'L', 1, 1)) {
                            state = test;
                        }
                        break;
                    case 2: //Right Goal
                        if (TurnRobot(90, 'R', 1, 1)) {
                            state = test;
                        }
                        break;
                    default:
                        state = test;
                }
                if (goal == 0){ //Forward
                    state = test;
                }
                if (goal == 1) {
                    
                }
                break;
            case launch:
                if (Launch()){
                    state = test;
                }
                break;
            case tocenter:
                if (DriveRobot(670,'F',1,8)){ //670
                    goal = ReadIR();
                    state = findgoal;
                }
                break;
            case reload:
                if(Solenoid(25,100,6)){
                    state = test;
                }
                break;
            case start:
                if (Start_Check()) {
                    _LATA0 = 1;
                    state = forward;
                    counter = 0;
                   // TurnRobot(90, 'R', 2, 1);
                    DriveRobot(200, 'F', speed, 1);
                }
                break;
            case forward:
                if (StepFinished()) {
                    counter++;
                    if (counter == 2) {
                        state = start;
                    }
                    else {
                        state = turning;
                        TurnRobot(90, 'F', speed, 1);
                    }
                }
                break;
            case turning:
                if (StepFinished()) {
                    state = forward;
                    DriveRobot(200, 'F', speed, 1);
                }
                break;
            case end:
               // if (step >= )
                break;
            default:
                state = start;
        }
    }
    return 0;
}



