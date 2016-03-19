
#include"Header.h"
#include"Steppers.h"
#include"Config.h"

_FOSCSEL(FNOSC_FRC) // 8 MHz Declare oscillator
_FICD(ICS_PGx1) //Set Debug Pins
_FOSC(OSCIOFNC_OFF)//Turn off Secondary oscillator
        
// Global Variables
int gtime = 0;
int step = 0;
int step_target = 0;
typedef enum{test, start, tocenter, forward, turning, end, reload, launch} statedef;
statedef state = start;
int start_button = 0;

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
        case 1:
            if (StepFinished()) {
                turnstate = 0;
                return 1;
            }
            break;
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

int RampRobot(double distance, char direction, int speed, int step_size) {
    static int drivestate = 0;
    static double ramp_speed = 20;
    static int ramp_time = 0;
    int ramp_rate = 2;
    int delay = 2;
    
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
            if (step > step_target*3.0/4.0){
                drivestate = 3;
            }
            break;
        case 3: // Ramp down to stop
            if (StepFinished()){ // Stop
                drivestate = 0;
                ramp_time = 0;
                ramp_speed = 20;
                return 1;
            }    
            else { 
                if (gtime-ramp_time > delay && ramp_speed > 20) {
                    ramp_speed -= ramp_rate;
                    ramp_time = gtime;
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            break;
            
    }
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
            _LATB1 = 1;
            break;
        case 1: // Activated
            if (gtime - startTime > timeON){ //On time finished
                _LATB1 = 0; // Turn off
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
    float IRThreshold = 2.0;
    float VoltageFront = (ADC1BUF0 / 4095) * 3.3;
    float VoltageLeft = (ADC1BUF1 / 4095) * 3.3;
    float VoltageRight = (ADC1BUF4 / 4095) * 3.3;
    
    switch (IRState) {
        case 'H':
            TurnRobot(180, 'L', 1, 1);
            if (VoltageLeft >= IRThreshold) {
                TurnRobot(90, 'R', 1, 1);
            }
            else if (VoltageRight >= IRThreshold) {
                TurnRobot(90, 'L', 1, 1);
            }
            else {
                
            }
        case 'S':
        case 'F':
        default:
            
    }
    
    // Front IR
    if (VoltageFront >= IRThreshold) {
        // Don't turn
        return 0;
    }
    
    // Left IR
    if (VoltageLeft >= IRThreshold) {
        // turn toward LED
        TurnRobot(90, 'L', 1, 1);
        return 1;
    }
    
    // Right IR
    if (VoltageRight >= IRThreshold) {
        //turn toward LED
        TurnRobot(90, 'R', 1, 1);
        return 2;
    }
    
    return -1;
}


int main () {
    
    
    _ANSA0 = 0; // Set up AN0 (Pin 2) as digital
    _TRISA0 = 0; // Set up Pin 2 as output
    _ANSA1 = 0; // Set up AN1 (Pin 3) as digital
    _TRISA1 = 0; // Set up Pin 3 as output
    _ANSB14 = 0; // Set up pin 17 as digital
    _TRISB14 = 1; // Set up pin 17 as input for push button start
    
    //Configure LAUNCH Pin on Pin 5
    _ANSB1 = 0; // Set Pin 5 as digital
    _TRISB1 = 0; // Set up Pin 5 as output
    
    //Configure Left Button on Pin 18
    _ANSB15 = 0; // Set Pin 18 as digital
    _TRISB15 = 1; // Set Pin 18 as input
    
    
    int counter = 0;
    
    TMR1_Config();
    config_step1();
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
    while(1) {
        switch (state) {
            case test:
                if (Start_Check()){
                    state = tocenter;
                }
                else {
                }
                break;
            case launch:
                if (Launch()){
                    state = test;
                }
                break;
            case tocenter:
                if (DriveRobot(670,'B',4,8)){ //670
                    state = test;
                }
                break;
            case reload:
                if(Solenoid(50,500,6)){
                    StepperStop();
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
                        TurnRobot(90, 'R', speed, 1);
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



