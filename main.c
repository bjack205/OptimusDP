
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
typedef enum{start, forward, turning, end, test} statedef;
statedef state = start;
int start_button = 0;

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
    step = step + 1;
    
}

void StepperStop(){
    stepper_out(1, 0, 0);
    _LATB0 = 0; // Sleep Motor
}

int StepFinished(){
    if (step > step_target) {
        step = 0;
        StepperStop();
        return 1;
    }
    return 0;
}

void TurnRobot(double angle, char direction, int speed, int step_size){
    step = 0;
    TMR2 = 0;
    //Validate step size input
    if (step_size != 1 && step_size != 2 && step_size != 8 && step_size != 16)
        step_size = 1; //default to whole step
    
    //Validate speed
    if (speed < 0 || speed > 4)
        speed = 2; // Default to half speed 
    
    angle = angle*PI/180.0; //convert to radians
    step_target = R_base*angle*step_size; // calculate number of steps
    stepper_out(step_size, direction, speed);
}

void DriveRobot(double distance, char direction, int speed, int step_size) {
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

int Solenoid(double time){
    static int SolState = 0;
    static int startTime = 0;
    switch (SolState){
        case 0: //New
            startTime = gtime;
            SolState = 1;
            _LATB1 = 1;
            break;
        case 1: // Running
            if (gtime - startTime > time){
                SolState = 2;
            }
            break;
        case 2: // End
            SolState = 0;
            _LATB1 = 0;
            return 1;
    }
    return 0;
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
    
    int speed = 1;
    
    _LATA0 = 0;
    _LATA1 = 0;
    while(1) {
        switch (state) {
            case test:
                if (_RB14){
                    if(Solenoid(1000))
                        state = end;
                }
                else {
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



