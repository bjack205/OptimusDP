
#include"Header.h"
#include"Steppers.h"
#include"Config.h"

_FOSCSEL(FNOSC_FRC) // 8 MHz Declare oscillator
_FICD(ICS_PGx1) //Set Debug Pins
_FOSC(OSCIOFNC_OFF)//Turn off Secondary oscillator
        
// Global Variables
int step = 0;
int step_target = 0;
typedef enum{start, forward, turning, end} statedef;
statedef state = start;
int start_button = 0;

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
    int current_state = _RB14;
    if (_RB14 != start_button && _RB14 == 1) {
        return 1;
    }
    else {
        return 0;
    }
    
    
}


int main () {
    
    
    _ANSA0 = 0; // Set up AN0 (Pin 2) as digital
    _TRISA0 = 0; // Set up Pin 2 as output
    _ANSA1 = 0; // Set up AN1 (Pin 3) as digital
    _TRISA1 = 0; // Set up Pin 3 as output
    _ANSB14 = 0; // Set up pin 17 as digital
    _TRISB14 = 1; // Set up pin 17 as input for push button start
    
    int counter = 0;
    
    config_step1();
    
    
    _LATA0 = 0;
    _LATA1 = 0;
    while(1) {
        switch (state) {
            case start:
                if (Start_Check()) {
                    _LATA0 = 1;
                    state = forward;
                    counter = 0;
                   // TurnRobot(90, 'R', 2, 1);
                    DriveRobot(200, 'F', 1, 1);
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
                        TurnRobot(90, 'R', 1, 1);
                    }
                }
                break;
            case turning:
                if (StepFinished()) {
                    state = forward;
                    DriveRobot(200, 'F', 1, 1);
                }
                break;
            case end:
               // if (step >= )
            default:
                state = start;
        }
    }
    // Drive CW, 1/2 step
    //stepper_out(1, 2);
    // Drive CCW, 1/2 step
//    stepper_out(0, 2, 1);
//    
//    
//    //PWM1_Config(1);
//    //PWM_Out(100.0,50.0,1,1);
//    int state = 1;
//    int steprev = 200;
//    int angle = 2*steprev*90.0/360.0;
//    
//    while (1){
//       
//        if(0) {
//        if (step > 200){
//                    state = 2;
//                    angle = 2*steprev*180.0/360.0;
//                    stepper_out(1,2,0);
//                    step = 0;
//        }
//        }
//        
//        
//        if (1){
//        switch (state){
//            case 1:
//                if (step > angle){
//                    state = 2;
//                    angle = 2*steprev*180.0/360.0;
//                    stepper_out(1,2,1);
//                    step = 0;
//                }
//                break;
//            case 2:
//                if (step > angle){
//                    state = 3;
//                    angle = steprev*270.0/360.0;
//                    stepper_out(0,1,1);
//                    step = 0;
//                }
//                break;
//            case 3:
//                if (step > angle){
//                    state = 4;
//                    angle = steprev;
//                    stepper_out(1,1,1);
//                    step = 0;
//                }
//                break;
//            case 4:
//                if (step > angle){
//                    state = 1;
//                    angle = 2*steprev*90.0/360.0;
//                    stepper_out(0,2,1);
//                    step = 1;
//                }
//                break;
//            default:
//                state = 0;
//                break;
//                
//        }
//        }
//            
//    }
//    
    return 0;
}



