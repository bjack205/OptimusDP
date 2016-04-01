#include"Steppers.h"

void config_step1(void)
{
    //Set up M0 and M1 
    _ANSA2 = 0; // Set pin 7 (RA2) as digital. M0
    _TRISA2 = 0; // Set pin 7 as output. 
    _ANSA3 = 0; // Set pin 8 as digital. M1
    _TRISA3 = 0; // Set pin 8 as output.
    
    //Set up SLEEP on Pin 11 (RB7)
    _TRISB7 = 0; // Set pin 4 as output
    _LATB7 = 0; //Default to off
    
    PWM1_Config(64); // Configure Pin 14 as PWM for Step
    
    //Set up LDIR on Pin 15 (AB12)
    _ANSB12 = 0;     // Set Digital
    _TRISB12 = 0;    // Set output
    _LATB12 = 1;    // Direction: 1 = CW; 0 = CCW
    
    //Set up RDIR on Pin 16 (AB13)
    _ANSB13 = 0;    // Set Digital
    _TRISB13 = 0;   // Set output
    _LATB13 = 1;     // Direction: 1 = CW; 0 = CCW
    
    //Set up Interrupts for counting steps
    _T2IP = 4;          // Select Timer2 interrupt priority (4 is default)
    _T2IE = 1;          // Enable Timer2 interrupt
    _T2IF = 0;          // Clear Timer2 interrupt flag
    
    
}

void stepper_out(int stepsize, char direction, int speed)
{
    //Turn off sleep
    _LATB7 = 1;
    
    //Set Step Size (Reciprocal of Step: 2 = 1/2 step, 8 = 1/8 step)
    switch (stepsize){
        case 1:
            _LATA2 = 0;
            _LATA3 = 0;
            break;
        case 2:
            _LATA2 = 1;
            _LATA3 = 0;
            break;
        case 8:
            _LATA2 = 0;
            _LATA3 = 1;
            break;
        case 16:
            _LATA2 = 1;
            _LATA3 = 1;
            break;
        default:
            _LATA2 = 0;
            _LATA3 = 0;
            break;
    }
     // Set direction 1 = CW; 0 = CCW
    int LDIR, RDIR;
    switch (direction) {
        case 'L': //Left
            LDIR = 0;
            RDIR = 1;
            break;
        case 'R': //Right
            LDIR = 1;
            RDIR = 0;
            break;
        case 'F': //Forward
            LDIR = 1;
            RDIR = 1;
            break;
        case 'B': //Backward
            LDIR = 0;
            RDIR = 0;
            break;
        default: //Forward
            LDIR = 1;
            RDIR = 1;
    }
    _LATB12 = LDIR;
    _LATB13 = RDIR;
    
    double fmax = 400;
    double freq;
    double DC = 50.0; // Default Duty Cycle
    switch (speed){
        case 0:
            freq = fmax;
            DC = 0;
            break;
        case 1:
            freq = fmax/4.0;
            break;
        case 2:
            freq = fmax/2.0;
            break;
        case 3:
            freq = fmax*3.0/4.0;
            break;
        case 4:
            freq = fmax;
            break;
        default:
            freq = 0;
            break;
    }
    freq *= stepsize;
    
    if (freq > 1000)
        freq = 1000;
    
    TMR2 = 0; // Reset timer
    PWM_Out(freq,DC,64,1);
    
}

void stepper_out_ramp(int stepsize, char direction, int freq)
{
    //Turn off sleep
    _LATB7 = 1;
    
    //Set Step Size (Reciprocal of Step: 2 = 1/2 step, 8 = 1/8 step)
    switch (stepsize){
        case 1:
            _LATA2 = 0;
            _LATA3 = 0;
            break;
        case 2:
            _LATA2 = 1;
            _LATA3 = 0;
            break;
        case 8:
            _LATA2 = 0;
            _LATA3 = 1;
            break;
        case 16:
            _LATA2 = 1;
            _LATA3 = 1;
            break;
        default:
            _LATA2 = 0;
            _LATA3 = 0;
            break;
    }
     // Set direction 1 = CW; 0 = CCW
    int LDIR, RDIR;
    switch (direction) {
        case 'L': //Left
            LDIR = 0;
            RDIR = 1;
            break;
        case 'R': //Right
            LDIR = 1;
            RDIR = 0;
            break;
        case 'F': //Forward
            LDIR = 1;
            RDIR = 1;
            break;
        case 'B': //Backward
            LDIR = 0;
            RDIR = 0;
            break;
        default: //Forward
            LDIR = 1;
            RDIR = 1;
    }
    _LATB12 = LDIR;
    _LATB13 = RDIR;
    
    double fmax = 400;
    double DC = 50.0; // Default Duty Cycle
    if (freq == 0){
        freq = fmax;
        DC = 0;
    }
    freq *= stepsize;
    
    PWM_Out(freq,DC,64,1);
    
}