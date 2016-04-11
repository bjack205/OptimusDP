
#include "Header.h"
#include "Steppers.h"
#include "Config.h"
#include "IRSensors.h"

_FOSCSEL(FNOSC_FRC & SOSCSRC_DIG) // 8 MHz Declare oscillator and Turn off Secondary oscillator on Pins 9 and 10
_FICD(ICS_PGx1) //Set Debug Pins
_FOSC(OSCIOFNC_OFF)//Turn off CLK output on Pin 8
        
// Global Variables
volatile int runtime = 0; //seconds
volatile int gtime = 0; //mils
int step = 0;
int step_target = 0;
int start_button = 0;
float VThresh_Front = 2.8;
float VThresh_Side = 2.5;
int failsafe_step = 100;

//State Variables
typedef enum{start,fliparound, FindHome, Reorient, tohome, reload, tocenter, wait, findgoal, launch, IRchange, test, test2} statedef;
statedef state = start; //FindHome
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
    if (gtime >= 1000){
        runtime++;
        gtime = 0;
    }
    
    
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    _T2IF = 0;
}

double GetTime(){
    return (double)(runtime)*1000.0+(double)(gtime);
}

//**************************************MOBILITY CODE********************************************
void StepperSleep(int sleep){
    if (sleep)
        _LATB7 = 0; //Sleep Motor
    else
        _LATB7 = 1; //Activate Motor
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
                turntime = GetTime();
            }          
            break;
        case 2: //Holding
            if (GetTime()-turntime > 500){
                turnstate = 0;
                _LATB7 = 0; // Sleep Motor
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
                holdtime = GetTime();
            }          
            break;
        case 2: //Holding
            if (GetTime()-holdtime > 500){
                drivestate = 0;
                _LATB7 = 0; // Sleep Motor
                return 1;
            }
            
    }
    return 0;
}

int RampTurn(double angle, char direction, int step_size) {
    static int turnstate = 0;
    static double ramp_speed = 50;
    static int ramp_time = 0;
    int ramp_rate = 2;
    int delay = 3;
    int speed = 200;
    int creepspeed = 50;
    int creepstep = 16; //Use finest step size
    static char creepdir = 'R';
    
    switch (turnstate){
        case 0: //Start
            step = 0;
            TMR2 = 0;
            
            //Validate step size input
            if (step_size != 1 && step_size != 2 && step_size != 8 && step_size != 16)
                step_size = 1; //default to whole step
            
            creepdir = direction;
            if (angle < 0){
                angle = -angle;
                if (direction == 'R') //Reverse Creep Direction
                    creepdir = 'L';
                else
                    creepdir = 'R';
            }

            angle = angle * PI / 180.0; //convert to radians
            step_target = R_base * angle*step_size; // calculate number of steps
            ramp_time = GetTime();
            
            turnstate = 1;
            stepper_out_ramp(step_size, direction, ramp_speed);
            break;
        case 1: //Ramp up Speed
            if (ramp_speed < speed) {
                if (GetTime()-ramp_time > delay){
                    ramp_speed += ramp_rate;
                    ramp_time = GetTime();
                    
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            else { // Write out max speed
                stepper_out_ramp(step_size, direction, speed);
                turnstate = 2;
            }
            if (step > step_target-40*step_size){ //Check if reaching end of cycle
                turnstate = 2;
            }
            break;
        case 2: // Max Speed
            if (step > step_target-40*step_size){
                turnstate = 3;
            }
            break;
        case 3: // Ramp down to stop
            if (StepFinished()){ // Stop
                ramp_time = GetTime();
                turnstate = 4;
                //if (state == findgoal){
                if (state == findgoal){ // Enable Creep
                    turnstate = 5;
                    stepper_out_ramp(creepstep, creepdir, creepspeed);
                }
            }    
            else { 
                if (GetTime()-ramp_time > delay && ramp_speed > 30) {
                    ramp_speed -= ramp_rate+1;
                    ramp_time = GetTime();
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            break;
        case 4: //Hold
            if (GetTime()-ramp_time > 200){
                turnstate = 0;
                ramp_time = 0;
                ramp_speed = 20;
                if (state != findgoal){
                    StepperSleep(1);
                }
                StepperStop();
                T3CONbits.TON = 0;
                return 1;
            }
            break;
        case 5: //Creeping
            // If IR is interrupted (Do in interrupt?)
            
            // TMR2 = 0;
            T3CONbits.TON = 1;
            
            
            if (step > step_target + failsafe_step*creepstep){ // Failsafe
                turnstate = 0;
                ramp_time = 0;
                ramp_speed = 20;
                _LATB7 = 0; // Sleep Motor
                T3CONbits.TON = 0;
                return -1;
                
            }
            if (ADC1BUF4/4095.0 * 3.3 > VThresh_Front) {
                turnstate = 4;
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

int RampDrive(double distance, char direction, int step_size) {
    static int drivestate = 0;
    static double ramp_speed = 20;
    static int ramp_time = 0;
    int ramp_rate = 1;
    int delay = 2;
    int speed = 200;
    int crashspeed = 100;
    
    switch (drivestate){
        case 0: //Start
            step = 0;
            TMR2 = 0;
            //Validate step size input
            if (step_size != 1 && step_size != 2 && step_size != 8 && step_size != 16)
                step_size = 1; //default to whole step

            step_target = distance*step_size; // calculate number of steps
            ramp_time = GetTime();
            
            drivestate = 1;
            break;
        case 1: //Ramp up Speed
            if (ramp_speed < speed) {
                if (GetTime()-ramp_time > delay){
                    ramp_speed += ramp_rate;
                    ramp_time = GetTime();
                    
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
                    drivestate = 3;
            }
            break;
        case 3: // Ramp down to stop
            if (StepFinished()){ // Stop
                ramp_time = GetTime();
                drivestate = 4;
                if (state == tohome){
                    drivestate = 5;
                    ramp_time = GetTime();
                    stepper_out_ramp(step_size, direction, crashspeed);
                }
            }    
            else { 
                if (GetTime()-ramp_time > delay && ramp_speed > crashspeed) {
                    ramp_speed -= ramp_rate+1;
                    ramp_time = GetTime();
                    stepper_out_ramp(step_size, direction, ramp_speed);
                }
            }
            break;
        case 4: //Hold
            if (GetTime()-ramp_time > 200){
                drivestate = 0;
                ramp_time = 0;
                ramp_speed = 20;
                StepperStop();
                if (state != tohome)
                    _LATB7 = 0; // Sleep Motor
                return 1;
            }
            break;
        case 5:
            if (WallContact()){
                ramp_time = GetTime();
                drivestate = 4;
            }
            if (GetTime()-ramp_time > 5000){ //Five second failsafe
                ramp_time = GetTime();
                drivestate = 4;
            }   
            break;         
    }
    return 0;
}

//*************************************OTHER CODE**************************************************

int Start_Check() {
    int value = 0;
    if (_RB14 == 1) {
        if (start_button == 0) { // Button Pressed 
            value = 1;
            gtime = 0;
            runtime = 0;
        }
        start_button = 1;
    }
    else {
        start_button = 0;
    }
    return value;
    
 }

//****************************************LAUNCH CODE******************************************
void MotorControl(float percent){
    PWM_Out(10000,percent,1,3);
}

int Solenoid(double timeON, double timeOFF, int repeat){
    static int SolState = 0;
    static int startTime = 0;
    static int repeat_count = 1;
    switch (SolState){
        case 0: //New
            startTime = GetTime();
            SolState = 1;
            _LATB8 = 1;
            break;
        case 1: // Activated
            if (GetTime() - startTime > timeON){ //On time finished
                _LATB8 = 0; // Turn off
                SolState = 2;
            }
            break;
        case 2: //Off
            if (GetTime() - startTime > timeON + timeOFF) {
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
int CollectBalls(double angle1, double angle2, int num){
    static int servostate = 0;
    static int startTime = 0;
    static int repeatCount = 1;
    
    int timeON = 500;
    int timeOFF = 200;
    switch (servostate){
        case 0:
            _LATB7 = 1;
            startTime = GetTime();
            servostate = 1;
            ServoControl(angle1);//Initial Angle
            break;
        case 1:
            if (GetTime() - startTime > timeON){ //On time finished
                ServoControl(angle2); //Interrupt Angle
                startTime = GetTime(); 
                servostate = 2;
            }
            break;
        case 2: //Interupting Beam
            if ((GetTime() - startTime) > (timeON + timeOFF)) {
                if (repeatCount >= num){
                    servostate = 3; //Exit
                }
                else {
                    servostate = 0; // Repeat
                }
                repeatCount++;
            }
            break;
        case 3: // End
            if (GetTime() - startTime > 500) {
                servostate = 0;
                repeatCount = 1;
                T3CONbits.TON = 0;
                _LATB7 = 0;
                return 1;
            }
    }
    return 0;     
}

int Launch(){
    static int stime = 0;
    static int launchstate = 0;
    static int waittime = 0;
    switch (launchstate){
        case 0:
            StepperSleep(0);
            stime = GetTime();
            MotorControl(85);
            launchstate = 1;
            waittime = GetTime();
            break;
        case 1: //Release balls
            if (GetTime() - waittime > 250) {
                if (Solenoid(200, 200, 6)) {
                    launchstate = 2;
                    waittime = GetTime();
                }
            }
            break;
        case 2: //Turn off motor
            if (GetTime() - waittime>500){
                MotorControl(0);
                launchstate = 0;
                stime = 0;
                StepperSleep(1);
                return 1;
            }
            
    }
    return 0;
}

//********************************************IR CODE***********************************************
int ReadIR() {
    //static char IRState = 'H'; // H = Home, S = Search for IR, F = Found IR and hone in
    float IRThreshold = VThresh_Side;
    float VoltageFront = 0; //(ADC1BUF4 / 4095.0) * 3.3;
    float VoltageLeft = (ADC1BUF0 / 4095.0) * 3.3;
    float VoltageRight = (ADC1BUF1 / 4095.0) * 3.3;
    
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

int LocateDispenser(){
    static int locatestate = 0;
    float VoltageFront;
    switch (locatestate ){
        case 0:
            locatestate = 1;
            break;
        case 1: //Turn Robot
            VoltageFront = ADC1BUF4 / 4095.0 *3.3;
            if (VoltageFront > VThresh_Front){
                locatestate = 2;
            }
            break;
        case 2:
            StepperStop();
            locatestate = 0;
            return 1;
    }
    return 0;
}

//***************************MAIN FUNCTION******************************************
int main () {
    TMR1_Config();
    config_ad();
    config_IR();
    config_step1();
    PWM2_Config(8);
    PWM3_Config(1);
    ServoControl(90);
    _TRISA4 = 0;
    
    //Configure Pin 9
    _ANSB4 = 0; // Set Pin 9 as digital
    _TRISB4 = 0; // Set up Pin 9 as output
    
    //Configure Left Button on Pin 18
    _ANSB15 = 0; // Set Pin 18 as digital
    _TRISB15 = 1; // Set Pin 18 as input
    
    //Configure Right Button on Pin 17
    _ANSB14 = 0;
    _TRISB14 = 1;
    
    //Configure Launch on Pin 5
    _ANSB1 = 0;
    _TRISB1 = 1;
    MotorControl(0);
    
    _TRISB8 = 0;
    
    
    int counter = 0;
    int done = 0;
    
    
    state = test;
    int speed = 3;
    int turn_value = 0;
    
    //_LATB1 = 1;
    //_LATB8 = 1;
    float VoltageFront = 0;
    
    while(1) {
        switch (state) {
            case test:
                if (Start_Check()){
                    orientation = forward;
                    //StepperStop();
                    state = FindHome;
                    //ServoControl(90);
                }
                else {
                    
                }
                break;
            case test2:
                if (_RB14){
                    MotorControl(100);
                }
                else{
                    MotorControl(0);
                }
                break;
            case FindHome:
                //if (Start_Check()) {
                    TurnRobot(360 * 2, 'R', 1, 8);
                    VoltageFront = ADC1BUF4 / 4095.0 * 3.3;
                    if (VoltageFront > VThresh_Front) {
                        StepperStop();
                        _LATB7 = 1;
                        state = fliparound;
                    } else {
                    //_LATB7 = 0;
                }
                //}
                break;
            case fliparound:
                if (RampTurn(180,'R',8)){
                    state = tohome;
                }
                break;
            case Reorient:
                switch (orientation){
                    case forward:
                        done = 1;
                        break;
                    case left:
                        if (RampTurn(90,'R',8))
                            done = 1;
                        break;
                    case right:
                        if (RampTurn(90,'L',8))
                            done = 1;
                        break;
                    case backwards:
                        if (RampTurn(180,'L',8)){
                            done = 1;
                        }
                        break;
                    default:
                        done = 1;
                        break;
                }
                if (done){
                    orientation = forward;
                    //state = test;
                    state = tohome;
                    done = 0;
                    gtime = 0;
                    runtime = 0;
                }
                
                break;
            case tohome:
                if (RampDrive(550,'B',8)){
                    state = reload;
                }
                break;
            case reload:
                if(CollectBalls(90-30,90,3)){
                    state = tocenter;
                    _LATB7 = 0; // Sleep Motor
                }
                break;
            case tocenter:
                if (RampDrive(670,'F',8)){ //670
                    if (GetTime() <= 15.5*1000){
                        state = wait;
                    } 
                    else {
                        goal = ReadIR();
                        state = findgoal;
                    }
                }
                break;
            case wait:
                if (GetTime() > 15.5*1000){
                    goal = ReadIR();
                    state = findgoal;
                }
                break;
            case findgoal:
                switch (goal){
                    case -1: //Forward
                        turn_value = RampTurn(-15,'R',8);
                        if (turn_value == 1) { //Turn Right and then hone left
                            state = launch;
                            orientation = forward;
                        }
                        else if (turn_value == -1){
                            state = IRchange;
                            orientation = forward;
                        }
                        break;
                    case 1: //Left Goal
                        turn_value = RampTurn(85,'L',8);
                        if (turn_value == 1) {
                                state = launch;
                                orientation = left;
                            }
                        else if (turn_value == -1){
                            state = IRchange;
                            orientation = left;
                        }
                        break;
                    case 2: //Right Goal
                        turn_value = RampTurn(85,'R',8);
                        if (turn_value == 1) {
                            state = launch;    
                            orientation = right;
                            }
                        else if (turn_value == -1){
                            state = IRchange;
                            orientation = right;
                        }
                        break;
                    default:
                        state = test;
                }
                break;
            
            case launch:
                if (Launch()){
                    //state = test;
                    state = Reorient;
                }
                break;
            
            case IRchange:
                switch (orientation){
                    case forward:
                        if (RampTurn(failsafe_step/R_base*(180/PI)-15,'R',16)){
                            done = 1;
                        }
                        break;
                    case left:
                        if (RampTurn(failsafe_step/R_base*(180/PI)+85,'R',16)){
                            done = 1;
                        }
                        break;
                    case right:
                        if (RampTurn(failsafe_step/R_base*(180/PI)+85,'L',16)){
                            done = 1;
                        }
                        break;
                }
                if (done){
                    goal = ReadIR();
                    state = findgoal;
                    done = 0;
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
            default:
                state = start;
        }
    }
    return 0;
}



