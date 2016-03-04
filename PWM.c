#include"PWM.h"

void PWM1_Config(int PS){
    //Set up PWM on OC1 (Pin 14) with Timer 2
    
    //Configure Timer 2
    T2CONbits.TON = 0; // Turn on Timer 2
    T2CONbits.TCKPS = PS_bit(PS); //8 pre-scaling
    T2CONbits.TCS = 0; // Select Internal Osc
    T2CONbits.T32 = 0; //16 bit timer
    
    //Sets up Output Compare Module
    OC1CON2bits.SYNCSEL = 0b01100; //Sync with Timer 2
    OC1CON2bits.OCTRIG = 0;
    
    //Select Clock Source
    OC1CON1bits.OCTSEL  = 0b000; //Select Timer 2
    OC1CON1bits.OCM = 0b110; //Edge-aligned
    
    T2CONbits.TON = 1; 
}

void PWM2_Config(int PS){
    //Set up PWM on OC2 (Pin 4) with Timer 3
    
    //Configure Timer 3
    T3CONbits.TON = 0; //Turn off Timer 3
    T3CONbits.TCKPS = PS_bit(PS); //No pre-scaling
    T3CONbits.TCS = 0; // Select Internal Osc
    
     //Sets up Output Compare Module
    OC2CON2bits.SYNCSEL = 0b01101; //Sync with Timer 3
    OC2CON2bits.OCTRIG = 0;
    
    //Select Clock Source
    OC2CON1bits.OCTSEL  = 0b001; //Select Timer 3
    OC2CON1bits.OCM = 0b110; //Edge-aligned
    
    //Turn on Timer 3
    T3CONbits.TON = 1; 
}

void PWM3_Config(int PS){
    //Set up PWM on OC3 (Pin 5) with Timer 4
    
    //Configure Timer 4
    T4CONbits.TON = 0; // Turn on Timer 4
    T4CONbits.TCKPS = PS_bit(PS); //8 pre-scaling
    T4CONbits.TCS = 0; // Select Internal Osc
    T4CONbits.T32 = 0; //16 bit timer
    
    //Sets up Output Compare Module
    OC3CON2bits.SYNCSEL = 0b01110; //Sync with Timer 4
    OC3CON2bits.OCTRIG = 0;
    
    //Select Clock Source
    OC3CON1bits.OCTSEL  = 0b010; //Select Timer 4
    OC3CON1bits.OCM = 0b110; //Edge-aligned
    
    T4CONbits.TON = 1; 
}

int PS_bit(int PS){
    switch (PS) {
        case 1:
            return 0b00;
        case 8:
            return 0b01;
        case 64:
            return 0b10;
        case 256:
            return 0b11;
        default:
            return 0b00;
    }
}

void PWM_Out(double Freq, double DC, int PS, int Ch){
    double Tcy = 1.0/Fcy; // Operating Period
    double Tpwm = 1.0/Freq;
    
    DC /= 100.0;
    
    //Write ON time
    double Ton_ticks = (Tpwm/(Tcy*PS)-1)*DC; 
    
    //Write Period
    double Tperiod = (Tpwm/(Tcy*PS)-1);
    
    switch (Ch){
        case 1:
            OC1R = Ton_ticks;
            PR2 = Tperiod;
            T2CONbits.TCKPS = PS_bit(PS);
        case 2:
            OC2R = Ton_ticks;
            PR3 = Tperiod;
            T3CONbits.TCKPS = PS_bit(PS);
        case 3:
            OC3R = Ton_ticks;
            PR4 = Tperiod;
            T4CONbits.TCKPS = PS_bit(PS);
    }
    
}


void VOut1(double voltage) {
    //Output voltage to OC1 (Pin 14)
    
    //Configurable Params
    double Fpwm = 1000;
    int PS = 1; //Pre-scaling
    
    double DC = voltage/Vref;
    double Tcy = 1.0/Fcy;
    double Tpwm = 1.0/Fpwm;
    
    //Write ON time
    double Ton = (Tpwm/(Tcy*PS)-1)*DC;
    OC1R = Ton;
    
    //Write PWM Period
    double Tperiod = (Tpwm/(Tcy*PS)-1);
    PR2 = Tperiod;
}

void VOut2(double voltage) {
    //Output voltage to OC1 (Pin 14)
    
    //Configurable Params
    double Fpwm = 1000;
    int PS = 1; //Pre-scaling
    
    double DC = voltage/Vref;
    double Tcy = 1.0/Fcy;
    double Tpwm = 1.0/Fpwm;
    
    //Write ON time
    double Ton = (Tpwm/(Tcy*PS)-1)*DC;
    OC2R = Ton;
    
    //Write PWM Period
    double Tperiod = (Tpwm/(Tcy*PS)-1);
    PR3 = Tperiod;
}