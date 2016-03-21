#include "IRSensors.h"\

void config_IR() {
    
    // turns pins to input
    
    _TRISA0 = 1;    // Front IR  - > ADC1BUF0
    _TRISA1 = 1;    // Left IR - > ADC1BUF1
    _TRISB2 = 1;    // Right IR - > ADC1BUF4
    
    
    // turns pins to analog
    _ANSA0 = 1;
    _ANSA1 = 1;
    _ANSB2 = 1;
    
    
}

