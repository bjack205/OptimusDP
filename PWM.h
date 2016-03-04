#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include"Header.h"
    
    // Adding Comment
    
void PWM1_Config();       
void PWM2_Config(); 
void PWM3_Config();
void PWM_Out(double Freq, double DC, int PS, int Ch);

int PS_bit(int PS);
void VOut1(double voltage);
void VOut2(double voltage);


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

