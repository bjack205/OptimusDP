#ifndef STEPPERS_H
#define	STEPPERS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include"PWM.h"

    extern int step;
void config_step1(void);
void stepper_out(int stepsize, char direction, int speed);

#ifdef	__cplusplus
}
#endif

#endif	/* STEPPERS_H */

