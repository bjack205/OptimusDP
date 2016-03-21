#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include"Header.h"
    
void config_ad(void)
{
    _ADON = 0; //Turn off A/D

    // AD1CON1 register
    _ADSIDL = 0;  // AD1CON1<13> -- A/D continues in idle mode
    _MODE12 = 1;  // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;    // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;    // AD1CON1<7:4> -- Auto conversion
			  // (internal counter)
    _ASAM = 1;    // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;   // AD1CON2<15:14> -- Use VDD as positive
                  // ref voltage
    _NVCFG = 0;   // AD1CON2<13> -- Use VSS as negative
                  // ref voltage
    _BUFREGEN = 1; // Result appears in buffer location for channel
    _CSCNA = 1;   // Scan Inputs
    _SMPI = 2;    // Sample every other
    _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only
    
    // AD1CON3 register
    _ADRC = 0;    // AD1CON3<15> -- Use system clock
    _SAMC = 0;    // AD1CON3<12:8> -- Auto sample every A/D
                  // period TAD
    _ADCS = 0x3F; // AD1CON3<7:0> -- A/D period TAD = 64*TCY
    
    // AD1CSSL register
    //AD1CSSL = 1;  
    _CSS0 = 1;
    _CSS1 = 1;
    _CSS4 = 1;
    

    //Turn on AD
    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}



#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

