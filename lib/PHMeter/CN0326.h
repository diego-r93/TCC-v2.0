#ifndef CN0326_H_
#define CN0326_H_

void CN0326_Init(void);
float CN0326_CalculateTemp(void);
float CN0326_CalculatePH(void);

float CN0326_CalculateInternalTemp(void);
float CN0326_CalculateAVDD(void);

/******************************* Internal defines ******************************/

#define YES 1
#define NO 2

#define TEMP_COEFF 0.00385055 /* [Ω/Ω/˚C] - defined by the standard => check the documentation */
#define FARADAY_CONST 96485   /* [Coulombs/mol] -  Faraday constant */
#define PH_CONST 2.303        /* Constant value used in Nernst formula for pH calculation */
#define AVOGADRO_NUMBER 8314  /* With [mV-Coulombs/˚K] - Avogadro number */
#define PH_ISO 7              /* Reference hydrogen ion concentration */
#define K_DEGREES 273.1       /* [˚K] - Kelvin degrees for 0˚C */
#define I_EXC 0.21            /* Excitation current [mA] */

/**************************** Configuration parameters **********************/

#define RMIN 1000 /* Minimum value for RTD resistance */
#define RMAX 1385 /* Maximum value for RTD resistance */
#define TMIN 0    /* Minimum value for RTD temperature */
#define TMAX 100  /* Maximum value for RTD temperature */

#define USE_IOUT2 YES /* Select if you want to use output current from IOUT2 pin (YES) or you want to use default value 210 [uA] of the excitation current */
#define TOLERANCE 0   /* Set a tolerance value for pH calculation */

#endif /* CN0326_H_ */