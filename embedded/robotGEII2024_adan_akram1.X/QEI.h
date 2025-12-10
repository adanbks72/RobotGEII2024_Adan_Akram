/* 
 * File:   QEI.h
 * Author: E306_PC2
 *
 * Created on 6 janvier 2025, 13:50
 */

#ifndef QEI_H
#define	QEI_H

void InitQEI1();
void InitQEI2();
void QEIUpdateData();
void SendPositionData();

#define COEF_VITESSE_POURCENT 40
#define DISTROUES 0.216
#define FREQ_ECH_QEI 250
#define POSITION_DATA 0x0061



#endif	/* QEI_H */

