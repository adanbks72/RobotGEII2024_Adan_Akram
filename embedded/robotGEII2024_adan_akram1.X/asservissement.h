/* 
 * File:   asservissement.h
 * Author: E306_PC2
 *
 * Created on 5 mai 2025, 16:21
 */

#ifndef PID_H
#define	PID_H

typedef struct _PidCorrector
{
    float Kp;
    float Ki;
    float Kd;
    float erreurProportionelleMax;
    float erreurIntegraleMax;
    float erreurDeriveeMax;
    float erreur;
    float erreurIntegrale;
    float corrP;
    float corrI;
    float epsilon_1;
    float corrD;
}PidCorrector;

void sendPID(int codeFunction);
void sendAsserv(int codeFunction);
void SetupPidAsservissement(volatile PidCorrector* PidCorr, float Kp, float Ki, float Kd, float proportionelleMax, float integralMax, float deriveeMax);
void UpdateAsservissement();

#endif	/* PID_H */



