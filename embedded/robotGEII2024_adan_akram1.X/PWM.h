#ifndef PWM_H
#define	PWM_H

#define MOTEUR_DROIT 0
#define MOTEUR_GAUCHE 1

void InitPWM(void);
void PWMUpdateSpeed();
void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur);
void PWMSetSpeedConsignePolaire(double vitesseLineaire, double vitesseAngulaire);

#endif	/* PWM_H */
