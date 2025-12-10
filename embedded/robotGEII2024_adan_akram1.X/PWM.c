#include <xc.h> 
#include "IO.h"
#include "PWM.h"
#include "Robot.h"
#include "ToolBox.h"
#include "main.h"
#include "QEI.h"

#define PWMPER 24.0 

unsigned char acceleration = 5;
double talon = 20;

void InitPWM(void) {
    PTCON2bits.PCLKDIV = 0b000;
    PTPER = 100 * PWMPER; 
    IOCON1bits.PMOD = 0b11; 
    IOCON1bits.PENL = 1;
    IOCON1bits.PENH = 1;
    FCLCON1 = 0x0003; 
    IOCON2bits.PMOD = 0b11; 
    IOCON2bits.PENL = 1;
    IOCON2bits.PENH = 1;
    FCLCON2 = 0x0003; 
    PTCONbits.PTEN = 1;
}


void PWMSetSpeed(float vitesseEnPourcents) {
    PDC2 = vitesseEnPourcents * PWMPER + talon;
    SDC2 = talon;
}


void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur) {
    if (!moteur) {
        robotState.vitesseGaucheConsigne = vitesseEnPourcents;
    } else if (moteur) {
        robotState.vitesseDroiteConsigne = vitesseEnPourcents;
    }
}

void PWMUpdateSpeed() {
    if (robotState.vitesseDroiteCommandeCourante < robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Min(
            robotState.vitesseDroiteCommandeCourante + acceleration,
            robotState.vitesseDroiteConsigne);
    if (robotState.vitesseDroiteCommandeCourante > robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Max(
            robotState.vitesseDroiteCommandeCourante - acceleration,
            robotState.vitesseDroiteConsigne);
    if (robotState.vitesseDroiteCommandeCourante >= 0) {
        PDC2 = robotState.vitesseDroiteCommandeCourante * PWMPER + talon;
        SDC2 = talon;
    } else {
        PDC2 = talon;
        SDC2 = -robotState.vitesseDroiteCommandeCourante * PWMPER + talon;
    }
    if (robotState.vitesseGaucheCommandeCourante < robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Min(
            robotState.vitesseGaucheCommandeCourante + acceleration,
            robotState.vitesseGaucheConsigne);
    if (robotState.vitesseGaucheCommandeCourante > robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Max(
            robotState.vitesseGaucheCommandeCourante - acceleration,
            robotState.vitesseGaucheConsigne);
    if (robotState.vitesseGaucheCommandeCourante > 0) {
        PDC1 = robotState.vitesseGaucheCommandeCourante * PWMPER + talon;
        SDC1 = talon;
    } else {
        PDC1 = talon;
        SDC1 = -robotState.vitesseGaucheCommandeCourante * PWMPER + talon;
    }
}

void PWMSetSpeedConsignePolaire(double vitesseLineaire, double vitesseAngulaire) {
    robotState.consigneVitesseLineaire = vitesseLineaire;
    robotState.consigneVitesseAngulaire = vitesseAngulaire;
}

