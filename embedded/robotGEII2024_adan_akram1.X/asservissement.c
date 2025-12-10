#include "Utilities.h"
#include "asservissement.h"
#include "UART_Protocol.h"
#include "Robot.h"
#include "QEI.h"
#include "PWM.h"
#include "GhostManager.h"
#include "math.h"

extern volatile GhostPosition ghostPosition;
 
 void SetupPidAsservissement(volatile PidCorrector* PidCorr, float Kp, float Ki, float Kd, float proportionelleMax, float integralMax, float deriveeMax)
{
    PidCorr->Kp = Kp;
    PidCorr->erreurProportionelleMax = proportionelleMax; //On limite la correction due au Kp
    PidCorr->Ki = Ki;
    PidCorr->erreurIntegraleMax = integralMax; //On limite la correction due au Ki
    PidCorr->Kd = Kd;
    PidCorr->erreurDeriveeMax = deriveeMax;
}
 
 
 
//float Correcteur(volatile PidCorrector* PidCorr, float erreur)
//{
//    PidCorr->erreur = erreur;
//    float erreurProportionnelle = LimitToInterval(erreur, -PidCorr->erreurProportionelleMax / PidCorr->Kp, PidCorr->erreurProportionelleMax / PidCorr->Kp);
//    PidCorr->corrP = PidCorr->Kp * erreurProportionnelle;
//    PidCorr->erreurIntegrale += PidCorr->erreur/FREQ_ECH_QEI;
//    if(PidCorr->Ki<=0)
//        PidCorr->erreurIntegrale = 0;
//    else
//        PidCorr->erreurIntegrale = LimitToInterval(PidCorr->erreurIntegrale + erreur/FREQ_ECH_QEI, -PidCorr->erreurIntegraleMax/ PidCorr->Ki, PidCorr->erreurIntegraleMax/ PidCorr->Ki);
//    PidCorr->corrI = PidCorr->Ki * PidCorr->erreurIntegrale;
//    float erreurDerivee = (erreur - PidCorr->epsilon_1)*FREQ_ECH_QEI;
//    float deriveeBornee = LimitToInterval(erreurDerivee, -PidCorr->erreurDeriveeMax/PidCorr->Kd, PidCorr->erreurDeriveeMax/PidCorr->Kd);
//    PidCorr->epsilon_1 = erreur;
//    PidCorr->corrD = deriveeBornee * PidCorr->Kd;
//    
//    return PidCorr->corrP+PidCorr->corrI+PidCorr->corrD;
//}
 
 double Correcteur(volatile PidCorrector* PidCorr, float erreur) {
    PidCorr->erreur = erreur;
    double erreurProportionnelle;
    if(PidCorr->Kp!=0)
        erreurProportionnelle = LimitToInterval(erreur, -PidCorr->erreurProportionelleMax / PidCorr->Kp, PidCorr->erreurProportionelleMax / PidCorr->Kp);
    else
        erreurProportionnelle = erreur;
    PidCorr->corrP = erreurProportionnelle * PidCorr->Kp;
    
    PidCorr->erreurIntegrale += erreur / FREQ_ECH_QEI;
    if(PidCorr->Ki!=0)
        PidCorr->erreurIntegrale = LimitToInterval(PidCorr->erreurIntegrale, -PidCorr->erreurIntegraleMax / PidCorr->Ki, PidCorr->erreurIntegraleMax / PidCorr->Ki);
    else
        PidCorr->erreurIntegrale = erreur;
    PidCorr->corrI = PidCorr->erreurIntegrale * PidCorr->Ki;
    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
       
    double deriveeBornee;
    if(PidCorr->Kd!=0)
        deriveeBornee = LimitToInterval(erreurDerivee, -PidCorr->erreurDeriveeMax / PidCorr->Kd, PidCorr->erreurDeriveeMax / PidCorr->Kd);
    else
        deriveeBornee = erreurDerivee;
    PidCorr->epsilon_1 = erreur;
    PidCorr->corrD = deriveeBornee * PidCorr->Kd;

    return PidCorr->corrP + PidCorr->corrI + PidCorr->corrD;
}

void UpdateAsservissement(){
    
    robotState.PidX.erreur = robotState.consigneVitesseLineaire - robotState.vitesseLineaireFromOdometry;
    robotState.PidTheta.erreur = robotState.consigneVitesseAngulaire - robotState.vitesseAngulaireFromOdometry;
    
    robotState.correctionVitesseLineaire = Correcteur(&robotState.PidX, robotState.PidX.erreur);
    robotState.correctionVitesseAngulaire = Correcteur(&robotState.PidTheta, robotState.PidTheta.erreur);
    
    robotState.PdTheta.erreur = ghostPosition.theta - robotState.angleRadianFromOdometry;    
    // robotState.correctionVitesseAngulaire += Correcteur(&robotState.PdTheta, robotState.PdTheta.erreur);
    
    double normeGhost = sqrt(ghostPosition.x * ghostPosition.x + ghostPosition.y * ghostPosition.y);
    double normeOdo = sqrt(robotState.xPosFromOdometry * robotState.xPosFromOdometry + robotState.yPosFromOdometry * robotState.yPosFromOdometry);
    
    robotState.PdLin.erreur = normeGhost - normeOdo;
    // robotState.correctionVitesseLineaire += Correcteur(&robotState.PdLin, robotState.PdLin.erreur);
    
    robotState.vitesseDroiteConsigne = -COEF_VITESSE_POURCENT * (robotState.correctionVitesseLineaire + (robotState.correctionVitesseAngulaire * DISTROUES/2));
    robotState.vitesseGaucheConsigne = COEF_VITESSE_POURCENT * (robotState.correctionVitesseLineaire - (robotState.correctionVitesseAngulaire * DISTROUES/2));
    
    
}

void sendPID(int codeFunction){
    unsigned char msg[48];
    getBytesFromFloat(msg, 0, robotState.PidX.Kp);
    getBytesFromFloat(msg, 4, robotState.PidX.Ki);
    getBytesFromFloat(msg, 8, robotState.PidX.Kd);
    getBytesFromFloat(msg, 12, robotState.PidX.erreurProportionelleMax);
    getBytesFromFloat(msg, 16, robotState.PidX.erreurIntegraleMax);
    getBytesFromFloat(msg, 20, robotState.PidX.erreurDeriveeMax);   
    
    getBytesFromFloat(msg, 24, robotState.PidTheta.Kp);
    getBytesFromFloat(msg, 28, robotState.PidTheta.Ki);
    getBytesFromFloat(msg, 32, robotState.PidTheta.Kd);
    getBytesFromFloat(msg, 36, robotState.PidTheta.erreurProportionelleMax);
    getBytesFromFloat(msg, 40, robotState.PidTheta.erreurIntegraleMax);
    getBytesFromFloat(msg, 44, robotState.PidTheta.erreurDeriveeMax);
    
    UartEncodeAndSendMessage(codeFunction, 48, msg);
 }
 
 void sendAsserv(int codeFunction){
    unsigned char msg[88];
    getBytesFromDouble(msg, 0, robotState.PidX.erreur);
    getBytesFromDouble(msg, 4, robotState.correctionVitesseLineaire);
    getBytesFromFloat(msg, 8, robotState.PidX.Kp);
    getBytesFromFloat(msg, 12, robotState.PidX.corrP);
    getBytesFromDouble(msg, 16, robotState.PidX.erreurProportionelleMax);
    getBytesFromDouble(msg, 20, robotState.PidX.Ki);   
    
    getBytesFromDouble(msg, 24, robotState.PidX.corrI);
    getBytesFromDouble(msg, 28, robotState.PidX.erreurIntegraleMax);
    getBytesFromFloat(msg, 32, robotState.PidX.Kd);
    getBytesFromFloat(msg, 36, robotState.PidX.corrD);
    getBytesFromDouble(msg, 40, robotState.PidX.erreurDeriveeMax);
    getBytesFromDouble(msg, 44, robotState.PidTheta.erreur);
    
    getBytesFromFloat(msg, 48, robotState.correctionVitesseAngulaire); //Commande linéaire
    getBytesFromFloat(msg, 52, robotState.PidTheta.Kp); // commande polaire
    getBytesFromFloat(msg, 56, robotState.PidTheta.corrP);//à vérifier
    getBytesFromFloat(msg, 60, robotState.PidTheta.erreurProportionelleMax);//à vérifier
    getBytesFromFloat(msg, 64, robotState.PidTheta.Ki);
    getBytesFromFloat(msg, 68, robotState.PidTheta.corrI);
    
    getBytesFromFloat(msg, 72, robotState.PidTheta.erreurIntegraleMax);
    getBytesFromFloat(msg, 76, robotState.PidTheta.Kd);
    getBytesFromFloat(msg, 80, robotState.PidTheta.corrD);
    getBytesFromFloat(msg, 84, robotState.PidTheta.erreurDeriveeMax);

    UartEncodeAndSendMessage(codeFunction, 88, msg);
 }



