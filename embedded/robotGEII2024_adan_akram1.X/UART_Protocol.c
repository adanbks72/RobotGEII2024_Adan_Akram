#include <xc.h>
#include "UART_Protocol.h"
#include "CB_TX1.h"
#include <stdint.h>
#include "timer.h"
#include "IO.h"
#include "CB_RX1.h"
#include "main.h"
#include "Utilities.h"
#include "asservissement.h"
#include "Robot.h"
#include "GhostManager.h"

extern volatile GhostPosition ghostPosition;

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* payload) {
    unsigned char message [6 + msgPayloadLength];
    int pos = 0;
    message[pos++] = 0xFE;
    message[pos++] = (unsigned char) (msgFunction >> 8);
    message[pos++] = (unsigned char) (msgFunction);
    message[pos++] = (unsigned char) (msgPayloadLength >> 8);
    message[pos++] = (unsigned char) (msgPayloadLength);
    for (int i = 0; i < msgPayloadLength; i++) {
        message[pos++] = payload[i];
    }
    char c = UartCalculateChecksum(msgFunction, msgPayloadLength, payload);
    message[pos++] = c;
    SendMessage(message, pos);
}

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    unsigned char c = 0;
    c ^= 0xFE;
    c ^= (unsigned char) (msgFunction >> 8);
    c ^= (unsigned char) msgFunction;
    c ^= (unsigned char) (msgPayloadLength >> 8);
    c ^= (unsigned char) msgPayloadLength;
    for (int n = 0; n < msgPayloadLength; n++)
        c ^= msgPayload[n];
    return c;
}

unsigned char rcvState = STATE_ATTENTE;
int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex = 0;
unsigned char calculatedChecksum;
unsigned char receivedChecksum;

void UartDecodeMessage(unsigned char c) {
    //Fonction prenant en entree un octet et servant a reconstituer les trames

    switch (rcvState) {
        case STATE_ATTENTE:
            if (c == 0xFE)
                rcvState = STATE_FUNCTION_MSB;
            break;
        case STATE_FUNCTION_MSB:
            msgDecodedFunction = c << 8;
            rcvState = STATE_FUNCTION_LSB;
            break;
        case STATE_FUNCTION_LSB:
            msgDecodedFunction |= c;
            rcvState = STATE_PAYLOAD_LENGTH_MSB;
            break;
        case STATE_PAYLOAD_LENGTH_MSB:
            msgDecodedPayloadLength = c << 8;
            rcvState = STATE_PAYLOAD_LENGTH_LSB;
            break;
        case STATE_PAYLOAD_LENGTH_LSB:
            msgDecodedPayloadLength |= c;
            if (msgDecodedPayloadLength < 1024) {
                if (msgDecodedPayloadLength > 0) {
                    rcvState = STATE_PAYLOAD;
                } else {
                    rcvState = STATE_CHECKSUM;
                }
            } else {
                rcvState = STATE_ATTENTE;
            }
            break;
        case STATE_PAYLOAD:
            if (msgDecodedPayloadIndex <= msgDecodedPayloadLength) {
                msgDecodedPayload[msgDecodedPayloadIndex] = c;
                if (++msgDecodedPayloadIndex >= msgDecodedPayloadLength) {
                    rcvState = STATE_CHECKSUM;
                    msgDecodedPayloadIndex = 0;
                }

            }
            break;
        case STATE_CHECKSUM:
            calculatedChecksum = c;

            receivedChecksum = UartCalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            if (calculatedChecksum == receivedChecksum) {
                //Success, on a un message valide
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            } else {
                //print("Les checksums sont différents");
            }
            rcvState = STATE_ATTENTE;
            break;
        default:
            rcvState = STATE_ATTENTE;
            break;
    }
}

void Uart2DecodeMessage(unsigned char c) {
    
}


int rcvFunction;

void UartProcessDecodedMessage(int rcvFunction, int payloadLength, unsigned char* payload) {
    switch (rcvFunction) {
        case CONFIG_PIDX:
            SetupPidAsservissement(&robotState.PidX, getFloat(payload, 0),  getFloat(payload, 4),  
                     getFloat(payload, 8), getFloat(payload, 12), getFloat(payload, 16), getFloat(payload, 20));
            break;
        case CONFIG_PIDTheta:
            SetupPidAsservissement(&robotState.PidTheta, getFloat(payload, 0),  getFloat(payload, 4),  
                     getFloat(payload, 8), getFloat(payload, 12), getFloat(payload, 16), getFloat(payload, 20));
            break;
            
        case CONFIG_VLINEAIRE:
            robotState.consigneVitesseLineaire = getFloat(payload, 0);
            break;
            
        case CONFIG_VANGULAIRE:
            robotState.consigneVitesseAngulaire = getFloat(payload, 0);
            break;
        
        case SET_GHOST_POSITION:
            ghostPosition.targetX = getFloat(payload, 0);
            ghostPosition.targetY = getFloat(payload, 4);
            break;
            
        default:
            break;

    }

}

void robotStateChange(unsigned char rbState ) {
    unsigned char msg[5];
    int position = 0;
    unsigned long tstamp = timestamp;
    msg[position++] = rbState;
    msg[position++] = (unsigned char) (tstamp >> 24);
    msg[position++] = (unsigned char) (tstamp >> 16);
    msg[position++] = (unsigned char) (tstamp >> 8);
    msg[position++] = (unsigned char) tstamp;

    UartEncodeAndSendMessage(0x0050, 5, msg);
}