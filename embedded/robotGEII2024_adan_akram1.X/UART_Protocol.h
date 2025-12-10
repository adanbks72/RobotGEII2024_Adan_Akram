#ifndef UART_PROTOCOL_H
#define	UART_PROTOCOL_H

#define STATE_ATTENTE 0
#define STATE_FUNCTION_MSB 1
#define STATE_FUNCTION_LSB 2
#define STATE_PAYLOAD_LENGTH_MSB 3
#define STATE_PAYLOAD_LENGTH_LSB 4
#define STATE_PAYLOAD 5
#define STATE_CHECKSUM 6
#define FUNCTION_VITESSE_GAUCHE 0x0041
#define FUNCTION_VITESSE_CENTRE 0x0042
#define SET_ROBOT_STATE 0x0051
#define CONFIG_PIDX 0x0091
#define CONFIG_PIDTheta 0x0092
#define CONFIG_VLINEAIRE 0x0071
#define CONFIG_VANGULAIRE 0x0072
#define SET_GHOST_POSITION 0x0089


unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void robotStateChange(unsigned char rbState);
void UartProcessDecodedMessage(int function, int payloadLength, unsigned char* payload);
void UartDecodeMessage(unsigned char c);
#endif	/* UART_PROTOCOL_H */