#ifndef CAN_H
#define CAN_H

void CAN_Init( void );
void CAN_Transmit( uint32_t id, uint8_t* data, uint8_t len );

#endif // CAN_H