/*
 * ublox_simple.c
 *
 *  Created on: 6 apr 2026
 *      Author: simon
 */


#include "ublox_simple.h"

void ublox_init(ublox_parser_state_t *state) {
	//printf("Init van ublox\n");
    state->step = 0;
    state->newDataReady = false;
}

bool ublox_process_byte(ublox_parser_state_t *state, uint8_t data) {
    bool parsed = false;

    switch (state->step) {
        case 0: // Sync char 1 (0xB5)
            if (data == 0xB5) state->step++;
            break;
        case 1: // Sync char 2 (0x62)
            if (data == 0x62) state->step++;
            else state->step = 0;
            break;
        case 2: // Class
            state->msgClass = data;
            state->ckA = state->ckB = data; // Checksum reset
            state->step++;
            break;
        case 3: // ID
            state->msgId = data;
            state->ckA += data; state->ckB += state->ckA;
            state->step++;
            break;
        case 4: // Payload Length (Low Byte)
            state->payloadLength = data;
            state->ckA += data; state->ckB += state->ckA;
            state->step++;
            break;
        case 5: // Payload Length (High Byte)
            state->payloadLength |= (uint16_t)(data << 8);
            state->ckA += data; state->ckB += state->ckA;

            // Controleer of de lengte logisch is om buffer overflows te voorkomen
            if (state->payloadLength > sizeof(state->payloadBuffer)) {
                state->step = 0;
                break;
            }
            state->payloadCounter = 0;
            state->step = (state->payloadLength == 0) ? 7 : 6;
            break;
        case 6: // Payload uitlezen
            state->ckA += data; state->ckB += state->ckA;
            state->payloadBuffer.bytes[state->payloadCounter] = data;
            state->payloadCounter++;
            if (state->payloadCounter == state->payloadLength) {
                state->step++;
            }
            break;
        case 7: // Checksum A
            if (state->ckA != data) state->step = 0; // Foute checksum
            else state->step++;
            break;
        case 8: // Checksum B
            if (state->ckB == data) {
                // Succesvolle ontvangst! Is dit de PVT data (Class 0x01, ID 0x07)?
                if (state->msgClass == 0x01 && state->msgId == 0x07) {
                    state->pvtData = state->payloadBuffer.pvt;
                    state->newDataReady = true;
                    parsed = true;
                }
            }
            state->step = 0; // Reset voor het volgende bericht
            break;
    }
    return parsed;
}
