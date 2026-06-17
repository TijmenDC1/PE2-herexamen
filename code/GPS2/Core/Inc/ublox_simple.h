#ifndef UBLOX_SIMPLE_H
#define UBLOX_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>

// De essentiële UBX-NAV-PVT structuur (exact overgenomen uit iNav)
typedef struct __attribute__((packed)) {
    uint32_t time;              // GPS msToW
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fix_type;           // 0=No fix, 2=2D, 3=3D
    uint8_t fix_status;
    uint8_t reserved1;
    uint8_t satellites;         // Aantal satellieten
    int32_t longitude;          // 1e-7 graden
    int32_t latitude;           // 1e-7 graden
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;       // Hoogte boven zeeniveau in mm
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
    int32_t ned_north;          // Snelheid Noord (mm/s)
    int32_t ned_east;           // Snelheid Oost (mm/s)
    int32_t ned_down;           // Snelheid Daal (mm/s)
    int32_t speed_2d;           // Grondsnelheid (mm/s)
    int32_t heading_2d;         // Koers (1e-5 graden)
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
    uint16_t position_DOP;
    uint16_t reserved2;
    uint16_t reserved3;
} ubx_nav_pvt_t;

// De 'state' van onze inkomende datastroom
typedef struct {
    uint8_t step;
    uint8_t msgClass;
    uint8_t msgId;
    uint16_t payloadLength;
    uint16_t payloadCounter;
    uint8_t ckA, ckB; // Checksums

    // Buffer om de inkomende payload in op te slaan
    union {
        ubx_nav_pvt_t pvt;
        uint8_t bytes[sizeof(ubx_nav_pvt_t) + 10];
    } payloadBuffer;

    // De actuele data die je applicatie kan uitlezen
    ubx_nav_pvt_t pvtData;
    bool newDataReady;
} ublox_parser_state_t;

// Functiedeclaraties
void ublox_init(ublox_parser_state_t *state);
bool ublox_process_byte(ublox_parser_state_t *state, uint8_t data);

#endif // UBLOX_SIMPLE_H
