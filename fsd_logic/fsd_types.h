#pragma once
/*
 * fsd_types.h — hardware-free CAN frame type shared by the protocol logic.
 *
 * Split out of libraries/mcp_can_2515.h so the FSD protocol core
 * (fsd_handler.c) compiles without pulling in the MCP2515 SPI driver or any
 * furi/HAL dependency. This lets the logic be unit-tested on the host (see
 * test/) and is the first step toward a single protocol core shared by the
 * Flipper and ESP32 builds.
 *
 * The CANFRAME layout is unchanged — same fields, same order, same MAX_LEN —
 * so this is behavior-preserving for the existing Flipper build.
 */

#include <stdint.h>

#define MAX_LEN 8

// CAN frame as handled by the MCP2515 driver and the FSD protocol logic.
typedef struct {
    uint32_t canId;
    uint8_t  ext;
    uint8_t  req;
    uint8_t  data_lenght;
    uint8_t  buffer[MAX_LEN];
} CANFRAME;
