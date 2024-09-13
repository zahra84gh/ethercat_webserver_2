#ifndef CMMT_H
#define CMMT_H

#include <stdint.h>

#define NOT_READY 0x0000
#define SWITCH_ON_DISABLED 0x0040
#define READY_TO_SWITCH_ON 0x0021
#define SWITCHED_ON 0x0023
#define OP_ENABLED 0x0027
#define QUICK_STOP_ACTIVE 0x0007
#define FAULT_REACTION_ACTIVE 0x000F
#define FAULT 0x0008

#define STATUS_READY_TO_SWTICH_ON 0x0001
#define STATUS_SWITCHED_ON 0x0002
#define STATUS_OP_ENABLED 0x0004
#define STATUS_FAULT 0x0008
#define STATUS_CMD_EXECUTED 0x0200
#define STATUS_DRIVE_IS_MOVING 0x0100
#define STATUS_TARGET_REACHED 0x0400 

#define CMD_SWITCH_ON 0x0F
#define CMD_SHUTDOWN 0x06
#define CMD_OP_ENABLE 0x08
#define CMD_OPMODE_01 0x10
#define CMD_OPMODE_02 0x20
#define CMD_OPMODE_03 0x40

#define OPMODE_POSITION 0x01
#define OPMODE_VELOCITY 0x03
#define OPMODE_TORQUE 0x04
#define OPMODE_CYCLIC_POSITION 0x08
#define OPMODE_CYCLIC_VELOCITY 0x09
#define OPMODE_CYCLIC_TORQUE 0x0A


/*
SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0018.0] 0x6041:0x00 0x10 UNSIGNED16   Statusword
  [0x001A.0] 0x6061:0x00 0x08 INTEGER8     Modes of operation display
  [0x001B.0] 0x6064:0x00 0x20 INTEGER32    Position actual value
  [0x001F.0] 0x606C:0x00 0x20 INTEGER32    Velocity actual value
  [0x0023.0] 0x6077:0x00 0x10 INTEGER16    Torque actual value
  [0x0025.0] 0x0000:0x00 0x08

*/

typedef struct __attribute__((__packed__)) {
	uint16_t status;
	int8_t display_op_mode;
	int32_t actual_position;
	int32_t actual_velocity;
	int16_t actual_torque;
  uint8_t padding1;
} cmmt_input_map_t;

/*
SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16   Controlword
  [0x0002.0] 0x6060:0x00 0x08 INTEGER8     Modes of operation
  [0x0003.0] 0x607A:0x00 0x20 INTEGER32    Target position
  [0x0007.0] 0x6081:0x00 0x20 UNSIGNED32   Profile velocity
  [0x000B.0] 0x60FF:0x00 0x20 INTEGER32    Target velocity
  [0x000F.0] 0x6071:0x00 0x10 INTEGER16    Target torque
  [0x0011.0] 0x60B1:0x00 0x20 INTEGER32    Velocity offset
  [0x0015.0] 0x60B2:0x00 0x10 INTEGER16    Torque offset
  [0x0017.0] 0x0000:0x00 0x08
*/

typedef struct __attribute__((__packed__)) {
  uint16_t ctrl_word;
  int8_t op_mode;
  int32_t target_position;
  uint32_t profile_velocity;
  int32_t target_velocity;
  int16_t target_torque;
  int32_t velocity_offset;
  int16_t torque_offset;
  uint8_t padding0;
} cmmt_output_map_t;


void cmmt_set_output(uint8_t index, void* value, char *type, void* generic_map);
void cmmt_send_input(char *response, void* generic_map);
void cmmt_send_output(char *response, void* generic_map);
void cmmt_jaw_mode(uint16_t angle, cmmt_output_map_t *output, cmmt_input_map_t *input);
void jaw_toggle(uint8_t state);
void jaw_set_angle(uint16_t angle);

#endif
