#include <stdint.h>

/*  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0013.0] 0x6041:0x00 0x10 UNSIGNED16   Statusword
  [0x0015.0] 0x6064:0x00 0x20 INTEGER32    Actual position
  [0x0019.0] 0x603F:0x00 0x10 UNSIGNED16   Error code */

typedef struct __attribute__((__packed__))
{
  uint16_t status;
  int32_t actual_position;
  uint16_t error;
} dx3_input_map_t;

/*
  SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16   Controlword
  [0x0002.0] 0x6060:0x00 0x08 UNSIGNED8    Modes of operation
  [0x0003.0] 0x607A:0x00 0x20 INTEGER32    Target position
  [0x0007.0] 0x6081:0x00 0x20 UNSIGNED32   Profile velocity
  [0x000B.0] 0x6083:0x00 0x20 UNSIGNED32   Profile acceleration
  [0x000F.0] 0x6084:0x00 0x20 UNSIGNED32   Profile deceleration

*/

typedef struct __attribute__((__packed__))
{
  uint16_t ctrl_word;
  uint8_t operation_mode;
  uint8_t padding0;
  int32_t target_position;                      
  uint32_t profile_velocity;
  uint32_t profile_acceleration;
  uint32_t profile_deceleration;
  int32_t target_velocity;
  int8_t homing_method;
  uint8_t padding1;
} dx3_output_map_t;

int dx3_setup(uint16 slave);
void dx3_populate_OElist(uint16_t index, uint16_t *data_type, char *name);
void dx3_send_input(char *response, void *generic_map);
void dx3_send_output(char *response, void *generic_map);
void dx3_set_output(uint8_t index, void *value, char *type, void *generic_map);