#ifndef PAN335_H
#define PAN335_H

#include <stdint.h>

#define TIMEOUT_CONF 0x3005
#define STP0_CONF 0x3010
#define STP1_CONF 0x3020             

#define PAN355_OPMODE_IDLE 0
#define PAN355_OPMODE_HOLD 1
#define PAN355_OPMODE_POSITION 2
#define PAN355_OPMODE_VELOCITY 3
#define PAN355_OPMODE_DO_TORQUE_HOME 100
#define PAN355_OPMODE_HOME_TP_CURRENT_ENC_POSITION 101
#define PAN355_OPMODE_SET_HOME 102
#define PAN355_OPMODE_RAW_ENCODER_POSITION 200
/*
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0014.0] 0x6000:0x01 0x08 UNSIGNED8    STATUS
  [0x0015.0] 0x6000:0x02 0x08 UNSIGNED8    ERROR
  [0x0016.0] 0x6010:0x01 0x08 UNSIGNED8    STP_ACTUAL_OPMODE
  [0x0017.0] 0x6010:0x02 0x08 UNSIGNED8    STP_ERROR
  [0x0018.0] 0x6010:0x03 0x08 UNSIGNED8    STP_STATUS
  [0x0019.0] 0x0000:0x00 0x08
  [0x001A.0] 0x6010:0x05 0x10 INTEGER16    STP_ACTUAL_ACCELERATION
  [0x001C.0] 0x6010:0x06 0x10 INTEGER16    STP_ACTUAL_VELOCITY
  [0x001E.0] 0x6010:0x07 0x20 INTEGER32    STP_ACTUAL_POSITION
  [0x0022.0] 0x6010:0x08 0x20 INTEGER32    STP_ACTUAL_POSITION_ERROR
  [0x0026.0] 0x6020:0x01 0x08 UNSIGNED8    STP_ACTUAL_OPMODE
  [0x0027.0] 0x6020:0x02 0x08 UNSIGNED8    STP_ERROR
  [0x0028.0] 0x6020:0x03 0x08 UNSIGNED8    STP_STATUS
  [0x0029.0] 0x0000:0x00 0x08
  [0x002A.0] 0x6020:0x05 0x10 INTEGER16    STP_ACTUAL_ACCELERATION
  [0x002C.0] 0x6020:0x06 0x10 INTEGER16    STP_ACTUAL_VELOCITY
  [0x002E.0] 0x6020:0x07 0x20 INTEGER32    STP_ACTUAL_POSITION
  [0x0032.0] 0x6020:0x08 0x20 INTEGER32    STP_ACTUAL_POSITION_ERROR
  [0x0036.0] 0x6030:0x01 0x10 UNSIGNED16   ADC_STATUS
  [0x0038.0] 0x6030:0x02 0x20 UNSIGNED32   ADC_VALUE
*/

typedef struct __attribute__((__packed__)) {
	uint8_t status;
	uint8_t error;
	uint8_t stp0_actual_opmode;
	uint8_t stp0_error;
	uint8_t stp0_status;
    uint8_t padding0;
	int16_t stp0_actual_acceleration;
	int16_t stp0_actual_velocity;
	int32_t stp0_actual_position;
	int32_t stp0_actual_position_error;
    uint8_t stp1_actual_opmode;
	uint8_t stp1_error;
	uint8_t stp1_status;
    uint8_t padding1;
	int16_t stp1_actual_acceleration;
	int16_t stp1_actual_velocity;
	int32_t stp1_actual_position;
	int32_t stp1_actual_position_error;
} pan355_input_map_t;

/*
    SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x7010:0x01 0x08 UNSIGNED8    STP_REQUESTED_OPMODE
  [0x0001.0] 0x0000:0x00 0x08
  [0x0002.0] 0x7010:0x03 0x10 UNSIGNED16   STP_TARGET_ACCELERATION
  [0x0004.0] 0x7010:0x04 0x10 INTEGER16    STP_TARGET_VELOCITY
  [0x0006.0] 0x7010:0x05 0x20 INTEGER32    STP_TARGET_POSITION
  [0x000A.0] 0x7020:0x01 0x08 UNSIGNED8    STP_REQUESTED_OPMODE
  [0x000B.0] 0x0000:0x00 0x08
  [0x000C.0] 0x7020:0x03 0x10 UNSIGNED16   STP_TARGET_ACCELERATION
  [0x000E.0] 0x7020:0x04 0x10 INTEGER16    STP_TARGET_VELOCITY
  [0x0010.0] 0x7020:0x05 0x20 INTEGER32    STP_TARGET_POSITION
*/

typedef struct __attribute__((__packed__)) {
  uint8_t stp0_requested_opmode;
  uint8_t padding0;
  uint16_t stp0_target_acceleration;
  int16_t stp0_target_velocity;
  int32_t stp0_target_position;
  uint8_t stp1_requested_opmode;
  uint8_t padding1;
  uint16_t stp1_target_acceleration;
  int16_t stp1_target_velocity;
  int32_t stp1_target_position;
} pan355_output_map_t;

void pan355_send_input(char *response, void* generic_map);
void pan355_send_output(char *response, void* generic_map);
void pan355_set_output(uint8_t index, void* value, char *type, void* generic_map);

#endif
