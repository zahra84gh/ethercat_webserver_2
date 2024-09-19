#include <stdint.h>



typedef struct __attribute__((__packed__))
{
  uint16_t status;
  int32_t actual_position;
  uint16_t error;
  int16_t torque_actual_value;
} dx3_input_map_t;



typedef struct __attribute__((__packed__))
{
  uint16_t ctrl_word;
  uint8_t operation_mode;
  uint8_t padding0;
  int32_t target_position;                      
  uint32_t profile_velocity;
  uint32_t profile_acceleration;
  uint32_t profile_deceleration;
  int32_t home_offset;   
  int32_t target_velocity;
} dx3_output_map_t;

int dx3_setup(uint16 slave);
void dx3_populate_OElist(uint16_t index, uint16_t *data_type, char *name);
void dx3_send_input(char *response, void *generic_map);
void dx3_send_output(char *response, void *generic_map);
void dx3_set_output(uint8_t index, void *value, char *type, void *generic_map);







#define TORQUE_LIMIT_INDEX 0x6072      // Max Torque  index
#define TORQUE_ACTUAL_INDEX 0x6077     // Torque Actual Value index
#define POSITION_ACTUAL_INDEX 0x6063  // actual position of motor index
#define HOMING_METHOD_INDEX 0x6098     // Homing method index
#define CONTROL_WORD_INDEX 0x6040      // Control Word index
#define STATUS_WORD_INDEX 0x6041       // Status Word index


#define CONTROL_WORD_HALT 0x0006
#define CONTROL_WORD_ENABLE_OPERATION 0x000F   //15
#define STATUS_HOMING_COMPLETE 0x3400  // Example value for homing complete



