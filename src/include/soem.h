#include <stdbool.h>
#include "osal_defs.h"

#define MAXSLAVES 20

typedef struct ethercat_slave{
    uint32_t slave_id;
    uint16_t slave_addr;
    char uid[32];
    char name[20];
    void (* send_output)(char *response, void* output_map);
    void (* send_input)(char *response, void* input_map);
    void (* set_output)(uint8_t idx, void* value, char *type, void* output_map);
    void* output_map;
    void* input_map;
}ethercat_slave_t;

typedef struct ethcercat_slave_collection{
    uint8_t slave_cnt;
    ethercat_slave_t _slaves[MAXSLAVES];
}ethcercat_slave_collection_t;

void get_adapters(char *response);
void ethercat_loop(void);
int initialize_ethercat(char *ifname);
OSAL_THREAD_FUNC ecatcheck( void *ptr );
ethercat_slave_t *get_slave(uint8_t idx);
ethcercat_slave_collection_t *get_all_slaves();
void init_ethercat_loop();
int retreive_info();
