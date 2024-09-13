#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ethercat.h"
#include "ethercatcoe.h"
#include "ethercattype.h"

static int dx3_write8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int dx3_write16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

static int dx3_write32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
   int wkc;

   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                      EC_TIMEOUTRXM);
   return wkc;
}

int dx3_setup(uint16 slave)
{
   int wkc = 0;

   printf ("DX3 drive setup\n");

   wkc += dx3_write8 (slave, 0x1C12, 0, 0);
   wkc += dx3_write8 (slave, 0x1C13, 0, 0);

   wkc += dx3_write8  (slave, 0x1A00, 0, 0);
   wkc += dx3_write32 (slave, 0x1A00, 1, 0x60410010);
   wkc += dx3_write32 (slave, 0x1A00, 2, 0x60640020);
   wkc += dx3_write32 (slave, 0x1A00, 3, 0x603F0010);
   wkc += dx3_write8  (slave, 0x1A00, 0, 3);

   wkc += dx3_write8  (slave, 0x1600, 0, 0);
   wkc += dx3_write32 (slave, 0x1600, 1, 0x60400010);
   wkc += dx3_write32 (slave, 0x1600, 2, 0x60600008);
   wkc += dx3_write32 (slave, 0x1600, 3, 0x607A0020);
   wkc += dx3_write32 (slave, 0x1600, 4, 0x60810020);
   wkc += dx3_write32 (slave, 0x1600, 5, 0x60830020);
   wkc += dx3_write32 (slave, 0x1600, 6, 0x60840020);
   wkc += dx3_write32 (slave, 0x1600, 7, 0x60FF0020);
   wkc += dx3_write32 (slave, 0x1600, 8, 0x60980008);
   wkc += dx3_write8  (slave, 0x1600, 0, 8);

   wkc += dx3_write16 (slave, 0x1C12, 1, 0x1600);
   wkc += dx3_write16 (slave, 0x1C12, 0, 1);

   wkc += dx3_write16 (slave, 0x1C13, 1, 0x1A00);
   wkc += dx3_write16 (slave, 0x1C13, 0, 1);
   
   if (wkc != 21)
   {
      printf ("DX3 setup failed\n");
      return -1;
   }

   return 0;
}

void dx3_populate_OElist(uint16_t index, uint16_t *data_type, char *name){
    printf("INDEX: %4.4x\n", index);
    switch (index)
    {
    case 0x6040:
        strcpy(name, "Controlword");
        *data_type = ECT_UNSIGNED16;
        break;
    case 0x6060:
        strcpy(name, "Modes of operation");
        *data_type = ECT_UNSIGNED8;
        break;
    case 0x607a:
        strcpy(name, "Target position");
        *data_type = ECT_INTEGER32;
        break;
    case 0x6041:
        strcpy(name, "Statusword");
        *data_type = ECT_UNSIGNED16;
        break;
    case 0x6064:
        strcpy(name, "Actual position");
        *data_type = ECT_INTEGER32;
        break;
    case 0x603F:
        strcpy(name, "Error code");
        *data_type = ECT_UNSIGNED16;
        break;
    case 0x6081:
        strcpy(name, "Profile velocity");
        *data_type = ECT_UNSIGNED32;
        break;
    case 0x6083:
        strcpy(name, "Profile acceleration");
        *data_type = ECT_UNSIGNED32;
        break;
    case 0x6084:
        strcpy(name, "Profile deceleration");
        *data_type = ECT_UNSIGNED32;
        break;
        case 0x60FF:
        strcpy(name, "target velocity");
        *data_type = ECT_INTEGER32;
        break;
    case 0x6098:
        strcpy(name, "homing method");
        *data_type = ECT_INTEGER8;
        break;

    default:
        strcpy(name, "");
        *data_type = ECT_BOOLEAN;
        break;
    }
}