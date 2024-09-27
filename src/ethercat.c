/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include "ethercat.h"
#include "include/soem.h"
#include "include/cmmt.h"
#include "include/eio116e.h"
#include "include/pan355.h"
#include "include/dx3.h"
#include "include/step_motor_pan355.h"

uint8_t global_sdo_read = 0;
uint8_t global_pdo_read = 0;
uint8_t global_get_info = 0;
uint8_t global_kill_loop = 0;
uint8_t global_status_loop = 0;
uint8_t global_jaw_mode = 0;
uint32_t input[21];

ethercat_slave_t *slave_configurations = NULL;
ethcercat_slave_collection_t slave_collection;
io116_collection_t *io116e_devices;

char IOmap[4096];
ec_ODlistt ODlist;
ec_OElistt OElist;
boolean printSDO = FALSE;
boolean printMAP = FALSE;
char usdo[128];

#define EC_TIMEOUTMON 500
// #define EIO116
// #define PAN355
// #define MX2
// #define CMMT
#define DX3

#ifdef DX3
dx3_input_map_t *input_map;
dx3_output_map_t *output_map;
#endif

#ifdef PAN335
pan355_input_map_t *input_map;
pan355_output_map_t *output_map;
#endif

#ifdef EIO116
io116e_input_map_t *input_map;
io116e_output_map_t *output_map;
#endif

#ifdef MX2
mx2_input_map_t *input_map;
mx2_output_map_t *output_map;
#endif

#ifdef CMMT
cmmt_input_map_t *input_map;
cmmt_output_map_t *output_map;
#endif

char IOmap[4096];

int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

#define OTYPE_VAR 0x0007
#define OTYPE_ARRAY 0x0008
#define OTYPE_RECORD 0x0009

#define ATYPE_Rpre 0x01
#define ATYPE_Rsafe 0x02
#define ATYPE_Rop 0x04
#define ATYPE_Wpre 0x08
#define ATYPE_Wsafe 0x10
#define ATYPE_Wop 0x20

FILE *fp;
char filename[20];

char *dtype2string(uint16 dtype, uint16 bitlen)
{
    static char str[32] = {0};

    switch (dtype)
    {
    case ECT_BOOLEAN:
        sprintf(str, "BOOLEAN");
        break;
    case ECT_INTEGER8:
        sprintf(str, "INTEGER8");
        break;
    case ECT_INTEGER16:
        sprintf(str, "INTEGER16");
        break;
    case ECT_INTEGER32:
        sprintf(str, "INTEGER32");
        break;
    case ECT_INTEGER24:
        sprintf(str, "INTEGER24");
        break;
    case ECT_INTEGER64:
        sprintf(str, "INTEGER64");
        break;
    case ECT_UNSIGNED8:
        sprintf(str, "UNSIGNED8");
        break;
    case ECT_UNSIGNED16:
        sprintf(str, "UNSIGNED16");
        break;
    case ECT_UNSIGNED32:
        sprintf(str, "UNSIGNED32");
        break;
    case ECT_UNSIGNED24:
        sprintf(str, "UNSIGNED24");
        break;
    case ECT_UNSIGNED64:
        sprintf(str, "UNSIGNED64");
        break;
    case ECT_REAL32:
        sprintf(str, "REAL32");
        break;
    case ECT_REAL64:
        sprintf(str, "REAL64");
        break;
    case ECT_BIT1:
        sprintf(str, "BIT1");
        break;
    case ECT_BIT2:
        sprintf(str, "BIT2");
        break;
    case ECT_BIT3:
        sprintf(str, "BIT3");
        break;
    case ECT_BIT4:
        sprintf(str, "BIT4");
        break;
    case ECT_BIT5:
        sprintf(str, "BIT5");
        break;
    case ECT_BIT6:
        sprintf(str, "BIT6");
        break;
    case ECT_BIT7:
        sprintf(str, "BIT7");
        break;
    case ECT_BIT8:
        sprintf(str, "BIT8");
        break;
    case ECT_VISIBLE_STRING:
        sprintf(str, "VISIBLE_STR(%d)", bitlen);
        break;
    case ECT_OCTET_STRING:
        sprintf(str, "OCTET_STR(%d)", bitlen);
        break;
    default:
        sprintf(str, "dt:0x%4.4X (%d)", dtype, bitlen);
    }
    return str;
}

char *otype2string(uint16 otype)
{
    static char str[32] = {0};

    switch (otype)
    {
    case OTYPE_VAR:
        sprintf(str, "VAR");
        break;
    case OTYPE_ARRAY:
        sprintf(str, "ARRAY");
        break;
    case OTYPE_RECORD:
        sprintf(str, "RECORD");
        break;
    default:
        sprintf(str, "ot:0x%4.4X", otype);
    }
    return str;
}

char *access2string(uint16 access)
{
    static char str[32] = {0};

    sprintf(str, "%s%s%s%s%s%s",
            ((access & ATYPE_Rpre) != 0 ? "R" : "_"),
            ((access & ATYPE_Wpre) != 0 ? "W" : "_"),
            ((access & ATYPE_Rsafe) != 0 ? "R" : "_"),
            ((access & ATYPE_Wsafe) != 0 ? "W" : "_"),
            ((access & ATYPE_Rop) != 0 ? "R" : "_"),
            ((access & ATYPE_Wop) != 0 ? "W" : "_"));
    return str;
}

char *SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
    int l = sizeof(usdo) - 1, i;
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;
    char es[32];

    memset(&usdo, 0, 128);
    ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
    if (EcatError)
    {
        return ec_elist2string();
    }
    else
    {
        static char str[64] = {0};
        switch (dtype)
        {
        case ECT_BOOLEAN:
            u8 = (uint8 *)&usdo[0];
            if (*u8)
                sprintf(str, "TRUE");
            else
                sprintf(str, "FALSE");
            break;
        case ECT_INTEGER8:
            i8 = (int8 *)&usdo[0];
            sprintf(str, "0x%2.2x / %d", *i8, *i8);
            break;
        case ECT_INTEGER16:
            i16 = (int16 *)&usdo[0];
            sprintf(str, "0x%4.4x / %d", *i16, *i16);
            break;
        case ECT_INTEGER32:
        case ECT_INTEGER24:
            i32 = (int32 *)&usdo[0];
            sprintf(str, "0x%8.8x / %d", *i32, *i32);
            break;
        case ECT_INTEGER64:
            i64 = (int64 *)&usdo[0];
            sprintf(str, "0x%16.16" PRIx64 " / %" PRId64, *i64, *i64);
            break;
        case ECT_UNSIGNED8:
            u8 = (uint8 *)&usdo[0];
            sprintf(str, "0x%2.2x / %u", *u8, *u8);
            break;
        case ECT_UNSIGNED16:
            u16 = (uint16 *)&usdo[0];
            sprintf(str, "0x%4.4x / %u", *u16, *u16);
            break;
        case ECT_UNSIGNED32:
        case ECT_UNSIGNED24:
            u32 = (uint32 *)&usdo[0];
            sprintf(str, "0x%8.8x / %u", *u32, *u32);
            break;
        case ECT_UNSIGNED64:
            u64 = (uint64 *)&usdo[0];
            sprintf(str, "0x%16.16" PRIx64 " / %" PRIu64, *u64, *u64);
            break;
        case ECT_REAL32:
            sr = (float *)&usdo[0];
            sprintf(str, "%f", *sr);
            break;
        case ECT_REAL64:
            dr = (double *)&usdo[0];
            sprintf(str, "%f", *dr);
            break;
        case ECT_BIT1:
        case ECT_BIT2:
        case ECT_BIT3:
        case ECT_BIT4:
        case ECT_BIT5:
        case ECT_BIT6:
        case ECT_BIT7:
        case ECT_BIT8:
            u8 = (uint8 *)&usdo[0];
            sprintf(str, "0x%x / %u", *u8, *u8);
            break;
        case ECT_VISIBLE_STRING:
            strcpy(str, "\"");
            strcat(str, usdo);
            strcat(str, "\"");
            break;
        case ECT_VISIBLE_STRING_WITHOUT_QUOTES:
            strcpy(str, usdo);
            break;
        case ECT_OCTET_STRING:
            str[0] = 0x00;
            for (i = 0; i < l; i++)
            {
                sprintf(es, "0x%2.2x ", usdo[i]);
                strcat(str, es);
            }
            break;
        default:
            sprintf(str, "Unknown type");
        }
        return str;
    }
}

/** Read PDO assign structure */
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;
    uint8 bitlen, obj_subidx;
    uint16 obj_idx;
    int abs_offset, abs_bit;

    rdl = sizeof(rdat);
    rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from slave ? */
    if ((wkc > 0) && (rdat > 0))
    {
        /* number of available sub indexes */
        nidx = rdat;
        bsize = 0;
        /* read all PDO's */
        for (idxloop = 1; idxloop <= nidx; idxloop++)
        {
            rdl = sizeof(rdat);
            rdat = 0;
            /* read PDO assign */
            wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
            /* result is index of PDO */
            idx = etohs(rdat);
            if (idx > 0)
            {
                rdl = sizeof(subcnt);
                subcnt = 0;
                /* read number of subindexes of PDO */
                wkc = ec_SDOread(slave, idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                subidx = subcnt;
                /* for each subindex */
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    rdl = sizeof(rdat2);
                    rdat2 = 0;
                    /* read SDO that is mapped in PDO */
                    wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                    rdat2 = etohl(rdat2);
                    /* extract bitlength of SDO */
                    bitlen = LO_BYTE(rdat2);
                    bsize += bitlen;
                    obj_idx = (uint16)(rdat2 >> 16);
                    obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;
                    ODlist.Slave = slave;
                    ODlist.Index[0] = obj_idx;
                    OElist.Entries = 0;
                    wkc = 0;
                    /* read object entry from dictionary if not a filler (0x0000:0x00) */
                    if (obj_idx || obj_subidx)
                        wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
                    if (OElist.Entries != nidx)
                    {
                        dx3_populate_OElist(obj_idx, &OElist.DataType[obj_subidx], OElist.Name[obj_subidx]);
                        OElist.Entries++;
                        wkc++;
                    }
                    fprintf(fp, "  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                    if ((wkc > 0) && OElist.Entries)
                    {
                        fprintf(fp, " %-12s %s\n", dtype2string(OElist.DataType[obj_subidx], bitlen), OElist.Name[obj_subidx]);
                    }
                    else
                        fprintf(fp, "\n");
                    bitoffset += bitlen;
                };
            };
        };
    };
    /* return total found bitlength (PDO) */
    return bsize;
}

int si_map_sdo(int slave, char *slave_id)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize, outputs_bo, inputs_bo;
    uint8 SMt_bug_add;

    printf("PDO mapping according to CoE :\n");
    SMt_bug_add = 0;
    outputs_bo = 0;
    inputs_bo = 0;
    rdl = sizeof(nSM);
    nSM = 0;
    /* read SyncManager Communication Type object count */
    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (nSM > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM--;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > EC_MAXSM)
            nSM = EC_MAXSM;
        /* iterate for every SM type defined */
        sprintf(filename, "./%s_pdo", slave_id);

        if (access(filename, F_OK) == 0)
            return retVal;

        fp = fopen(filename, "w");
        if (fp == NULL)
        {
            printf("Error creating pdo file!\n");
            return -1;
        }
        printf("HERHEHRE\n");
        for (iSM = 2; iSM <= nSM; iSM++)
        {
            rdl = sizeof(tSM);
            tSM = 0;
            /* read SyncManager Communication Type */
            wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            if (wkc > 0)
            {
                if ((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
                {
                    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                    printf("Activated SM type workaround, possible incorrect mapping.\n");
                }
                if (tSM)
                    tSM += SMt_bug_add; // only add if SMt > 0

                if (tSM == 3) // outputs
                {
                    /* read the assign RXPDO */
                    fprintf(fp, "  SM%1d outputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].outputs - (uint8 *)&IOmap[0]), outputs_bo);
                    outputs_bo += Tsize;
                }
                if (tSM == 4) // inputs
                {
                    /* read the assign TXPDO */
                    fprintf(fp, "  SM%1d inputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].inputs - (uint8 *)&IOmap[0]), inputs_bo);
                    inputs_bo += Tsize;
                }
            }
        }
        fclose(fp);
    }

    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset)
{
    uint16 a, w, c, e, er;
    uint8 eectl;
    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint8 bitlen;
    int totalsize;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;
    char str_name[EC_MAXNAME + 1];

    eectl = ec_slave[slave].eep_pdi;

    totalsize = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0; c < EC_MAXSM; c++)
        PDO->SMbitsize[c] = 0;
    if (t > 1)
        t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
        a = PDO->Startpos;
        w = ec_siigetbyte(slave, a++);
        w += (ec_siigetbyte(slave, a++) << 8);
        PDO->Length = w;
        c = 1;
        /* traverse through all PDOs */
        do
        {
            PDO->nPDO++;
            PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
            PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
            PDO->BitSize[PDO->nPDO] = 0;
            c++;
            /* number of entries in PDO */
            e = ec_siigetbyte(slave, a++);
            PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
            a++;
            obj_name = ec_siigetbyte(slave, a++);
            a += 2;
            c += 2;
            if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
            {
                str_name[0] = 0;
                if (obj_name)
                    ec_siistring(str_name, slave, obj_name);
                if (t)
                    printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                else
                    printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                printf("     addr b   index: sub bitl data_type    name\n");
                /* read all entries defined in PDO */
                for (er = 1; er <= e; er++)
                {
                    c += 4;
                    obj_idx = ec_siigetbyte(slave, a++);
                    obj_idx += (ec_siigetbyte(slave, a++) << 8);
                    obj_subidx = ec_siigetbyte(slave, a++);
                    obj_name = ec_siigetbyte(slave, a++);
                    obj_datatype = ec_siigetbyte(slave, a++);
                    bitlen = ec_siigetbyte(slave, a++);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;

                    PDO->BitSize[PDO->nPDO] += bitlen;
                    a += 2;

                    /* skip entry if filler (0x0000:0x00) */
                    if (obj_idx || obj_subidx)
                    {
                        str_name[0] = 0;
                        if (obj_name)
                            ec_siistring(str_name, slave, obj_name);

                        printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                        printf(" %-12s %s\n", dtype2string(obj_datatype, bitlen), str_name);
                    }
                    bitoffset += bitlen;
                    totalsize += bitlen;
                }
                PDO->SMbitsize[PDO->SyncM[PDO->nPDO]] += PDO->BitSize[PDO->nPDO];
                c++;
            }
            else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
            {
                c += 4 * e;
                a += 8 * e;
                c++;
            }
            if (PDO->nPDO >= (EC_MAXEEPDO - 1))
                c = PDO->Length; /* limit number of PDO entries in buffer */
        } while (c < PDO->Length);
    }
    if (eectl)
        ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return totalsize;
}

int si_map_sii(int slave)
{
    int retVal = 0;
    int Tsize, outputs_bo, inputs_bo;

    printf("PDO mapping according to SII :\n");

    outputs_bo = 0;
    inputs_bo = 0;
    /* read the assign RXPDOs */
    Tsize = si_siiPDO(slave, 1, (int)(ec_slave[slave].outputs - (uint8 *)&IOmap), outputs_bo);
    outputs_bo += Tsize;
    /* read the assign TXPDOs */
    Tsize = si_siiPDO(slave, 0, (int)(ec_slave[slave].inputs - (uint8 *)&IOmap), inputs_bo);
    inputs_bo += Tsize;
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

void si_sdo(int cnt, char *slave_id)
{
    int i, j;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if (ec_readODlist(cnt, &ODlist) > 0)
    {
        printf(" CoE Object Description found, %d entries.\n", ODlist.Entries);
        sprintf(filename, "./%s_sdo", slave_id);

        if (access(filename, F_OK) == 0)
            return;

        fp = fopen(filename, "w");
        if (fp == NULL)
        {
            printf("Error creating sdo file!\n");
            return;
        }
        for (i = 0; i < ODlist.Entries; i++)
        {
            uint8_t max_sub;
            char name[128] = {0};

            ec_readODdescription(i, &ODlist);
            while (EcatError)
                fprintf(fp, " - %s\n", ec_elist2string());
            snprintf(name, sizeof(name) - 1, "\"%s\"", ODlist.Name[i]);
            if (ODlist.ObjectCode[i] == OTYPE_VAR)
            {
                fprintf(fp, "0x%04x      %-40s      [%s]\n", ODlist.Index[i], name,
                        otype2string(ODlist.ObjectCode[i]));
            }
            else
            {
                fprintf(fp, "0x%04x      %-40s      [%s  maxsub(0x%02x / %d)]\n",
                        ODlist.Index[i], name, otype2string(ODlist.ObjectCode[i]),
                        ODlist.MaxSub[i], ODlist.MaxSub[i]);
            }
            memset(&OElist, 0, sizeof(OElist));
            ec_readOE(i, &ODlist, &OElist);
            while (EcatError)
                fprintf(fp, "- %s\n", ec_elist2string());

            if (ODlist.ObjectCode[i] != OTYPE_VAR)
            {
                int l = sizeof(max_sub);
                ec_SDOread(cnt, ODlist.Index[i], 0, FALSE, &l, &max_sub, EC_TIMEOUTRXM);
            }
            else
            {
                max_sub = ODlist.MaxSub[i];
            }

            for (j = 0; j < max_sub + 1; j++)
            {
                if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
                {
                    snprintf(name, sizeof(name) - 1, "\"%s\"", OElist.Name[j]);
                    fprintf(fp, "    0x%02x      %-40s      [%-16s %6s]      ", j, name,
                            dtype2string(OElist.DataType[j], OElist.BitLength[j]),
                            access2string(OElist.ObjAccess[j]));
                    if ((OElist.ObjAccess[j] & 0x0007))
                    {
                        fprintf(fp, "%s", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
                    }
                    fprintf(fp, "\n");
                }
            }
        }
    }
    else
    {
        while (EcatError)
            fprintf(fp, "%s", ec_elist2string());
    }
    fclose(fp);
}

ethercat_slave_t *get_slave(uint8_t idx)
{
    return &slave_collection._slaves[idx];
}

ethcercat_slave_collection_t *get_all_slaves()
{
    return &slave_collection;
}

void init_config(uint32_t product_id)
{
    switch (product_id)
    {
    case 0x000080001:
        uint8_t u8_val = 0;
        uint16_t u16_val = 0x03e8;
        uint32_t u32_val = 0x01;
        ec_SDOwrite(1, TIMEOUT_CONF, 0x01, FALSE, sizeof(u16_val), &u16_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, TIMEOUT_CONF, 0x02, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, TIMEOUT_CONF, 0x03, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
        u8_val = 0;
        ec_SDOwrite(1, STP0_CONF, 0x01, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x01, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        u8_val = 8;
        ec_SDOwrite(1, STP0_CONF, 0x02, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x02, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        u8_val = 10;
        ec_SDOwrite(1, STP0_CONF, 0x03, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x03, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        u8_val = 15;
        ec_SDOwrite(1, STP0_CONF, 0x04, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x04, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        u32_val = 0;
        ec_SDOwrite(1, STP0_CONF, 0x05, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x05, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
        u32_val = 0;
        ec_SDOwrite(1, STP0_CONF, 0x06, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x06, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
        u8_val = 8;
        ec_SDOwrite(1, STP0_CONF, 0x07, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x07, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        u8_val = 0;
        ec_SDOwrite(1, STP0_CONF, 0x08, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        ec_SDOwrite(1, STP1_CONF, 0x08, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
        break;

    default:
        break;
    }
}

int retreive_info()
{
    int cnt, i, j, nSM;
    uint16 ssigen;
    int expectedWKC;
    printMAP = true;
    printSDO = true;
    char name[128];
    printf("%d slaves found and configured.\n", ec_slavecount);
    ec_readstate();
    slave_collection.slave_cnt = ec_slavecount;
    for (cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
               cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
               ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
        slave_collection._slaves[cnt - 1].slave_id = cnt;
        strcpy(slave_collection._slaves[cnt - 1].name, SDO2string(cnt, 0x1008, 0x00, ECT_VISIBLE_STRING_WITHOUT_QUOTES));
        printf("Name: %s\n", slave_collection._slaves[cnt - 1].name);
        if (0 == strcmp(slave_collection._slaves[cnt - 1].name, "EIO116"))
        {
            slave_collection._slaves[cnt - 1].send_output = io116e_send_output;
            slave_collection._slaves[cnt - 1].send_input = io116e_send_input;
            slave_collection._slaves[cnt - 1].set_output = io116e_set_output;
            slave_collection._slaves[cnt - 1].output_map = (io116e_output_map_t *)ec_slave[cnt].outputs;
            slave_collection._slaves[cnt - 1].input_map = (io116e_input_map_t *)ec_slave[cnt].inputs;
        }
        else if (0 == strcmp(slave_collection._slaves[cnt - 1].name, "CMMT-AS-MP-S1"))
        {
            slave_collection._slaves[cnt - 1].send_output = cmmt_send_output;
            slave_collection._slaves[cnt - 1].send_input = cmmt_send_input;
            slave_collection._slaves[cnt - 1].set_output = cmmt_set_output;
            slave_collection._slaves[cnt - 1].input_map = (cmmt_input_map_t *)ec_slave[cnt].inputs;
            slave_collection._slaves[cnt - 1].output_map = (cmmt_output_map_t *)ec_slave[cnt].outputs;
        }
        else if (0 == strcmp(slave_collection._slaves[cnt - 1].name, "PAN355"))
        {
            slave_collection._slaves[cnt - 1].send_output = pan355_send_output;
            slave_collection._slaves[cnt - 1].send_input = pan355_send_input;
            slave_collection._slaves[cnt - 1].set_output = pan355_set_output;
            slave_collection._slaves[cnt - 1].input_map = (pan355_input_map_t *)ec_slave[cnt].inputs;
            slave_collection._slaves[cnt - 1].output_map = (pan355_output_map_t *)ec_slave[cnt].outputs;
        }
        // else if(0 == strcmp(slave_collection._slaves[cnt - 1].name, "DX3 SERVO DRIVES")){
        else if (ec_slave[cnt].eep_man == 0x000002de)
        {
            printf("DX3 drive found!\n");
            ec_slave[cnt].PO2SOconfig = dx3_setup;

            ec_reconfig_slave(cnt, EC_TIMEOUTRET3);
            slave_collection._slaves[cnt - 1].send_output = dx3_send_output;
            slave_collection._slaves[cnt - 1].send_input = dx3_send_input;
            slave_collection._slaves[cnt - 1].set_output = dx3_set_output;
            slave_collection._slaves[cnt - 1].input_map = (dx3_input_map_t *)ec_slave[cnt].inputs;
            slave_collection._slaves[cnt - 1].output_map = (dx3_output_map_t *)ec_slave[cnt].outputs;
        }
        if (ec_slave[cnt].hasdc)
            printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
        printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0,
               (ec_slave[cnt].activeports & 0x02) > 0,
               (ec_slave[cnt].activeports & 0x04) > 0,
               (ec_slave[cnt].activeports & 0x08) > 0);
        printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
        slave_collection._slaves[cnt - 1].slave_addr = ec_slave[cnt].configadr;
        printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
        for (nSM = 0; nSM < EC_MAXSM; nSM++)
        {
            if (ec_slave[cnt].SM[nSM].StartAddr > 0)
                printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n", nSM, etohs(ec_slave[cnt].SM[nSM].StartAddr), etohs(ec_slave[cnt].SM[nSM].SMlength),
                       etohl(ec_slave[cnt].SM[nSM].SMflags), ec_slave[cnt].SMtype[nSM]);
        }
        for (j = 0; j < ec_slave[cnt].FMMUunused; j++)
        {
            printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
                   etohl(ec_slave[cnt].FMMU[j].LogStart), etohs(ec_slave[cnt].FMMU[j].LogLength), ec_slave[cnt].FMMU[j].LogStartbit,
                   ec_slave[cnt].FMMU[j].LogEndbit, etohs(ec_slave[cnt].FMMU[j].PhysStart), ec_slave[cnt].FMMU[j].PhysStartBit,
                   ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
        }
        printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
               ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
        printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
        ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
        /* SII general section */
        if (ssigen)
        {
            ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
            ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
            ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
            ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
            if ((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
            {
                ec_slave[cnt].blockLRW = 1;
                ec_slave[0].blockLRW++;
            }
            ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
            ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
            ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
        }
        printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
               ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
        printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
               ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);

        char slave_id[50];
        sprintf(slave_id, "%4.4x_%8.8x_%8.8x_%8.8x", ec_slave[cnt].configadr, ec_slave[cnt].eep_man, ec_slave[cnt].eep_id, ec_slave[cnt].eep_rev);
        printf("Slave unique id: %s", slave_id);
        strcpy(slave_collection._slaves[cnt - 1].uid, slave_id);

        // if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
        //     si_sdo(cnt, slave_id);
        if (printMAP)
        {
            if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                si_map_sdo(cnt, slave_id);
            else
                si_map_sii(cnt);
        }
    }
    return 0;
}

char ifbuf[1024];

void get_adapters(char *response)
{
    uint8_t counter = 0;
    char adapter_buffer[150];
    ec_adaptert *adapter = NULL;
    strcpy(response, "{");
    adapter = ec_find_adapters();
    while (adapter != NULL)
    {
        sprintf(adapter_buffer, "\"adapter_%d\": \"%s\",", counter, adapter->name);
        strcat(response, adapter_buffer);
        adapter = adapter->next;
        counter++;
    }
    response[strlen(response) - 1] = '\0';
    strcat(response, "}");
    ec_free_adapters(adapter);
}

void init_ethercat_loop()
{
    if (global_status_loop)
    {
        global_kill_loop = 1;
        while (global_status_loop)
        {
        }
    }
    while (!global_status_loop)
    {
    }
}

void dx3_send_input(char *response, void *generic_map)
{
    char input_status[100];
    dx3_input_map_t *input_map = (dx3_input_map_t *)generic_map;

    sprintf(input_status, "%d %d %d %d",
            input_map->status,
            input_map->actual_position,
            input_map->error,
            input_map->torque_actual_value);

    memcpy(response, input_status, sizeof(input_status));
}

void dx3_send_output(char *response, void *generic_map)
{
    char output_status[100];
    dx3_output_map_t *output_map = (dx3_output_map_t *)generic_map;

    sprintf(output_status, "%d %d %d %d %d %d %d %d",
            output_map->ctrl_word,
            output_map->operation_mode,
            output_map->target_position,
            output_map->profile_velocity,
            output_map->profile_acceleration,
            output_map->profile_deceleration,
            output_map->home_offset,
            output_map->target_velocity);

    memcpy(response, output_status, sizeof(output_status));
}

void dx3_set_output(uint8_t index, void *value, char *type, void *generic_map)
{
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;

    if (0 == strcmp(type, "INTEGER8"))
    {
        i8 = (int8_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER16"))
    {
        i16 = (int16_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER32"))
    {
        i32 = (int32_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER64"))
    {
        i64 = (int64_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED8"))
    {
        u8 = (uint8_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED16"))
    {
        u16 = (uint16_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED32"))
    {
        u32 = (uint32_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED64"))
    {
        u64 = (uint64_t *)value;
    }
    else if (0 == strcmp(type, "REAL32"))
    {
        sr = (float *)value;
    }
    else if (0 == strcmp(type, "REAL64"))
    {
        dr = (double *)value;
    }

    dx3_output_map_t *output_map = (dx3_output_map_t *)generic_map;

    switch (index)
    {
    case 0:
        output_map->ctrl_word = *u16;
        break;
    case 1:
        output_map->operation_mode = *u8;
        break;
    case 2:
        output_map->target_position = *i32;
        break;
    case 3:
        output_map->profile_velocity = *u32;
        break;
    case 4:
        output_map->profile_acceleration = *u32;
        break;
    case 5:
        output_map->profile_deceleration = *u32;
        break;
    case 6:
        output_map->home_offset = *i32;
        break;
    case 7:
        output_map->target_velocity = *i32;
        break;
    default:
        break;
    }
}

void pan355_send_input(char *response, void *generic_map)
{
    char input_status[100];
    pan355_input_map_t *input_map = (pan355_input_map_t *)generic_map;

    sprintf(input_status, "%d %d %d %d %d %d %d %d "
                          "%d %d %d %d %d %d %d %d ",
            input_map->status,
            input_map->error,
            input_map->stp0_actual_opmode,
            input_map->stp0_error,
            input_map->stp0_status,
            input_map->stp0_actual_acceleration,
            input_map->stp0_actual_velocity,
            input_map->stp0_actual_position,
            input_map->stp0_actual_position_error,
            input_map->stp1_actual_opmode,
            input_map->stp1_error,
            input_map->stp1_status,
            input_map->stp1_actual_acceleration,
            input_map->stp1_actual_velocity,
            input_map->stp1_actual_position,
            input_map->stp1_actual_position_error);

    memcpy(response, input_status, sizeof(input_status));
}

void pan355_send_output(char *response, void *generic_map)
{
    char output_status[100];
    pan355_output_map_t *output_map = (pan355_output_map_t *)generic_map;

    sprintf(output_status, "%d %d %d %d %d %d %d %d ",
            output_map->stp0_requested_opmode,
            output_map->stp0_target_acceleration,
            output_map->stp0_target_velocity,
            output_map->stp0_target_position,
            output_map->stp1_requested_opmode,
            output_map->stp1_target_acceleration,
            output_map->stp1_target_velocity,
            output_map->stp1_target_position);

    memcpy(response, output_status, sizeof(output_status));
}

void pan355_set_output(uint8_t index, void *value, char *type, void *generic_map)
{
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;

    if (0 == strcmp(type, "INTEGER8"))
    {
        i8 = (int8_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER16"))
    {
        i16 = (int16_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER32"))
    {
        i32 = (int32_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER64"))
    {
        i64 = (int64_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED8"))
    {
        u8 = (uint8_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED16"))
    {
        u16 = (uint16_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED32"))
    {
        u32 = (uint32_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED64"))
    {
        u64 = (uint64_t *)value;
    }
    else if (0 == strcmp(type, "REAL32"))
    {
        sr = (float *)value;
    }
    else if (0 == strcmp(type, "REAL64"))
    {
        dr = (double *)value;
    }

    pan355_output_map_t *output_map = (pan355_output_map_t *)generic_map;

    switch (index)
    {
    case 0:
        output_map->stp0_requested_opmode = *u8;
        break;
    case 1:
        output_map->stp0_target_acceleration = *u16;
        break;
    case 2:
        output_map->stp0_target_velocity = *i16;
        break;
    case 3:
        output_map->stp0_target_position = *i32;
        break;
    case 4:
        output_map->stp1_requested_opmode = *u8;
        break;
    case 5:
        output_map->stp1_target_acceleration = *u16;
        break;
    case 6:
        output_map->stp1_target_velocity = *i16;
        break;
    case 7:
        output_map->stp1_target_position = *i32;
        break;
    default:
        break;
    }
}

void io116e_send_input(char *response, void *generic_map)
{
    char input_status[100];
    io116e_input_map_t *input_map = (io116e_input_map_t *)generic_map;

    sprintf(input_status, "%d %d %d %d %d %d %d "
                          "%d %d %d %d %d %d %d "
                          "%d %d %d %d %d %d %d %d",
            input_map->status,
            input_map->error,
            input_map->temperature,
            input_map->output_state,
            input_map->input_state,
            input_map->triggered,
            input_map->count_0,
            input_map->count_1,
            input_map->count_2,
            input_map->count_3,
            input_map->count_4,
            input_map->count_5,
            input_map->count_6,
            input_map->count_7,
            input_map->adv_count_0,
            input_map->adv_count_1,
            input_map->adv_count_2,
            input_map->adv_count_3,
            input_map->adv_count_4,
            input_map->adv_count_5,
            input_map->adv_count_6,
            input_map->adv_count_7);

    memcpy(response, input_status, sizeof(input_status));
}

void io116e_send_output(char *response, void *generic_map)
{
    char output_status[200];
    io116e_output_map_t *output_map = (io116e_output_map_t *)generic_map;

    sprintf(output_status, "%d %d %d %d %d %d %d "
                           "%d %d %d %d %d %d %d "
                           "%d %d %d %d %d %d %d "
                           "%d %d %d %d %d %d %d ",
            output_map->output_cmd,
            output_map->out_prescale_0,
            output_map->out_prescale_1,
            output_map->out_prescale_2,
            output_map->out_prescale_3,
            output_map->out_update,
            output_map->in_prescale_0,
            output_map->in_prescale_1,
            output_map->in_prescale_2,
            output_map->in_prescale_3,
            output_map->in_prescale_4,
            output_map->in_prescale_5,
            output_map->in_prescale_6,
            output_map->in_prescale_7,
            output_map->in_update,
            output_map->in_filter,
            output_map->in_filter_enabled,
            output_map->duty_cycle_0,
            output_map->duty_cycle_1,
            output_map->duty_cycle_2,
            output_map->duty_cycle_3,
            output_map->pulse_offset_0,
            output_map->pulse_offset_1,
            output_map->pulse_count_0,
            output_map->pulse_count_1,
            output_map->pulse_count_2,
            output_map->pulse_count_3,
            output_map->pulse_updated);

    memcpy(response, output_status, sizeof(output_status));
}

void io116e_set_output(uint8_t index, void *value, char *type, void *generic_map)
{
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;

    if (0 == strcmp(type, "INTEGER8"))
    {
        i8 = (int8_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER16"))
    {
        i16 = (int16_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER32"))
    {
        i32 = (int32_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER64"))
    {
        i64 = (int64_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED8"))
    {
        u8 = (uint8_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED16"))
    {
        u16 = (uint16_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED32"))
    {
        u32 = (uint32_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED64"))
    {
        u64 = (uint64_t *)value;
    }
    else if (0 == strcmp(type, "REAL32"))
    {
        sr = (float *)value;
    }
    else if (0 == strcmp(type, "REAL64"))
    {
        dr = (double *)value;
    }

    io116e_output_map_t *output_map = (io116e_output_map_t *)generic_map;

    switch (index)
    {
    case 0:
        output_map->output_cmd = *u16;
        break;
    case 1:
        output_map->out_prescale_0 = *u8;
        break;
    case 2:
        output_map->out_prescale_1 = *u8;
        break;
    case 3:
        output_map->out_prescale_2 = *u8;
        break;
    case 4:
        output_map->out_prescale_3 = *u8;
        break;
    case 5:
        output_map->in_prescale_0 = *u16;
        break;
    case 6:
        output_map->in_prescale_1 = *u16;
        break;
    case 7:
        output_map->in_prescale_2 = *u16;
        break;
    case 8:
        output_map->in_prescale_3 = *u16;
        break;
    case 9:
        output_map->in_prescale_4 = *u16;
        break;
    case 10:
        output_map->in_prescale_5 = *u16;
        break;
    case 11:
        output_map->in_prescale_6 = *u16;
        break;
    case 12:
        output_map->in_prescale_7 = *u16;
        break;
    case 13:
        output_map->in_update = *u16;
        break;
    case 14:
        output_map->in_filter = *u32;
        break;
    case 15:
        output_map->in_filter_enabled = *u8;
        break;
    case 16:
        output_map->duty_cycle_0 = *u8;
        break;
    case 17:
        output_map->duty_cycle_1 = *u8;
        break;
    case 18:
        output_map->duty_cycle_2 = *u8;
        break;
    case 19:
        output_map->duty_cycle_3 = *u8;
        break;
    case 20:
        output_map->pulse_offset_0 = *u8;
        break;
    case 21:
        output_map->pulse_offset_1 = *u8;
        break;
    case 22:
        output_map->pulse_count_0 = *u32;
        break;
    case 23:
        output_map->pulse_count_1 = *u32;
        break;
    case 24:
        output_map->pulse_count_2 = *u32;
        break;
    case 25:
        output_map->pulse_count_3 = *u32;
        break;
    case 26:
        output_map->pulse_updated = *u32;
        break;
    case 27:
        output_map->trigger_count_0 = *u32;
        break;
    case 28:
        output_map->trigger_count_1 = *u32;
        break;
    case 29:
        output_map->trigger_count_2 = *u32;
        break;
    case 30:
        output_map->trigger_count_3 = *u32;
        break;
    case 31:
        output_map->trigger_count_4 = *u32;
        break;
    case 32:
        output_map->trigger_count_5 = *u32;
        break;
    case 33:
        output_map->trigger_count_6 = *u32;
        break;
    case 34:
        output_map->trigger_count_7 = *u32;
        break;
    case 35:
        output_map->trigger_count_8 = *u32;
        break;
    case 36:
        output_map->trigger_count_9 = *u32;
        break;
    case 37:
        output_map->trigger_count_10 = *u32;
        break;
    case 38:
        output_map->trigger_count_11 = *u32;
        break;
    case 39:
        output_map->trigger_count_12 = *u32;
        break;
    case 40:
        output_map->trigger_count_13 = *u32;
        break;
    case 41:
        output_map->trigger_count_14 = *u32;
        break;
    case 42:
        output_map->trigger_count_15 = *u32;
        break;
    case 43:
        output_map->trigger_enabled = *u32;
        break;
    case 44:
        output_map->trigger_updated = *u32;
        break;
    case 45:
        output_map->in_count_command = *u16;
        break;
    case 46:
        output_map->in_count_latch_command = *u8;
        break;
    case 47:
        output_map->in_count_latch_updated = *u8;
        break;
    default:
        break;
    }
}

void cmmt_send_input(char *response, void *generic_map)
{
    char input_status[100];
    cmmt_input_map_t *input_map = (cmmt_input_map_t *)generic_map;

    sprintf(input_status, "%d %d %d %d %d",
            input_map->status,
            input_map->display_op_mode,
            input_map->actual_position,
            input_map->actual_velocity,
            input_map->actual_torque);

    memcpy(response, input_status, sizeof(input_status));
}

void cmmt_send_output(char *response, void *generic_map)
{
    char output_status[200];
    cmmt_output_map_t *output_map = (cmmt_output_map_t *)generic_map;

    sprintf(output_status, "%d %d %d %d %d %d %d %d",
            output_map->ctrl_word,
            output_map->op_mode,
            output_map->target_position,
            output_map->profile_velocity,
            output_map->target_velocity,
            output_map->target_torque,
            output_map->velocity_offset,
            output_map->torque_offset);

    memcpy(response, output_status, sizeof(output_status));
}

void cmmt_set_output(uint8_t index, void *value, char *type, void *generic_map)
{
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;

    if (0 == strcmp(type, "INTEGER8"))
    {
        i8 = (int8_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER16"))
    {
        i16 = (int16_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER32"))
    {
        i32 = (int32_t *)value;
    }
    else if (0 == strcmp(type, "INTEGER64"))
    {
        i64 = (int64_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED8"))
    {
        u8 = (uint8_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED16"))
    {
        u16 = (uint16_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED32"))
    {
        u32 = (uint32_t *)value;
    }
    else if (0 == strcmp(type, "UNSIGNED64"))
    {
        u64 = (uint64_t *)value;
    }
    else if (0 == strcmp(type, "REAL32"))
    {
        sr = (float *)value;
    }
    else if (0 == strcmp(type, "REAL64"))
    {
        dr = (double *)value;
    }

    cmmt_output_map_t *output_map = (cmmt_output_map_t *)generic_map;

    switch (index)
    {
    case 0:
        output_map->ctrl_word = *u16;
        break;
    case 1:
        output_map->op_mode = *i8;
        printf("index: %d  ... value: %d", index, *(int8_t *)value);
        break;
    case 2:
        output_map->target_position = *i32;
        break;
    case 3:
        output_map->profile_velocity = *u32;
        break;
    case 4:
        output_map->target_velocity = *i32;
        break;
    case 5:
        output_map->target_torque = *i16;
        break;
    case 6:
        output_map->velocity_offset = *i32;
        break;
    case 7:
        output_map->torque_offset = *i16;
        break;
    default:
        break;
    }
}

const float angle_jaws_shut = (154.6 / 3.6);
const int32_t position_jaws_shut = angle_jaws_shut * 1000;
int32_t position_jaws_open = 0;
bool setpoint_set = false;
bool jaws_shut = false;
uint8_t position_offset = 0;

double position_calc;
int32_t position = 25000;

void jaw_toggle(uint8_t state)
{
    global_jaw_mode = state;
}

void jaw_set_angle(uint16_t offset)
{
    position_jaws_open = -offset * 1000;
}

void cmmt_jaw_mode(uint16_t angle, cmmt_output_map_t *output, cmmt_input_map_t *input)
{
    bool climbing = true;

    printf("SETPOINT: %d\n", position);
    while (input->status & CMD_SWITCH_ON)
    {
        printf("Switch on\n");
        if (!(input->status & STATUS_DRIVE_IS_MOVING))
        {
            printf("Drive NOT moving");
            if (setpoint_set)
            {
                printf("Setpoint set\n");
                output->ctrl_word = 127;
                setpoint_set = false;
            }
            else
            {
                printf("Setpoint NOT set\n");
                output->ctrl_word = 15;
                position = position * -1;
                output->target_position = position;
                printf("SETPOINT: %d\n", position);
                setpoint_set = true;
            }
        }
        osal_usleep(15000);
    }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;
    (void)ptr; /* Not used */

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int initialize_ethercat(char *ifname)
{
    int i;
    inOP = FALSE;
    int IO_map_size;
    int device_state;

    printf("ec: initialize_ethercat: start\n");

    /* Configure Ethercat master and bind socket to ifname */
    if (!ec_init(ifname))
    {
        printf("ec: initialize_ethercat: Failed to use %s - abort", ifname);
        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: ec_init on %s succeeded.\n", ifname);

    /* Enumerate and init all slaves */
    if (ec_config_init(FALSE) <= 0)
    {
        printf("ec: initialize_ethercat: Failed to init slaves - abort\n");
        ec_close();
        return -1;
    }

    /* Set slaves in init state */
    ec_slave[0].state = EC_STATE_INIT + EC_STATE_ACK;
    ec_writestate(0);

    device_state = ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE * 1);

    if (device_state != EC_STATE_INIT)
    {
        printf("ec: initialize_ethercat: Unable to set device in boot state device_state=%d - abort\n", device_state);
        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: init state reached\n");

    /* Set slaves in boot state and clear all errors (if any) */
    ec_slave[0].state = EC_STATE_BOOT + EC_STATE_ACK;
    ec_writestate(0);

    /* For some reason is this ready call needed */
    ec_readstate();

    device_state = ec_statecheck(0, EC_STATE_BOOT, EC_TIMEOUTSTATE * 1);

    if (device_state != EC_STATE_BOOT && device_state != EC_STATE_INIT)
    {
        printf("ec: initialize_ethercat: Unable to set device in boot or init state device_state=%d - abort\n", device_state);
        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: boot/init state reached - device_state=%d\n", device_state);

    /* Set slaves in init state */
    ec_slave[0].state = EC_STATE_INIT + EC_STATE_ACK;
    ec_writestate(0);

    device_state = ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE * 1);

    if (device_state != EC_STATE_INIT)
    {
        printf("ec: initialize_ethercat: Unable to set device in init state device_state=%d - abort\n", device_state);
        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: init state reached\n");

    /* Wait for slaves to come up */
    int temp_count = 0;
    do
    {
        ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE * 4);
        uint16_t w;
        temp_count = ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);
    } while (temp_count < ec_slavecount);

    printf("ec: initialize_ethercat: All slaves are back after init/boot/init seq.\n");

    /* Enumerate and init all slaves again after we have left boot state */
    if (ec_config_init(FALSE) <= 0)
    {
        printf("ec: initialize_ethercat: Failed to config init slaves - abort\n");
        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: ec_config_init again after boot state\n");

    /* Set slaves in pre-op state */
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);

    device_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 1);

    if (device_state != EC_STATE_PRE_OP)
    {
        printf("ec: initialize_ethercat: Unable to set device in pre-op state device_state=%d - abort\n", device_state);

        for (int i = 1; i <= ec_slavecount; i++)
        {
            device_state = ec_statecheck(i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 1);
            printf("\t slave %d state %d\n", i, device_state);
        }

        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: pre-op state reached\n");

    /* Map all PDOs from slaves to IOmap with Outputs/Inputs in sequential order (legacy SOEM way).*/
    IO_map_size = ec_config_map(&IOmap);

    printf("\tec: initialize_ethercat: IO_map_size %d\n", IO_map_size);

    /* Check actual slave state. This is a blocking function. */
    device_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    if (device_state != EC_STATE_SAFE_OP)
    {
        printf("ec: initialize_ethercat: Unable to set device in safe-op state device_state=%d - abort\n", device_state);
        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: save-op state reached\n");

    /* Make sure all slaves are in safe-op */
    if (ec_slave[0].state != EC_STATE_SAFE_OP)
    {
        printf("ec: initialize_ethercat: Not all slaves reached safe operational state device_state= %d - abort\n", device_state);
        ec_readstate();
        for (i = 1; i <= ec_slavecount; i++)
        {
            if (ec_slave[i].state != EC_STATE_SAFE_OP)
            {
                printf("ec: initialize_ethercat: Slave %d State=%2x StatusCode=%4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }

        ec_close();
        return -1;
    }

    printf("ec: initialize_ethercat: initialized\n");

    return 0;
}

float cycle_count = 0;
int counter_counter = 0;
int counter_set = 0;

#ifdef STEP MOTOR HOMING + DX3
int own_counter_1 = 0;
int own_counter_2 = 0;
uint8_t homing_on_found_offset = 0;
static int ethercat_loop_counter = 0;

int16_t torque_actual_value;
int size_torque = sizeof(torque_actual_value);
int32 homing_torque_limit = 300;

bool homing_done = false;

static time_t homing_start_time = 0;
static bool delay_done = false;
int32 pose_offset;
int32 target_1 = 400000;
int32 target_2 = 200000;
static bool target_1_reached = false;
static bool target_2_reached = false;

typedef enum
{
    STATE_HOMING_1,
    STATE_HOMING_2,
    STATE_DELAY,
    STATE_CONFIGURE_TARGET_1,
    STATE_CONFIGURE_TARGET_2,
    STATE_PREPARE_TARGET_1,
    STATE_MOVE_TO_TARGET_1,
    STATE_REACHED_TARGET_1,
    STATE_PREPARE_TARGET_2,
    STATE_MOVE_TO_TARGET_2,
    STATE_REACHED_TARGET_2,
    STOP
} MotorState;

MotorState motor_state = STATE_HOMING_1;

#endif

// STEP MOTOR PAN355
void set_timeout(uint16_t timeout)
{
    ec_SDOwrite(1, PDO_COM_TIMEOUT_CONFIGURATION, 0x01, FALSE, sizeof(timeout), &timeout, EC_TIMEOUTRXM);
}

void set_stp_0_action(uint8_t stp_0_action)
{
    ec_SDOwrite(1, PDO_COM_TIMEOUT_CONFIGURATION, 0x02, FALSE, sizeof(stp_0_action), &stp_0_action, EC_TIMEOUTRXM);
}

void set_stp_1_action(uint8_t stp_1_action)
{
    ec_SDOwrite(1, PDO_COM_TIMEOUT_CONFIGURATION, 0x03, FALSE, sizeof(stp_1_action), &stp_1_action, EC_TIMEOUTRXM);
}

void set_stp_pulse_div(uint16_t STP_CONFIGURATION, uint8_t pulse_div)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x01, FALSE, sizeof(pulse_div), &pulse_div, EC_TIMEOUTRXM);
}

void set_stp_ramp_div(uint16_t STP_CONFIGURATION, uint8_t ramp_div)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x02, FALSE, sizeof(ramp_div), &ramp_div, EC_TIMEOUTRXM);
}

void set_stp_i_hold(uint16_t STP_CONFIGURATION, uint8_t i_hold)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x03, FALSE, sizeof(i_hold), &i_hold, EC_TIMEOUTRXM);
}

void set_stp_i_run(uint16_t STP_CONFIGURATION, uint8_t i_run)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x04, FALSE, sizeof(i_run), &i_run, EC_TIMEOUTRXM);
}

void set_stp_max_position_error(uint16_t STP_CONFIGURATION, uint32_t max_pos_error)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x05, FALSE, sizeof(max_pos_error), &max_pos_error, EC_TIMEOUTRXM);
}

void set_stp_run_current_timeout(uint16_t STP_CONFIGURATION, uint32_t current_timeout)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x06, FALSE, sizeof(current_timeout), &current_timeout, EC_TIMEOUTRXM);
}

void set_stp_microstep_setting(uint16_t STP_CONFIGURATION, uint8_t microstep_setting)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x07, FALSE, sizeof(microstep_setting), &microstep_setting, EC_TIMEOUTRXM);
}

void set_stp_invert_motor_direction(uint16_t STP_CONFIGURATION, uint8_t invert_motor_dir)
{
    ec_SDOwrite(1, STP_CONFIGURATION, 0x08, FALSE, sizeof(invert_motor_dir), &invert_motor_dir, EC_TIMEOUTRXM);
}

static int ethercat_loop_counter = 0;
void ethercat_loop(void)
{
    int i, chk;
    int slave_state;
    inOP = FALSE;

#ifdef PAN355
    uint8_t u8_val = 0;
    uint16_t u16_val = 0x03e8;
    uint32_t u32_val = 0x01;
    ec_SDOwrite(1, TIMEOUT_CONF, 0x01, FALSE, sizeof(u16_val), &u16_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, TIMEOUT_CONF, 0x02, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, TIMEOUT_CONF, 0x03, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
    u8_val = 0;
    ec_SDOwrite(1, STP0_CONF, 0x01, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x01, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    u8_val = 8;
    ec_SDOwrite(1, STP0_CONF, 0x02, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x02, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    u8_val = 10;
    ec_SDOwrite(1, STP0_CONF, 0x03, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x03, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    u8_val = 15;
    ec_SDOwrite(1, STP0_CONF, 0x04, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x04, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    u32_val = 0;
    ec_SDOwrite(1, STP0_CONF, 0x05, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x05, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
    u32_val = 0;
    ec_SDOwrite(1, STP0_CONF, 0x06, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x06, FALSE, sizeof(u32_val), &u32_val, EC_TIMEOUTRXM);
    u8_val = 8;
    ec_SDOwrite(1, STP0_CONF, 0x07, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x07, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    u8_val = 0;
    ec_SDOwrite(1, STP0_CONF, 0x08, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
    ec_SDOwrite(1, STP1_CONF, 0x08, FALSE, sizeof(u8_val), &u8_val, EC_TIMEOUTRXM);
#endif

    printf("ec: ethercat_loop: start\n");
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("ec: ethercat_loop: Calculated workcounter %d\n", expectedWKC);
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);
    chk = 40;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        slave_state = ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    printf("ec: ethercat_loop: All slaves should be in operational state : new state = %d chk=%d\n", slave_state, chk);

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        printf("ec: ethercat_loop: Not all slaves reached operational state.\n");
        ec_readstate();

        for (i = 1; i <= ec_slavecount; i++)
        {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                printf("ec: ethercat_loop: Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }

        printf("ec: ethercat_loop: Unable to get all ethercat devices in operation mode - giving up :(\n");
        return;
    }

    inOP = TRUE;

    ethercat_loop_counter = 0;

#ifdef EIO116
    input_map = (io116e_input_map_t *)ec_slave[1].inputs;
    output_map = (io116e_output_map_t *)ec_slave[1].outputs;
#endif

#ifdef PAN355
    input_map = (pan355_input_map_t *)ec_slave[1].inputs;
    output_map = (pan355_output_map_t *)ec_slave[1].outputs;
#endif

#ifdef MX2
    input_map = (mx2_input_map_t *)ec_slave[1].inputs;
    output_map = (mx2_output_map_t *)ec_slave[1].outputs;
#endif

#ifdef CMMT
    input_map = (cmmt_input_map_t *)ec_slave[1].inputs;
    output_map = (cmmt_output_map_t *)ec_slave[1].outputs;
    uint8_t climbing = 0;
    uint32_t velocity = 20;
    int32_t loop_count = 0;
#endif

#ifdef STEP MOTOR HOMING + DX3

    input_map = (dx3_input_map_t *)ec_slave[1].inputs;
    output_map = (dx3_output_map_t *)ec_slave[1].outputs;
#endif

    // STEP MOTOR PAN355
    ec_slave_stp_output *stp_out_ptr = (ec_slave_stp_output *)ec_slave[1].outputs;
    ec_slave_stp_input *stp_in_ptr = (ec_slave_stp_input *)ec_slave[1].inputs;

    typedef enum
    {
        MOTOR_STATE_IDLE,
        MOTOR_STATE_MOVE_FORWARD,
        MOTOR_STATE_MOVE_BACKWARD,
        MOTOR_STATE_IDLE_PERIOD
    } MotorState;

    MotorState motor_state = MOTOR_STATE_IDLE;

    int initial_position;
    int target_position;
    int forward_target = 20000;
    int backward_target = -20000;
    bool moving_to_position = false;
    int idle_counter = 0;

    set_timeout(1000);
    set_stp_0_action(1);
    set_stp_1_action(1);
    set_stp_pulse_div(STP0_CONFIGURATION, pulse_div_value);
    set_stp_ramp_div(STP0_CONFIGURATION, ramp_div_value);
    set_stp_i_hold(STP0_CONFIGURATION, i_hold_value);
    set_stp_i_run(STP0_CONFIGURATION, i_run_value);
    set_stp_max_position_error(STP0_CONFIGURATION, max_pos_error_value);
    set_stp_run_current_timeout(STP0_CONFIGURATION, current_timeout_value);
    set_stp_microstep_setting(STP0_CONFIGURATION, microstep_setting_value);
    set_stp_invert_motor_direction(STP0_CONFIGURATION, invert_motor_dir_value);

    int wkc_current = wkc;
    printf("Start ethercat loop\n");
    global_status_loop = 1;


    /* Ethercat cyclic loop */
    while (1)
    {

        ethercat_loop_counter++;

        if (ethercat_loop_counter % 200 == 100)
            printf("ethercat_loop_counter %d slaves %d\n", ethercat_loop_counter, ec_slavecount);

        if (global_jaw_mode)
        {
            if (!counter_set)
            {
                counter_counter = ethercat_loop_counter;
                counter_set = true;
            }
            if (input_map->status & CMD_SWITCH_ON)
            {
                if (input_map->status & STATUS_TARGET_REACHED)
                {
                    if (setpoint_set)
                    {
                        output_map->ctrl_word = 63;
                        setpoint_set = false;
                        cycle_count += 0.5;
                    }
                    else
                    {
                        output_map->ctrl_word = 15;
                        if (!jaws_shut)
                        {
                            position = -position_jaws_shut;
                            jaws_shut = true;
                            setpoint_set = true;
                            output_map->target_position = position;
                        }
                        else
                        {
                            if ((counter_counter + 200) <= ethercat_loop_counter)
                            {
                                position = position_jaws_open;
                                jaws_shut = false;
                                counter_counter = ethercat_loop_counter;
                                setpoint_set = true;
                                output_map->target_position = position;
                            }
                        }
                    }
                }
            }
        }
        else if (counter_set)
        {
            printf("CYCLES: %f\n", cycle_count);
            counter_set = false;
            cycle_count = 0;
        }

        /*

        #ifdef DX3 Jonas

                  if(ethercat_loop_counter % 200 == 100){
                      printf("Statusword: 0x%4.4x\n", input_map->status);
                      printf("Error: 0x%4.4x\n", input_map->error);
                  }

                  uint32_t acc;
                  int size_acc = sizeof(acc);
                  ec_SDOread(1, 0x6083, 0x00, FALSE, &size_acc, &acc, EC_TIMEOUTSAFE);
                  if(ethercat_loop_counter % 500 == 0) printf("before acc sdo is : %d\n", acc);
                  if(ethercat_loop_counter % 500 == 0) printf("before acc ptr is : %d\n", input_map->status);

                  uint32_t dec;
                  int size_dec = sizeof(dec);
                  ec_SDOread(1, 0x6084, 0x00, FALSE, &size_dec, &dec, EC_TIMEOUTSAFE);
                  if(ethercat_loop_counter % 500 == 0) printf("before dec sdo is : %d\n", dec);
                  if(ethercat_loop_counter % 500 == 0) printf("before dec ptr is : %d\n", output_map->profile_deceleration);


                  output_map->profile_acceleration = 1000;
                  output_map->profile_deceleration = 500;

                  uint32_t acc_2;
                  int size_acc_2 = sizeof(acc);
                  ec_SDOread(1, 0x6083, 0x00, FALSE, &size_acc_2, &acc, EC_TIMEOUTSAFE);
                  if(ethercat_loop_counter % 500 == 0) printf("after acc sdo is : %d\n", acc);
                  if(ethercat_loop_counter % 500 == 0) printf("after acc ptr is : %d\n", output_map->profile_acceleration);

                  uint32_t dec_2;
                  int size_dec_2 = sizeof(dec);
                  ec_SDOread(1, 0x6084, 0x00, FALSE, &size_dec_2, &dec, EC_TIMEOUTSAFE);
                  if(ethercat_loop_counter % 500 == 0) printf("after dec sdo is : %d\n", dec);
                  if(ethercat_loop_counter % 500 == 0) printf("after dec ptr is : %d\n", output_map->profile_deceleration);


                   if(ethercat_loop_counter == 1000)
                      output_map->ctrl_word = 0x0006;

                  if(ethercat_loop_counter == 1010)
                      output_map->ctrl_word = 0x0007;

                  if(ethercat_loop_counter == 1020)
                      output_map->ctrl_word = 0x000F;

                  if(ethercat_loop_counter == 1030)
                      output_map->operation_mode = 0x0006;

                  if(ethercat_loop_counter == 1050)
                      output_map->ctrl_word = 31;

                  if(ethercat_loop_counter == 1090)
                      output_map->ctrl_word = 0x000F;

                  if(ethercat_loop_counter == 1100)
                      output_map->operation_mode = 0x0001;

                  if(ethercat_loop_counter == 2000)
                      output_map->target_position = 25000;

                  if(ethercat_loop_counter == 2010)
                      output_map->ctrl_word = 0x007F;

                  if(ethercat_loop_counter == 2100){
                      output_map->ctrl_word = 0x000F;
                      output_map->target_position = -15000;
                  }

                  if(ethercat_loop_counter == 2110)
                      output_map->ctrl_word = 0x007F;

                  if(ethercat_loop_counter == 2200){
                      output_map->ctrl_word = 0x000F;
                      output_map->target_position = 35000;
                  }

                  if(ethercat_loop_counter == 2210)
                      output_map->ctrl_word = 0x007F;

                  if(ethercat_loop_counter == 2300){
                      output_map->ctrl_word = 0x000F;
                      output_map->target_position = -15000;
                  }

                  if(ethercat_loop_counter == 2310)
                      output_map->ctrl_word = 0x007F;

                  if(ethercat_loop_counter == 2400){
                      output_map->ctrl_word = 0x000F;
                      output_map->target_position = 45000;
                  }

                  if(ethercat_loop_counter == 2410)
                      output_map->ctrl_word = 0x007F;

        #endif

        */

#ifdef STEP MOTOR HOMING + DX3
        ec_SDOread(1, TORQUE_ACTUAL_INDEX, 0x00, FALSE, &size_torque, &torque_actual_value, EC_TIMEOUTRXM);
        if (ethercat_loop_counter <= 3500 && ethercat_loop_counter % 30 == 0)
            printf("actual torque is: %d\n", torque_actual_value);

        if (ethercat_loop_counter == 1000)
        {
            output_map->ctrl_word = 0x000F; // 15
            output_map->target_velocity = 0x0003;
            output_map->profile_acceleration = 0x0003;
            output_map->profile_deceleration = 0x0003;
        }

        if (ethercat_loop_counter == 1010)
        {
            output_map->ctrl_word = 0x80; //&128
        }

        if (ethercat_loop_counter == 1100)
        {
            output_map->ctrl_word = 0x0006; // 6
        }

        if (ethercat_loop_counter == 1200)
            output_map->ctrl_word = 0x000F; // 15

        if (ethercat_loop_counter == 1250)
        {
            output_map->ctrl_word = 0x000F;    // 15
            output_map->operation_mode = 0x03; // 3    profile velocity
        }

        if (ethercat_loop_counter >= 1300 && !homing_done)
        {
            output_map->ctrl_word = 0x007F; // 127

            ec_SDOread(1, TORQUE_ACTUAL_INDEX, 0x00, FALSE, &size_torque, &torque_actual_value, EC_TIMEOUTRXM);

            if (ethercat_loop_counter >= 1400 && !homing_done && abs(torque_actual_value) >= abs(homing_torque_limit))
            {
                printf("Torque limit reached: %d, setting homing...\n", torque_actual_value);

                pose_offset = input_map->actual_position;
                printf("Offset position: %d\n", pose_offset);

                output_map->ctrl_word = 0x010F; // 271 stop motor

                homing_done = true;
                homing_start_time = time(NULL);
                delay_done = false;
            }
        }

        switch (motor_state)
        {
        case STATE_HOMING_1:
            if (homing_done && homing_on_found_offset == 0)
            {
                output_map->operation_mode = 0x06; // Homing mode
                output_map->ctrl_word = 15;
                homing_on_found_offset = 1;
                printf("Homing step 1 completed.\n");
                motor_state = STATE_HOMING_2;
            }
            break;

        case STATE_HOMING_2:

            if (homing_done && homing_on_found_offset == 1)
            {
                output_map->ctrl_word = 31;
                homing_on_found_offset = 2;
                printf("Homing done.\n");
                homing_start_time = time(NULL);
                motor_state = STATE_DELAY;
            }
            break;

        case STATE_DELAY:

            if (homing_done && !delay_done)
            {
                time_t current_time = time(NULL);
                if (difftime(current_time, homing_start_time) >= 2.0)
                {
                    delay_done = true;
                    printf("5-second delay completed. Preparing target_1 configuration.\n");
                    own_counter_1 = ethercat_loop_counter + 10;
                    motor_state = STATE_CONFIGURE_TARGET_1;
                }
            }
            break;

        case STATE_CONFIGURE_TARGET_1:
            if (ethercat_loop_counter >= 1500 && delay_done)
            {
                output_map->ctrl_word = 0x000F; // 15
                output_map->profile_velocity = 0x0005;
                output_map->profile_acceleration = 0x0001;
                output_map->profile_deceleration = 0x0001;

                output_map->operation_mode = 0x01; // 1
                printf("Target_1 is configured. \n");

                own_counter_1 = ethercat_loop_counter + 10;
                motor_state = STATE_PREPARE_TARGET_1;
            }
            break;

        case STATE_PREPARE_TARGET_1:
            if (ethercat_loop_counter == own_counter_1)
            {

                output_map->target_position = -target_1;
                printf("Target 1 set to: %d\n", output_map->target_position);
                motor_state = STATE_MOVE_TO_TARGET_1;
            }
            break;

        case STATE_MOVE_TO_TARGET_1:
        {
            output_map->ctrl_word = 63; //  absolute position
            motor_state = STATE_REACHED_TARGET_1;
        }
        break;

        case STATE_REACHED_TARGET_1:
            if (!target_1_reached)
            {
                if (ethercat_loop_counter % 4 == 0)
                    printf("Pose after homing :%d\n", input_map->actual_position);

                if (input_map->actual_position <= -target_1 && input_map->actual_position >= -target_1 - 3000)
                {
                    printf("Target 1 reached. Actual position: %d\n", input_map->actual_position);
                    target_1_reached = true;
                    own_counter_2 = ethercat_loop_counter + 1;
                    motor_state = STATE_CONFIGURE_TARGET_2;
                }
                else
                {
                    motor_state = STATE_MOVE_TO_TARGET_1;
                }
            }
            break;

        case STATE_CONFIGURE_TARGET_2:
        {
            if (target_1_reached && ethercat_loop_counter == own_counter_2)
            {
                output_map->ctrl_word = 0x000F; // 15
                output_map->profile_velocity = 0x0005;
                output_map->profile_acceleration = 0x0001;
                output_map->profile_deceleration = 0x0001;

                output_map->operation_mode = 0x01; // 1
                printf("Target_2 is configured. \n");
                motor_state = STATE_PREPARE_TARGET_2;
            }
        }
        break;

        case STATE_PREPARE_TARGET_2:
        {
            if (target_1_reached && ethercat_loop_counter == own_counter_2 + 10)
            {
                output_map->target_position = target_2;
                printf("Target 2 set to: %d\n", output_map->target_position);
                motor_state = STATE_MOVE_TO_TARGET_2;
                target_1_reached = false;
            }
        }
        break;

        case STATE_MOVE_TO_TARGET_2:
        {
            output_map->ctrl_word = 127; // relativ position
            target_2_reached = false;
            motor_state = STATE_REACHED_TARGET_2;
        }
        break;

        case STATE_REACHED_TARGET_2:
            if (!target_2_reached)
            {
                if (ethercat_loop_counter % 3 == 0)
                    printf("Pose after target_1 :%d\n", input_map->actual_position);

                if (input_map->actual_position >= -target_1 + target_2 && input_map->actual_position <= -target_1 + target_2 + 3000)
                {
                    printf("Target 2 reached. Actual position: %d\n", input_map->actual_position);
                    target_2_reached = true;
                    motor_state = STATE_CONFIGURE_TARGET_1;
                }
                else
                {
                    motor_state = STATE_MOVE_TO_TARGET_2;
                }
            }
            break;

        default:
            printf("Unknown state!\n");
            break;
        }
#endif

// STEP MOTOR PAN355
        if (ethercat_loop_counter == 10)
        {
            stp_out_ptr->stp_target_acceleration_0 = 1000;
            stp_out_ptr->stp_target_velocity_0 = 100;
        }

        switch (motor_state)
        {
        case MOTOR_STATE_IDLE:
            if (ethercat_loop_counter > 200 && !moving_to_position)
            {
                initial_position = stp_in_ptr->stp_actual_position_0;
                printf("Initial position for forward: %d\n", initial_position);
                target_position = initial_position + forward_target;
                stp_out_ptr->stp_target_position_0 = target_position;
                stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_POSITION;
                moving_to_position = true;
                motor_state = MOTOR_STATE_MOVE_FORWARD;
            }
            break;

        case MOTOR_STATE_MOVE_FORWARD:
            if (stp_in_ptr->stp_actual_position_0 >= target_position)
            {
                printf("Reached forward target position:  %d\n", stp_in_ptr->stp_actual_position_0);
                stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_IDLE;
                moving_to_position = false;
               // idle_counter = ethercat_loop_counter + 300;
                motor_state = MOTOR_STATE_IDLE_PERIOD;
            }
            break;

        case MOTOR_STATE_IDLE_PERIOD:
            if (ethercat_loop_counter >= idle_counter)
            {initial_position = stp_in_ptr->stp_actual_position_0;
                printf("Initial position for backward: %d\n", initial_position);
                target_position = initial_position + backward_target;
                stp_out_ptr->stp_target_position_0 = target_position;
                stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_POSITION;
                printf("Switching to backward movement.\n");
                motor_state = MOTOR_STATE_MOVE_BACKWARD;
            }
            break;

        case MOTOR_STATE_MOVE_BACKWARD:
            if (stp_in_ptr->stp_actual_position_0 <= target_position)
            {
                printf("Reached backward target position:  %d\n", stp_in_ptr->stp_actual_position_0);
                stp_out_ptr->stp_requested_opmode_0 = PAN355_OPMODE_IDLE;
                printf("Motor is now in idle mode.\n");
                moving_to_position = false;
              //  idle_counter = ethercat_loop_counter + 300;
                motor_state = MOTOR_STATE_IDLE;
            }
            break;
        }

#ifdef CMMT
        if (ethercat_loop_counter % 200 == 100)
        {
            printf("\n\nStatus: %d\n"
                   "-- OP_MODE: %d\n"
                   "-- ACTUAL_POSITION: %d\n"
                   "-- ACTUAL_VELOCITY: %d\n"
                   "-- ACTUAL_TORQUE: %d\n"
                   "-- NOT_READY: %s --\n"
                   "-- SWITCH_ON_DISABLED: %s --\n"
                   "-- READY_TO_SWITCH_ON: %s --\n"
                   "-- SWITCHED_ON: %s --\n"
                   "-- OP_ENABLED: %s --\n"
                   "-- QUICK_STOP_ACTIVE: %s --\n"
                   "-- FAULT_REACTION_ACTIVE: %s --\n"
                   "-- FAULT: %s --\n\n"
                   "-- TARGET POSITION: %d --\n"
                   "-- TARGET_VELOCITY: %d --\n"
                   "-- TARGET_TORQUE: %d --\n\n",
                   input_map->status,
                   input_map->display_op_mode,
                   input_map->actual_position,
                   input_map->actual_velocity,
                   input_map->actual_torque,
                   input_map->status & NOT_READY ? "TRUE" : "FALSE",
                   input_map->status & SWITCH_ON_DISABLED ? "TRUE" : "FALSE",
                   input_map->status & READY_TO_SWITCH_ON ? "TRUE" : "FALSE",
                   input_map->status & SWITCHED_ON ? "TRUE" : "FALSE",
                   input_map->status & OP_ENABLED ? "TRUE" : "FALSE",
                   input_map->status & QUICK_STOP_ACTIVE ? "TRUE" : "FALSE",
                   input_map->status & FAULT_REACTION_ACTIVE ? "TRUE" : "FALSE",
                   input_map->status & FAULT ? "TRUE" : "FALSE",
                   output_map->target_position,
                   output_map->target_velocity,
                   output_map->target_torque);
        }

        if (ethercat_loop_counter == 500)
        {
            output_map->ctrl_word = 128;
        }

        if (ethercat_loop_counter == 501)
        {
            output_map->ctrl_word = 0x00;
        }

        if (ethercat_loop_counter == 3000)
        {
            output_map->op_mode = 0x01;
            output_map->target_velocity = 10000;
            output_map->profile_velocity = 100;
            output_map->target_torque = 10;
            output_map->target_position = 35000;
        }

        if (ethercat_loop_counter == 3001)
        {
            output_map->ctrl_word |= CMD_OPMODE_02;
            output_map->ctrl_word |= CMD_OPMODE_01;
        }

        if (ethercat_loop_counter % 200 == 100 && ethercat_loop_counter > 3500)
        {
            if (input_map->actual_position + 20000 >= 99999)
            {
                climbing = 0;
            }
            else if (input_map->actual_position - 20000 <= 0)
            {
                climbing = 1;
            }

            if (velocity < 100)
            {
                velocity += 20;
            }
            else
                velocity = 100;

            output_map->op_mode = 0x01;
            output_map->target_velocity = 10000;
            output_map->profile_velocity = velocity;
            output_map->target_torque = 10;
            output_map->target_position = climbing ? 20000 : -20000;

            loop_count = ethercat_loop_counter + 10;
        }

        if (ethercat_loop_counter == loop_count && loop_count != 0)
        {
            output_map->ctrl_word |= CMD_OPMODE_02;
            output_map->ctrl_word |= CMD_OPMODE_03;
            output_map->ctrl_word |= CMD_OPMODE_01;
        }

        if (ethercat_loop_counter % 500 == 100 && ethercat_loop_counter > 999)
        {

            if (input_map->status & SWITCH_ON_DISABLED)
            {
                printf("SHUTDOWN CMD SENT\n");
                output_map->ctrl_word = CMD_SHUTDOWN;
            }
            else if (input_map->status & READY_TO_SWITCH_ON)
            {
                output_map->ctrl_word = CMD_SWITCH_ON;
            }
            else if (input_map->status & QUICK_STOP_ACTIVE)
            {
                output_map->ctrl_word = CMD_OP_ENABLE;
            }
            if (input_map->status & SWITCHED_ON)
            {
            }
        }

#endif

#ifdef MX2
        int16_t forward = 1;
        int16_t reverse = 2;
        int16_t stop = 0;

        if (ethercat_loop_counter == 1)
        {
            output_map->cmd = forward;
            output_map->freq_reference = 600;
        }

        if (ethercat_loop_counter == 1000)
        {
            output_map->cmd = stop;
        }
        if (ethercat_loop_counter == 1500)
        {
            output_map->cmd = reverse;
        }
        if (ethercat_loop_counter == 2000)
        {
            output_map->cmd = stop;
        }
#endif
        if (global_get_info)
            global_get_info = retreive_info(global_sdo_read, global_pdo_read);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (global_kill_loop)
        {
            global_kill_loop = 0;
            break;
        }

        int wkc_check = ec_read_all_states() * 3;

        if (wkc_check != wkc_current)
        {
            if (wkc_check > wkc_current)
                printf("---------------------------------------------------------------\n"
                       "New slave connected! ---wkc: %d - new wkc: %d--- Reinitializing\n"
                       "---------------------------------------------------------------\n",
                       wkc_current, wkc_check);
            else if (wkc_check < wkc)
                printf("---------------------------------------------------------------\n"
                       "     Slave lost! ---wkc: %d - new wkc: %d--- Reinitializing\n"
                       "---------------------------------------------------------------\n",
                       wkc_current, wkc_check);
            break;
        }

        /* If work counter is not as expected - report and continue */
        // if(wkc < expectedWKC) {
        //	printf("wkc not increasing wkc=%d expectedWKC=%d\n", wkc, expectedWKC);
        //	osal_usleep(5000);
        //	continue;
        // }

        osal_usleep(5000);
    }

    /* Stop ethercat and close socket */
    ec_close();
    global_status_loop = 0;
}
