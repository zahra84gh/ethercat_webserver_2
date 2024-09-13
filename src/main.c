#include <sys/types.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <microhttpd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/sendfile.h>
#include "include/main.h"
#include <cJSON.h>
#include <ctype.h>
#include "include/soem.h"
#include "osal.h"
#include "osal_defs.h"

#define PORT 8888

OSAL_THREAD_HANDLE thread1;

char *askpage;

const char *greatingpage = "<html><body><h1>Welcome, %s!</center></h1></body></html>";

const char *errorpage = "<html><body>This doesn't seem to be right.</body></html>";

char current_adapter[20] = "";

int print_out_key(void *cls, enum MHD_ValueKind kind,
                  const char *key, const char *value)
{
    printf("%s: %s\n", key, value);
    return MHD_YES;
}

static int send_object(struct MHD_Connection *connection,
                     char *response_string, int status_code)
{
    int ret;
    struct MHD_Response *response;

    response = MHD_create_response_from_buffer(strlen(response_string), (void *)response_string, MHD_RESPMEM_PERSISTENT);
    if (!response)
        return MHD_NO;

    ret = MHD_queue_response(connection, status_code, response);
    MHD_destroy_response(response);

    return ret;
}

static int send_page(struct MHD_Connection *connection,
                     int fd, size_t fsize, int status_code)
{
    int ret;
    struct MHD_Response *response;
    response = MHD_create_response_from_fd(fsize, fd);
    if (!response)
        return MHD_NO;
    ret = MHD_queue_response(connection, status_code, response);
    MHD_destroy_response(response);

    return ret;
}

void request_completed(void *cls, struct MHD_Connection *connection,
                       void **con_cls,
                       enum MHD_RequestTerminationCode toe)
{
    struct connection_info_struct *con_info = *con_cls;

    if (NULL == con_info)
        return;
    if (con_info->connectiontype == POST)
    {
        MHD_destroy_post_processor(con_info->postprocessor);
        if (con_info->answerstring){
            free(con_info->answerstring);
        }
            
    }
    free(con_info);
    *con_cls = NULL;
}

char *generate_input_response(char *data){
    return data;
}

static int iterate_post(void *coninfo_cls, enum MHD_ValueKind kind, const char *key,
                        const char *filename, const char *content_type,
                        const char *transfer_encoding, const char *data,
                        uint64_t off, size_t size)
{
    struct connection_info_struct *con_info = coninfo_cls;

    if(0 == strcmp(key, "adapter_request"))
    {
        char *adapter_buffer = malloc(2048);
        get_adapters(adapter_buffer);
        con_info->answer_type = OBJECT;
        char *answerstring;
        answerstring = malloc(MAXANSWERSIZE);
        strcpy(answerstring, adapter_buffer);
        con_info->answerstring = answerstring;
        return MHD_YES;
    }

    if(0 == strcmp(key, "slave_request"))
    {
        char *response;
        response = malloc(MAXANSWERSIZE);
        
        strcpy(current_adapter, data);
        init_ethercat_loop();
        ethcercat_slave_collection_t *slaves = get_all_slaves();
        uint8_t n_slaves = slaves->slave_cnt;
        if(slaves == NULL){
            printf("No slaves found\n");
            n_slaves = 0;
        }
        char buffer[100];
        if(n_slaves == 0){
            sprintf(response, "{}");
        }
        else{
            sprintf(response, "{");
            for(int i=0; i<n_slaves; i++){
                
                sprintf(buffer, "\"%d\":{\"name\": \"%s\", \"uid\": \"%s\"},", slaves->_slaves[i].slave_id, slaves->_slaves[i].name, slaves->_slaves[i].uid);
                strcat(response, buffer);
            }
            response[strlen(response) - 1] = '\0';
            strcat(response, "}");
        }
        con_info->answer_type = OBJECT;
        char *answerstring;
        answerstring = malloc(MAXANSWERSIZE);
        strcpy(answerstring, response);
        con_info->answerstring = answerstring;
        return MHD_YES;
    }

    if(0 == strcmp(key, "sdo_request"))
    {
        char filename[50];
        char *sdo_buffer;
        sprintf(filename, "./%s_sdo", data);
        FILE *fp = fopen(filename, "r");
        if(fp == NULL){
            printf("SDO file not found. Connection to slave %s", data);
            retreive_info();
            fp = fopen(filename, "r");
        }
        int fd = fileno(fp);
        printf("filename: %s\n", filename);
        fseek(fp, 0L, SEEK_END);
        size_t sdo_size = ftell(fp);
        fseek(fp, 0L, SEEK_SET);

        /* sdo_buffer = malloc(sdo_size);
        char *adapter_buffer = malloc(2048);
        fgets(sdo_buffer, sdo_size, fp);

        get_adapters(adapter_buffer); */
        char *answerstring;
        answerstring = malloc(MAXANSWERSIZE);
        strcpy(answerstring, filename);
        con_info->answer_type = PAGE;
        con_info->answerfd = fd; 
        con_info->size = sdo_size;
         con_info->answerstring = answerstring;
        return MHD_YES;
    }

    if(0 == strcmp(key, "pdo_request"))
    {
        char filename[50];
        char *sdo_buffer;
        sprintf(filename, "./%s_pdo", data);
        FILE *fp = fopen(filename, "r");
        if(fp == NULL){
            printf("SDO file not found. Connection to slave %s", data);
            retreive_info();
            fp = fopen(filename, "r");
        }
        int fd = fileno(fp);
        printf("filename: %s\n", filename);
        fseek(fp, 0L, SEEK_END);
        size_t sdo_size = ftell(fp);
        fseek(fp, 0L, SEEK_SET);

        /* sdo_buffer = malloc(sdo_size);
        char *adapter_buffer = malloc(2048);
        fgets(sdo_buffer, sdo_size, fp);

        get_adapters(adapter_buffer); */
        char *answerstring;
        answerstring = malloc(MAXANSWERSIZE);
        strcpy(answerstring, filename);
        con_info->answer_type = PAGE;
        con_info->answerfd = fd; 
        con_info->size = sdo_size;
        con_info->answerstring = answerstring;
        return MHD_YES;
    }

    if(0 == strcmp(key, "get_input_status")){
        char *response = malloc(MAXANSWERSIZE);
        ethcercat_slave_collection_t *slaves = get_all_slaves();
        for(int i=0; i<slaves->slave_cnt; i++){
            if(0 == strcmp(data, slaves->_slaves[i].uid))
                slaves->_slaves[i].send_input(response, slaves->_slaves[i].input_map);
        }
        con_info->answer_type = OBJECT;
        con_info->answerstring = response;
        return MHD_YES;
    }

    if(0 == strcmp(key, "get_output_status")){
        char *response = malloc(MAXANSWERSIZE);
        ethcercat_slave_collection_t *slaves = get_all_slaves();
        for(int i=0; i<slaves->slave_cnt; i++){
            if(0 == strcmp(data, slaves->_slaves[i].uid))
                slaves->_slaves[i].send_output(response, slaves->_slaves[i].output_map);
        }
        con_info->answer_type = OBJECT;
        con_info->answerstring = response;
        return MHD_YES;
    }

    if(0 != strstr(key, "pdo_post")){
        char tmp[256];
        int x;
        void *value;
        uint8_t idx_slave;
        uint8_t idx_pdo;
        int buffer;
        char *incoming_data[4];
        char *word;
        word = strtok(data, " ");
        incoming_data[0] = word;
        for(int i=1; i<4; i++){
            word = strtok(NULL, " ");
            incoming_data[i] = word;
        }
        ethcercat_slave_collection_t *slaves = get_all_slaves();
        for(int i=0; i<slaves->slave_cnt; i++){
            if(0 == strcmp(incoming_data[0], slaves->_slaves[i].uid)){
                idx_pdo = atoi(incoming_data[1]);
                buffer = atoi(incoming_data[3]);
                value = &buffer;
                slaves->_slaves[i].set_output(idx_pdo, value, incoming_data[2], slaves->_slaves[i].output_map);
                return MHD_YES;
            }
        }
        return MHD_YES;
    }

    if(0 == strcmp(key, "ping")){
        char *response = malloc(MAXANSWERSIZE);
        ethcercat_slave_collection_t *slaves = get_all_slaves();
        uint8_t n_slaves = slaves->slave_cnt;
        if(slaves == NULL){
            printf("No slaves found\n");
            n_slaves = 0;
        }
        char buffer[100];
        if(n_slaves == 0){
            sprintf(response, "{}");
        }
        else{
            sprintf(response, "{");
            for(int i=0; i<n_slaves; i++){
                
                sprintf(buffer, "\"%d\": \"%s\",", i, slaves->_slaves[i].uid);
                strcat(response, buffer);
            }
            response[strlen(response) - 1] = '\0';
            strcat(response, "}");
        }
        con_info->answer_type = OBJECT;
        char *answerstring;
        answerstring = malloc(MAXANSWERSIZE);
        strcpy(answerstring, response);
        con_info->answerstring = answerstring;
        return MHD_YES;
    }

    if(0 == strcmp(key, "jaw_toggle")){
        jaw_toggle(atoi(data));
    }

    if(0 == strcmp(key, "jaw_set_angle")){
        jaw_set_angle(atoi(data));
    }

    con_info->answerstring = NULL;
    return MHD_YES;
}


static int answer_to_connection(void *cls, struct MHD_Connection *connection,
                                const char *url,
                                const char *method, const char *version,
                                const char *upload_data,
                                size_t *upload_data_size, void **con_cls)
{
    if (NULL == *con_cls)
    {
        struct connection_info_struct *con_info;

        con_info = malloc(sizeof(struct connection_info_struct));
        if (NULL == con_info)
            return MHD_NO;
        con_info->answerstring = NULL;
        if (0 == strcmp(method, "POST"))
        {
            con_info->postprocessor = MHD_create_post_processor(connection, POSTBUFFERSIZE, iterate_post, (void *)con_info);

            if (NULL == con_info->postprocessor)
            {
                free(con_info);
                return MHD_NO;
            }
            con_info->connectiontype = POST;
        }
        else
            con_info->connectiontype = GET;

        *con_cls = (void *)con_info;
        return MHD_YES;
    }

    if (0 == strcmp(method, "GET"))
    {
        FILE *fp = fopen("src/index.html", "r");
        int fd = fileno(fp);
        fseek(fp, 0L, SEEK_END);
        size_t size = ftell(fp);
        fseek(fp, 0L, SEEK_SET);
        return send_page(connection, fd, size, 200);
    }

    if (0 == strcmp(method, "POST"))
    {
        struct connection_info_struct *con_info = *con_cls;

        if (*upload_data_size != 0)
        {
            MHD_post_process(con_info->postprocessor, upload_data,
                             *upload_data_size);
            *upload_data_size = 0;

            return MHD_YES;
        }
        else if (NULL != con_info->answerstring){
            switch (con_info->answer_type)
            {
            case PAGE:
                return send_page(connection, con_info->answerfd, con_info->size, 200);
                break;

            case OBJECT:
                return send_object(connection, con_info->answerstring, 200);
                break;

            default:
                break;
            }
        }
            
    }

    return send_page(connection, errorpage, strlen(errorpage), 200);
}

int main()
{
    struct MHD_Daemon *daemon;

    daemon = MHD_start_daemon(MHD_USE_INTERNAL_POLLING_THREAD, PORT, NULL, NULL,
                              &answer_to_connection, NULL,
                              MHD_OPTION_NOTIFY_COMPLETED, &request_completed, NULL,
                              MHD_OPTION_END);
    if (NULL == daemon)
        return 1;
    printf("EtherCat webserver started\n");
    while(!strcmp(current_adapter, "")){

    }  

    osal_thread_create(&thread1, 128000, &ecatcheck, NULL);

    while(1){
        initialize_ethercat(current_adapter);
        retreive_info();
        ethercat_loop();
    }

    getchar();

    MHD_stop_daemon(daemon);
    return 0;
}