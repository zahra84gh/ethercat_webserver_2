
#define PORT 8888
#define MAXNAMESIZE 40
#define MAXANSWERSIZE 350
#define GET 0
#define POST 1

#define POSTBUFFERSIZE 512
#define MAXCLIENTS 2

enum answer_types{
    PAGE,
    OBJECT
};

struct connection_info_struct
{
    int connectiontype;
    int answer_type;
    int answerfd;
    size_t size;
    char *answerstring;
    struct MHD_PostProcessor *postprocessor;
};