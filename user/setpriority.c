#include "kernel/types.h"
#include "kernel/stat.h"
#include "kernel/param.h"
#include "user/user.h"
#include "kernel/fs.h"
#include "kernel/fcntl.h"

int
main(int argc, char *argv[])
{
    if(argc < 3 || (argv[1][0] < '0' || argv[1][0] > '9')){
        fprintf(2, "Usage: %s command\n", argv[0]);
        exit(1);
    }

    if (set_priority(atoi(argv[1]), atoi(argv[2])) < 0) {
        fprintf(2, "%s: set_priority failed\n", argv[0]);
        exit(1);
    }
    exit(0);
}