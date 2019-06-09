#include "mbed.h"
#include "fileio.h"
#include "./usartbootloader.h"

int main() {
    printf("starting!\n");
    /*wait(0.5);
    init_io();
    //format_disk();
    wait(0.5);
    print_dir();
    //format_disk();
    //return 0;



    FILE *program = fopen("/fat/program.bin", "r");

    if (program == NULL) {
        printf("replacement program not found!\n");
        return -1;
    }

    printf("program found!\n");
*/
    USARTBootloader slave(D7, D5, D1, D0);

    while(1) ;
}