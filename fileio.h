#include <mbed.h>
#include <FATFileSystem.h>
#include <SDBlockDevice.h>
#include <MBRBlockDevice.h>
#include <ReadOnlyBlockDevice.h>

#define DEVICE_SPI 1

SDBlockDevice sd(
        D11, //mosi
        D12, //miso
        D13, //clk
        D10 //cs
    );

MBRBlockDevice part1(&sd, 1);
//ReadOnlyBlockDevice robd(&part1);
FATFileSystem fat("fat");

void format_disk() {
    printf("formatting...\n");
    MBRBlockDevice::partition(&sd, 1, 0x83, 0,  1024 * 1024 * 32);
    MBRBlockDevice part(&sd, 1);
    FATFileSystem::format(&part);
    printf("done?\n");
}
bool init_io() {
    MBRBlockDevice::partition(&sd, 1, 0x83, 0,  1024 * 1024 * 32);
    if (sd.init() != 0) {
        printf("sd did not initialize\n");
        return false;
    }
    if (part1.init() != 0) {
        printf("part1 did not initialize\n");
        return false;
    }
    
    printf("fat mount: %d\n", -fat.mount(&part1));
    
    return true;
}

void print_dir() {
    DIR *dir;
    struct dirent *ent;
    printf("dir of sd card: \n");
    if ((dir = opendir ("/fat/")) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            printf ("\t%s\n", ent->d_name);
        }
        closedir (dir);
    } else {
        /* could not open directory */
        printf("error reading directory!\n");
    }
}
bool deinit_io() {
    // not needed for any readonly ops
    //part1.sync();
    //sd.sync();
    return 1;
}