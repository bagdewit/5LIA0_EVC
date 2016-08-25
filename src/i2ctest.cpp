#include <unistd.h>             //Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <sys/ioctl.h>          //Needed for I2C port
#include <linux/i2c-dev.h>      //Needed for I2C port
#include "stdio.h"
#include "stdlib.h"

int main(int argc, char *argv[]) {
    int file_i2c;
    int length;
    unsigned char *buffer;
    buffer = (unsigned char*) malloc(60);

    
    //----- OPEN THE I2C BUS -----
    char *filename = (char*)"/dev/i2c-1";
    if ((file_i2c = open(filename, O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check errno to see what went wrong
        printf("Failed to open the i2c bus");
        return 1;
    }
    
    int addr = 0x08;          //<<<<<The I2C address of the slave
    if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        //ERROR HANDLING; you can check errno to see what went wrong
        return 1;
    }

    //----- WRITE BYTES -----
    if(argc!=3) {
        buffer[0] =  128;  buffer[1] = 'F';  buffer[2] = 'L';  buffer[3] = 'U';
        buffer[4] =  '+';  buffer[5] = '1';  buffer[6] = '5';  buffer[7] = '0';
        buffer[8] =  '+';  buffer[9] = '0'; buffer[10] = '0'; buffer[11] = '0';
        buffer[12] = '0'; buffer[13] = '5'; buffer[14] = '0'; buffer[15] = '0';
        length = 4;         //<<< Number of bytes to write
    } else {
        buffer = (unsigned char*) argv[1];
        length = (argv[2][0]-'0')*10+(argv[2][1]-'0');
    }
    if (write(file_i2c, buffer, length) != length)      //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
    }
    usleep(500); //the arduino needs a few clockticks to supply the data

    //----- READ BYTES -----
    length = 2;         //<<< Number of bytes to read
    if (read(file_i2c, buffer, length) != length)       //read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        //ERROR HANDLING: i2c transaction failed
        printf("Failed to read from the i2c bus.\n");
    }
    else
    {
        printf("Data read: ");
        for(int i=0; i<length; i++)
            printf("%d ", buffer[i]);
        printf("\n");
        printf("Data read: ");
        for(int i=0; i<length; i++)
            printf("%c ", buffer[i]);
        printf("\n");
    }

    return 0;
}
