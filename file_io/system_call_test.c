#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>


int main(int argc, char const *argv[])
{
    int fd =  open("io11.txt",O_RDONLY);
    
    char buffer[1024];
    ssize_t b_size;

    if(fd==-1){
         perror("open");
    }
    while ((b_size=read(fd,buffer,sizeof(buffer))>0))
    {
        write(STDERR_FILENO,buffer,b_size);
    }
    if(b_size==-1){
        // printf("failed!!!\n");
        perror("open");
        close(fd);
        exit(EXIT_FAILURE);
    }
    close(fd);
    // exit(EXIT_SUCCESS);
    
    return 0;
}
