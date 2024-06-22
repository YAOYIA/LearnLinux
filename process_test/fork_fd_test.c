#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>

int main(int argc, char const *argv[])
{
    int fd =  open("io.txt",O_CREAT | O_WRONLY |O_APPEND,0644);
    if(fd<0){
        perror("open:");
    }

    char buffer[1024];
    ssize_t write_byts;
    pid_t pid = fork();
    if(pid<0){
        perror("fork");
        exit(EXIT_FAILURE);
    }else if (pid==0)
    {   
        strcpy(buffer,"this is child data!!!\n");
      
    }else{
        sleep(1);
        strcpy(buffer,"this is father data!!!\n");
    }
    write_byts = write(fd,buffer,strlen(buffer));

    
    if(write_byts<0){
        perror("write:");
        close(fd);
        exit(EXIT_FAILURE);
    }
    printf("finish written!!\n");
    close(fd);

    if (pid==0)
    {   
        printf("child has finished!!!\n");
      
    }else{
        printf("father has finished!!!\n");
    }
    

    return 0;
}
