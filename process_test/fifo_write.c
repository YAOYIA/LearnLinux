#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

int main(int argc, char const *argv[])
{
    char *pipe_name = "/tmp/myinfo";
    if(mkfifo(pipe_name,0664)!=0){
        perror("mkfifo");
        
        exit(EXIT_FAILURE);
    }

    int fd = open(pipe_name,O_WRONLY);

    if(fd<0){
        perror
        ("open");
    }

    char buf[100];
    ssize_t read_num;
    while ((read_num = read(STDIN_FILENO,buf,100))>0)
    {
        write(fd,buf,read_num);
    }
    if(read_num<0){
        perror("read");
        close(fd);
        exit(EXIT_FAILURE);
    }
    printf("senf success!!\n");
    close(fd);
    //释放管道
    unlink(pipe_name);

    return 0;
}
