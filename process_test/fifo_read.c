#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>


int main(int argc, char const *argv[])
{
    char *pipe_name = "/tmp/myinfo";
    // if(mkfifo(pipe_name,0655)!=0){
    //     perror("mkfifo");
    //     exit(EXIT_FAILURE);
    // }

    int fd = open(pipe_name,O_RDONLY);

    char buf[100];
    ssize_t read_num;
    while ((read_num = read(fd,buf,100))>0)
    {
        write(STDOUT_FILENO,buf,read_num);
    }
    if(read_num<0){
        perror("read");
        close(fd);
        exit(EXIT_FAILURE);
    }
    printf("read success!!\n");
    close(fd);




    return 0;
}
