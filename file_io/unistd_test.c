#include <unistd.h>//unix标准库
#include <fcntl.h>
#include <stdio.h>
int main(int argc, char const *argv[])
{
    int fd =open("io1.txt",O_RDONLY | O_CREAT,0775);
    if(fd==-1){
        printf("file open faild!!\n");
    }else{
        printf("file open sucess!!\n");
    }
    return 0;
}
