#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>


/*
这里打印出的是虚拟地址
*/
int main(int argc, char const *argv[])
{
    int val=123;
    pid_t pid = fork();
    if(pid<0){
        perror("fork");
    }else if(pid==0){
        val=321;
        printf("当前的值是%d,它的地址是%p\n",val,&val);
    }else{
        sleep(1);
        printf("当前的值是%d,它的地址是%p\n",val,&val);
    }
    return 0;
}
