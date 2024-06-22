
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char const *argv[])
{
    printf("teacher teach old number %d\n",getpid());

    pid_t pid = fork();
    printf("pid: %d\n",pid);
    //fork之后，所有的代码都是在父子进程中各自执行一次
    //fork之前只在父进程运行，但是在fork之后，父子进程都要运行
    if(pid<0){
        perror("pid failed:");
    }else if (pid==0)
    {
        printf("new number %d is invited by old number %d\n",getpid(),getppid());
    }else{
        //执行单独的父进程代码的
        printf("old number %d invite new number %d\n",getpid(),pid);
    }
    
    return 0;
}

