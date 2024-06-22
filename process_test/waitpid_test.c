#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

int main(int argc, char const *argv[])
{
   int stu;
    pid_t pid = fork();
    if(pid<0){
        perror("fork:");
        exit(EXIT_FAILURE);
    }else if (pid==0)
    {
  
        char* args[]={"/usr/bin/ping","-c","10","www.atguigu.com",NULL};
        char* envs[] = {NULL};
        int flag = execve(args[0],args,envs);//跳转到二楼
        if(flag<-1){
            perror("execev:");
        }
    }else{
      
        printf("i am %d,i invite a new member %d\n",getpid(),pid);
        waitpid(pid,&stu,0);
    }
    
    return 0;
}
