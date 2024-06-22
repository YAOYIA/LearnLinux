#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>


int main(int argc, char const *argv[])
{
    char* name = "old";
    printf("%s %d still learn in one,my partent is%d!!!\n",name,getpid(),getppid());

    pid_t pid = fork();
    if(pid<0){
        perror("fork:");
        exit(EXIT_FAILURE);
    }else if (pid==0)
    {
        char* newname = "dsy";
        char* args[]={"/home/zsf/webserver/learn_linux/process_test/erlou",newname,NULL};
        char* envs[] = {NULL};
        execve(args[0],args,envs);//跳转到二楼
    }else{
        // sleep(1);
        printf("i am %d,i invite a new member %d,my father is %d\n",getpid(),pid,getppid());
        fgetc(stdin);
    }
    
    return 0;
}
