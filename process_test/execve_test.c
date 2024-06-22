#include <stdio.h>
#include <unistd.h>

int main(int argc, char const *argv[])
{
    char* name = "banzhang";

    printf("i am %s,number%d,i am one!\n",name,getpid());

    char* args[] ={"/home/zsf/webserver/learn_linux/process_test/erlou",name,NULL};
    char* evns[] = {NULL};
    int re = execve(args[0],args,evns);
    if(re ==-1){
        printf("you don'y have chance to erlou!!\n");
        return 1;
    }
    
    return 0;
}
