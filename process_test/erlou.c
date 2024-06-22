#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


int main(int argc, char const *argv[])
{
    if(argc<2){
        printf("you can't in erlou!!\n");
    }
 
    printf("i am %s,number %d,i am in erlou!!,my fater is %d\n",argv[1],getpid(),getppid());
    return 0;
}
