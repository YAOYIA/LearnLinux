#include <stdio.h>
#include <stdlib.h>

int main(int argc, char const *argv[])
{
    int souR = system("ping -c 100 www.atguigu.com");
    if(souR!=0){
        perror("system:");
        exit(EXIT_FAILURE);
    }
    return 0;
}
