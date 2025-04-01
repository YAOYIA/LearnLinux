#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
int main() {
    printf("begin\n");
    if (fork() == 0) {
        printf("child:pid=%d,ppid=%d\n",getpid(),getppid());
        // sleep(1);
        for (size_t j = 0; j < 10; j++)
        {
            printf("j=%d",j);
            sleep(1);
        }
        printf("child exit\n");
        exit(0);
    } else {
        printf("parent:pid=%d,ppid=%d\n",getpid(),getppid());
        // sleep(2);
        for (size_t i = 0; i < 10; i++)
        {
            printf("i=%d",i);
            sleep(1);
        }
        printf("parent exit\n");
        exit(0);
    }
}