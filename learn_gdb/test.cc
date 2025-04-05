#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>


int x=0,y=0;

pthread_t pthid1,pyhid2;

void *thread1(void *arg) ;
void *thread2(void *arg) ;


int main() {
    if (pthread_create(&pthid1,NULL,thread1,(void*)0)!=0)
    {
        printf("create thread error\n");
        return -1;
    }

    if (pthread_create(&pyhid2,NULL,thread2,(void*)0)!=0)
    {
        printf("create thread error\n");
        return -1;
    }
    
    printf("111\n");
    pthread_join(pthid1,NULL);
    printf("222 \n");
    pthread_join(pyhid2,NULL);
    printf("333\n");
    return 0;
    
    
}

void *thread1(void *arg) {
    for (x = 0; x < 100; x++)
    {
        printf("x = %d\n",x);
        sleep(1);
    }
    pthread_exit(NULL);
    
}


void *thread2(void *arg) {
    for (y = 0; y < 100; y++)
    {
        printf("y = %d\n",y);
        sleep(1);
    }
    pthread_exit(NULL);
    
}