#include <stdio.h>
#include <stdlib.h>
#include <semaphore.h>
#include <pthread.h>

int num=0;
sem_t unname;


void* ops(void* arg){
    sem_wait(&unname);
    int tem = num+1;
    num=tem;
    // num+=1;
    sem_post(&unname);
    return (void*)0;
}

int main(int argc, char const *argv[])
{
    sem_init(&unname,0,1);
    pthread_t pid[10000];
    for (int i = 0; i < 10000; i++)
    {
        pthread_create(pid+i,NULL,ops,NULL);
    }
    
    for (int i = 0; i < 10000; i++)
    {
        pthread_join(pid[i],NULL);
    }
    printf("shared num is %d\n",num);
    sem_destroy(&unname);
    return 0;
}
