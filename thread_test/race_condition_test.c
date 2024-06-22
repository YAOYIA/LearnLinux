#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define THREAD_COUNT 20000

static pthread_mutex_t counter_mutex = PTHREAD_MUTEX_INITIALIZER;

void* add_call(void*arg){
    int *num = arg;
    pthread_mutex_lock(&counter_mutex);
    (*num)+=1;
    pthread_mutex_unlock(&counter_mutex);
    return (void*)0;

}

int main(int argc, char const *argv[])
{
    pthread_t pid[THREAD_COUNT];
    int num=0;
    for (size_t i = 0; i < THREAD_COUNT; i++)
    {
        pthread_create(pid+i,NULL,add_call,&num);
    }

    for (size_t i = 0; i < THREAD_COUNT; i++)
    {
        pthread_join(pid[i],NULL);
    }
    printf("累积的结果为：%d\n",num);
    
    
    return 0;
}


