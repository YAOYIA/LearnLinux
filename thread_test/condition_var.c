#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#define BUFFER_SIZE 5
char buffer[BUFFER_SIZE];
int count=0;
static pthread_mutex_t mutex= PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

void* pro(void* argv){
    int item=1;
    pthread_mutex_lock(&mutex);
    while (1)
    {
        if (count==BUFFER_SIZE)
        {
            pthread_cond_wait(&cond,&mutex);
        }
        buffer[count++]=item++;
        printf("白月光发送了一个幸运数字%d\n",buffer[count-1]);
        pthread_cond_signal(&cond);
        
    }
    pthread_mutex_unlock(&mutex);
    
}
void* con(void* argv){
    pthread_mutex_lock(&mutex);
    while (1)
    {
        if (count==0)
        {
            pthread_cond_wait(&cond,&mutex);
        }
      
        printf("白月光接收了一个幸运数字%d\n",buffer[--count]);
        pthread_cond_signal(&cond);
        
    }
    pthread_mutex_unlock(&mutex);

}

int main(int argc, char const *argv[])
{

    pthread_mutex_init(&mutex,NULL);
    pthread_t p_pid,c_pid;
    pthread_create(&p_pid,NULL,pro,NULL);
    pthread_create(&c_pid,NULL,con,NULL);

    pthread_join(p_pid,NULL);
    pthread_join(c_pid,NULL);


    return 0;
}



