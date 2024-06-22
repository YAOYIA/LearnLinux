#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <time.h>
#include <pthread.h>

sem_t* full_;
sem_t* empty_;

int share_num=0;

int rand_num(){
    srand(time(NULL));
    return rand();
}

void* producer(void*){
    // sem_wait(empty_);
    for (int i = 0; i < 5; i++)
    {   
        sem_wait(empty_);
        printf("\n======>第 %d 轮数据传输<==========\n",i+1);
        share_num = rand_num();
        printf("producer has sent data\n");
        sem_post(full_);
    }
    // sem_post(full_);

}

void* consumer(void*){
    // sem_wait(full_);
    for (int i = 0; i < 5; i++)
    {
        sem_wait(full_);
        printf("consumer has read data\n");
        printf("the shard_num is %d\n", share_num);
        sleep(1);
        sem_post(empty_);
    }
    // sem_post(empty_);
}

int main(int argc, char const *argv[])
{
    pthread_t producer_id,consumer_id;
    //写到堆中
    full_ = malloc(sizeof(sem_t));
    empty_ = malloc(sizeof(sem_t));
    sem_init(full_,0,0);
    sem_init(empty_,0,1);

    pthread_create(&producer_id,NULL,producer,NULL);
    pthread_create(&consumer_id,NULL,consumer,NULL);

    pthread_join(producer_id,NULL);
    pthread_join(consumer_id,NULL);

    sem_destroy(full_);
    sem_destroy(empty_);
    return 0;
}
