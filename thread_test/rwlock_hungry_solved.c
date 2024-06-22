#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

/**
 * 
 * 这里的八个线程基本上是同时创建的，需要进行读写权限的争抢，具体的顺序不知道。读锁可以共同使用，但是写锁只可以一个线程使用
 */
pthread_rwlock_t mutex;
int share_num=0;

void* call_writer(void* arg){
    // pthread_rwlock(mutex);
    pthread_rwlock_wrlock(&mutex);
    int tmp= share_num + 1;
    // share_num+=1;
  
    share_num = tmp;
    pthread_rwlock_unlock(&mutex);
    // pthread_unrwlock(mutex);
    printf("当前是%s,shared_num:%d\n",(char*)arg,share_num);

}
void* call_reader(void* arg){
    pthread_rwlock_rdlock(&mutex);
    sleep(1);
    printf("当前是%s,shared_num:%d\n",(char*)arg,share_num);
    pthread_rwlock_unlock(&mutex);
    

}

int main(int argc, char const *argv[])
{
    //创建读写锁的属性对象
    pthread_rwlockattr_t attr;
    pthread_rwlockattr_init(&attr);
    pthread_rwlockattr_setkind_np(&attr,PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP);
    pthread_rwlock_init(&mutex,NULL);

    pthread_t writer1,writer2,reader1,reader2,reader3,reader4,reader5,reader6;
  

    pthread_create(&writer1,NULL,call_writer,"writer1");
    pthread_create(&writer2,NULL,call_writer,"writer2");
    sleep(3);
    pthread_create(&reader1,NULL,call_reader,"reader1");
    pthread_create(&reader2,NULL,call_reader,"reader1");
    pthread_create(&reader3,NULL,call_reader,"reader1");
    pthread_create(&reader4,NULL,call_reader,"reader1");
    pthread_create(&reader5,NULL,call_reader,"reader1");
    pthread_create(&reader6,NULL,call_reader,"reader1");

    pthread_join(writer1,NULL);
    pthread_join(writer2,NULL);
    pthread_join(reader1,NULL); 
    pthread_join(reader2,NULL);
    pthread_join(reader3,NULL);
    pthread_join(reader4,NULL);
    pthread_join(reader5,NULL);
    pthread_join(reader6,NULL);
    pthread_rwlock_destroy(&mutex);

    return 0;
}
