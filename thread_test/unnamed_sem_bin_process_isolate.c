#include <sys/mman.h>
#include <stdio.h>
#include <semaphore.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>

int main(int argc, char const *argv[])
{
    char* shm_value_name = "/unnamed_sem_shm_value";
    int value_pd = shm_open(shm_value_name,O_CREAT | O_RDWR,0666);
    ftruncate(value_pd,sizeof(int));

    int* value = mmap(NULL,sizeof(int),PROT_READ | PROT_WRITE,MAP_SHARED,value_pd,0);
    *value = 0;
    sem_t sem;
    //这里将会出现错误，因为信号量必须被定义在共享内存呢，但是此处定义在局部空间欸
    sem_init(&sem,1,1);
    // *value = 0;
    pid_t pid = fork();
    if(pid==0){
        sem_wait(&sem);
        int tmp = *value + 1;
        sleep(1);
        *value = tmp;
        sem_post(&sem);

    }else if(pid>0){
        sem_wait(&sem);
        int tmp = *value + 1;
        sleep(1);
        *value = tmp;
        sem_post(&sem);
        waitpid(pid,NULL,0);
        printf("this is father, child finished\n");
        printf("the final value is %d\n", *value);

    }else{
        perror("fork");
    }

    if(close(value_pd)==-1){
        perror("close value");
    }
 
    if (munmap(value,sizeof(int)))
    {
        perror("munmap value");
    }
    
    if(pid>0){
        if(shm_unlink(shm_value_name)==-1){
            perror("shm_unlink_value");
        }
     
    }


    return 0;
}
