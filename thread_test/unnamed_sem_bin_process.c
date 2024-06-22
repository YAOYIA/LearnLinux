#include <stdio.h>
#include <semaphore.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/mman.h>

/*
信号量必须置于共享内存内，确保每个进程都可以访问，否则每个进程维护一个信号量，后者没有起到进程间通信的作用
*/
int main(int argc, char const *argv[])
{
    char *shm_sem_name = "/unnamed_sem_shm_sem";
    char *shm_value_name = "/unname_sem_shm_value";

    int sem_fd = shm_open(shm_sem_name,O_CREAT|O_RDWR,0666);
    int value_fd =shm_open(shm_value_name,O_CREAT|O_RDWR,0666);
    ftruncate(sem_fd, sizeof(sem_t));
    ftruncate(value_fd, sizeof(int));

    sem_t* sem = mmap(NULL,sizeof(sem_t),PROT_WRITE | PROT_READ,MAP_SHARED,sem_fd,0);
    int* value = mmap(NULL,sizeof(int),PROT_WRITE | PROT_READ,MAP_SHARED,value_fd,0);
    *value = 0;

    sem_init(sem,1,1);

    pid_t pid = fork();
    
    if(pid==0){
        sem_wait(sem);
        int tmp = *value + 1;
        *value = tmp;
        sem_post(sem);

    }else if(pid>0){
        sem_wait(sem);
        int tmp = *value + 1;
        *value = tmp;
        sem_post(sem);
        waitpid(pid,NULL,0);
        printf("this is father, child finished\n");
        printf("the final value is %d\n", *value);

    }else{
        perror("fork");
    }

    if(close(value_fd)==-1){
        perror("close value");
    }
    if(close(sem_fd)==-1){
        perror("close sem");
    }
    if (munmap(value,sizeof(int)))
    {
        perror("munmap value");
    }
    if (munmap(sem,sizeof(sem_t)))
    {
        perror("munmap sem");
    }
    
    if(pid>0){
        if(shm_unlink(shm_value_name)==-1){
            perror("shm_unlink_value");
        }
        if(shm_unlink(shm_sem_name)==-1){
            perror("shm_unlink_sem");
        }
    }
    return 0;
}

















// int main(int argc, char const *argv[])
// {
//     char *shm_sem_name = "/unnamed_sem_shm_sem";
//     char *shm_value_name = "/unname_sem_shm_value";

//     int sem_fd = shm_open(shm_sem_name,O_CREAT | O_RDWR,0666);
//     int value_fd = shm_open(shm_value_name,O_CREAT | O_RDWR,0666);

//     ftruncate(sem_fd,sizeof(sem_t));
//     ftruncate(value_fd,sizeof(int));

//     int* value = mmap(NULL,sizeof(int),PROT_READ | PROT_WRITE,MAP_SHARED,value_fd,0);
//     sem_t* sem = mmap(NULL,sizeof(sem_t),PROT_READ | PROT_WRITE,MAP_SHARED,sem_fd,0);
//     *value=0;
//     sem_init(sem,1,1);
//     pid_t pid=fork();
//     if(pid==0){
//         sem_wait(sem);
//         int tem = *value+1;
//         sleep(1);
//         *value = tem;
//         sem_post(sem);

//     }else if (pid>0)
//     {
//         sem_wait(sem);
//         int tem = *value+1;
//         sleep(1);
//         *value = tem;
//         sem_post(sem);
//         waitpid(pid,NULL,0);
//         printf("this is father, child finished\n");
//         printf("the final value is %d\n", *value);
//     }else{
//         perror("fork");
//     }

//     if(munmap(sem,sizeof(sem_t))==-1){
//         perror("munmap sem");
//     }
//     if(munmap(value,sizeof(int))==-1){
//         perror("munmap value");
//     }
//     if(pid>0){
//         if (sem_destroy(sem)==-1)
//         {
//             perror("sem destroy");
//         }
        
//         if(shm_unlink(shm_sem_name)==-1){
//             perror("shmunlink sem");
//         }
//         if(shm_unlink(shm_value_name)==-1){
//             perror("shmunlink value");
//         }
//     }
    

//     return 0;
// }
