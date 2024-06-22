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

    sem_t* sem = mmap(NULL,sizeof(sem_t),PROT_READ | PROT_WRITE,MAP_SHARED,value_pd,0);

   
    sem_init(sem,1,0);
    // *value = 0;
    pid_t pid = fork();
    if(pid==0){
        sleep(1);
        printf("this is son\n");
        sem_post(sem);
    }else if(pid>0){
        sem_wait(sem);
        sleep(1);
        waitpid(pid,NULL,0);
        printf("this is father\n");


    }else{
        perror("fork");
    }

    if(close(value_pd)==-1){
        perror("close value");
    }
 
    if (munmap(sem,sizeof(sem_t)))
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
