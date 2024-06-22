
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>


int main(int argc, char const *argv[])
{
    int num=0;
    char *shm_named = "/unnamed_sem_shm_value";
    int shm_fd = shm_open(shm_named,O_CREAT | O_RDWR,0666);
    ftruncate(shm_fd,sizeof(int));
    int* value = mmap(NULL,sizeof(int),PROT_READ | PROT_WRITE,MAP_SHARED,shm_fd,0);
    *value =0;
    pid_t pid = fork();
    if(pid==0){
        int tmp = *value+1;
        sleep(1);
        *value = tmp;

    }else if (pid>0)
    {
        int tmp = *value+1;
        sleep(1);
        *value = tmp;
        waitpid(pid,NULL,0);
        printf("this is father,child has finished\n");
        printf("the final value is %d\n",*value);
        
    }else{
        perror("fork");
    }

    if(munmap(value,sizeof(int))==-1){
        perror("munmap value");
    }

    if(close(shm_fd)==-1){
        perror("close");

    }

    if (pid>0)
    {
        shm_unlink(shm_named);
    }
    
    
    return 0;
}
