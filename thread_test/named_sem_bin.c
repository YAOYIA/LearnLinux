#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/mman.h>

int main(int argc, char const *argv[])
{
    char* sem_name = "/name_sem";
    char* shm_name = "/named_sem_shm";

    sem_t* sem =  sem_open(sem_name,O_CREAT,0666,1);
    int fd = shm_open(shm_name,O_CREAT | O_RDWR,0666);
    ftruncate(fd,sizeof(int));

    int* value = mmap(NULL,sizeof(int),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);

    pid_t pid = fork();
    if (pid < 0)
    {
        perror("fork");
    }else if(pid==0){
        sem_wait(sem);
        int tmp = *value + 1;
        sleep(1);
        *value = tmp;
        sem_post(sem);
    }
    // 每个进程都应该在使用完毕后关闭对信号量的连接
    sem_close(sem);
    if (pid > 0)
    {
        waitpid(pid, NULL, 0);
        printf("子进程执行结束，value = %d\n", *value);

        // 有名信号量的取消链接只能执行一次
        sem_unlink(sem_name);
    }

    // 父子进程都解除内存共享对象的映射，并关闭相应的文件描述符
    munmap(value, sizeof(int));
    close(fd);

    // 只有父进程应该释放内存共享对象
    if (pid > 0)
    {
        if (shm_unlink(shm_name) == -1)
         {
            perror("shm_unlink");
        }
    }
    return 0;
}
