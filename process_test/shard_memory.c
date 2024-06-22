#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <string.h>


int main(int argc, char const *argv[])
{
    
    //创建共享内存对象
    char shm_name[100]={0};
    sprintf(shm_name,"/letter%d",getpid());

    int fd;
    fd = shm_open(shm_name,O_RDWR | O_CREAT,0644);
    if(fd<0){
        perror("shm_open");
        exit(EXIT_FAILURE);
    }

    //2设置共享内存大小
    ftruncate(fd,1024);

    //3内存映射
    char* share;
    share = mmap(NULL,1024,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    //映射完成 关闭fd链接 不是释放
    close(fd);

    //4使用共享内存，完成进程间通信
    pid_t pid = fork();
    if(pid<0){
        perror("fork");
        exit(EXIT_FAILURE);
    }else if(pid==0){
        strcpy(share,"i love you dsy!!\n");
        printf("child %d have sent information to share!!\n",getpid());
    }else{
        waitpid(pid,NULL,0);
        printf("father %d has receive child %d information\n %s ",getpid(),pid,share);
        //5释放映射区
        int re = munmap(share,1024);
        if(re == -1){
            perror("munmap");
            exit(EXIT_FAILURE);
        }
    }
    
    
    //释放共享内存对象，这里子进程和父进程都要进行释放
    shm_unlink(shm_name);

    return 0;
}
