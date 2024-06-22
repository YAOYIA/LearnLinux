#include <mqueue.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
int main(int argc, char const *argv[])
{
    //常见消息队列
    char *mq_name="/father_son_mq";
    struct mq_attr attr;
    //有用参数 表示消息队列的容量
    attr.mq_maxmsg=10;
    attr.mq_msgsize=100;
    //被忽略的消息 在创建消息队列的时候用不到
    attr.mq_flags=0;
    attr.mq_curmsgs=0;
 
    
    mqd_t mqdes = mq_open(mq_name,O_RDWR | O_CREAT,0644,&attr);
    if(mqdes==(mqd_t)-1){
        perror("my_open");
        exit(EXIT_FAILURE);
    }
    //创建父子进程
    pid_t pid = fork();
    if(pid<0){
        perror("fork");
    }else if(pid==0){
        //子进程 等待接收消息队列在中的信息
        char read_buf[100];
        struct timespec time_info;
        for (size_t i = 0; i < 10; i++)
        {
            memset(read_buf,0,100);
            clock_gettime(0,&time_info);
            time_info.tv_sec+=15;
            //接收消息，打印到控制台
            if (mq_timedreceive(mqdes,read_buf,100,NULL,&time_info)==-1){
                perror("mq_timedreceive");
            }
            printf("son has received a information%s\n",read_buf);
        }
        

    }else{
        //父进程 发送消息到消息队列中
        char sned_buff[100];
        struct timespec time_info;
        
        
        for(size_t i=0;i<10;i++){
            //清空buffer
            memset(sned_buff,0,100);
            sprintf(sned_buff,"father the %d send the information",(int)(i+1));
            clock_gettime(0,&time_info);
            time_info.tv_sec+=5;
            if (mq_timedsend(mqdes,sned_buff,strlen(sned_buff),0,&time_info)==-1){
                perror("mq_timedsend");
            }
            printf("father sent a information,sleep 1 s\n");
            sleep(1);

        }
        
    }
    //最红释放消息队列的引用
    close(mqdes);
    //清楚消息队列只需要执行一次
    if(pid>0){
        mq_unlink(mq_name);
    }



    return 0;
}
