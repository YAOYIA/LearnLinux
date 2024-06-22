#include <mqueue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>


int main(int argc, char const *argv[])
{
    
    //创建消息队列
    struct mq_attr attr;
    attr.mq_maxmsg=10;
    attr.mq_msgsize=100;
    attr.mq_flags=0;
    attr.mq_curmsgs=0;
    char* name = "/p_c_mq";
    mqd_t mqdes =  mq_open(name,O_RDWR | O_CREAT,0644,&attr);
    if (mqdes==(mqd_t)-1)
    {
        perror("mqdes");
    }

    //不断接收控制台中的数据，发送消息队列
    char read_buf[100];
    struct timespec time_info;
    while (1)
    {
        memset(read_buf,0,100);
     
        clock_gettime(0,&time_info);
        time_info.tv_sec+=5;
        
        if(mq_timedreceive(mqdes,read_buf,100,NULL,&time_info)==-1){
            perror("mq_timedsend");
        }
        if(read_buf[0]==EOF){
            printf("接收到生产者发送的结束信息，准备退出。。\n");
            break;
        }
        printf("已经接收到生产者的消息%s\n",read_buf);
    }

    //关闭文件描述符
    close(mqdes);

    //清楚消息队列
    mq_unlink(name);

    

    return 0;
}
