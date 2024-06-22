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
    char write_buf[100];
    struct timespec time_info;
    while (1)
    {
        memset(write_buf,0,100);
        ssize_t read_count = read(STDIN_FILENO,write_buf,100);
        clock_gettime(0,&time_info);
        time_info.tv_sec+=5;
        if(read_count==-1){
            perror("read");
        }else if(read_count==0){
            printf("EOF,exit...\n");
            char eof = EOF;
            //将eof当作一条消息发给消息队列
            if(mq_timedsend(mqdes,&eof,1,0,&time_info)==-1){
                perror("mq_timedsend");
            }
            break;
        }
        if(mq_timedsend(mqdes,write_buf,strlen(write_buf),0,&time_info)==-1){
            perror("mq_timedsend");
        }
        printf("从命令行接收数据，已经发送消息给消息队列\n");
    }

    //关闭文件描述符
    close(mqdes);

    //清楚消息队列
    // mq_unlink(name);

    

    return 0;
}
