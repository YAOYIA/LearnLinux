#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define BUF_LEN 1024

char* buf;
//读取控制台信息
//读线程的逻辑
void*  input_thread(void* argv){
    int i=0;
    while (1)
    {
        char c = fgetc(stdin);
        if (c && c!='\n'){
            buf[i++]=c;
            if(i>=BUF_LEN){
                i=0;
            }
        }
    }
    
}


//缓存信息写出到控制台
//写线程
void*  output_thread(void* argv){
    int i=0;
    while (1)
    {
        if(buf[i]){
            fputc(buf[i],stdout);
            fputc('\n',stdout);
            buf[i++]=0;
            //读取数据到最大下标
            if(i>=1024){
                i=0;
            }
        }else{
            sleep(1);
        }
        
    }
    
}
int main(int argc, char const *argv[])
{
    //声明线程id
    pthread_t pid_input;
    pthread_t pid_output;
    //创建线程
    buf = malloc(BUF_LEN);
    pthread_create(&pid_input,NULL,input_thread,NULL);
    pthread_create(&pid_output,NULL,output_thread,NULL);

    //主线程等待读写线程结束
    pthread_join(pid_input,NULL);
    pthread_join(pid_output,NULL);

    free(buf);
    return 0; 
}
