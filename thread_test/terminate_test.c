#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Result{
    char* p;
    int len;
} Result;

void* red_call(void* argv){
    //初始化result结构体
    Result *result = malloc(sizeof(Result));
    //解析传递的参数
    char code = *((char*)argv);
    //声明村昂读取消息的字符串
    char *ans=malloc(101);

    while (1)
    {
        fgets(ans,100,stdin);
       if (ans[0]=code)
       {
        free(ans);
        printf("红玫瑰离开了\n");
        char *redans=strdup("红玫瑰独自去了纽约\n");
        result->p = redans;
        result->len = strlen(redans);

        pthread_exit((void*) result);  
        
       }else{
        printf("红玫瑰还在等你！\n");
       }
        
    }

}
void* white_call(void* argv){
    //初始化result结构体
    Result *result = malloc(sizeof(Result));
    //解析传递的参数
    char code = *((char*)argv);
    //声明村昂读取消息的字符串
    char *ans=malloc(101);

    while (1)
    {
        fgets(ans,100,stdin);
       if (ans[0]=code)
       {
        free(ans);
        printf("白玫瑰离开了\n");
        char *redans=strdup("白玫瑰独自去了纽约\n");
        result->p = redans;
        result->len = strlen(redans);

        pthread_exit((void*) result);  
        
       }else{
        printf("白玫瑰还在等你！\n");
       }
        
    }
    
}
int main(int argc, char const *argv[])
{
    pthread_t pid_red;
    pthread_t pid_while;

    char red_code = 'r';
    char white_code = 'w';

    pthread_create(&pid_red,NULL,&red_call,&red_code);
    pthread_create(&pid_while,NULL,&white_call,&white_code);

    Result* red_Results=NULL;
    Result* white_Results=NULL;


    pthread_join(pid_red,(void**) &red_Results);
    pthread_join(pid_while,(void**) &white_Results);
    return 0;
}
