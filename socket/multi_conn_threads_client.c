#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>


#define handle_error(cmd, result) \
if (result < 0)\
{ \
perror(cmd);\
return -1;\
}

void* read_from_server(void* arg){
    int fd = *(int*)arg;
    char* buffer = malloc(1024*sizeof(char));
    ssize_t count=0;
    while ((count=recv(fd,buffer,1024,0)))
    {
        if(count<0){
            perror("recv");
        }
        // printf("已经收到服务端%d的消息%s\n",fd,buffer);
        fputs(buffer,stdout);
    }
    printf("服务端请求关闭\n");
    // shutdown(fd,SHUT_RD);
    // close(fd);
    free(buffer);
    return NULL;
}
void* write_to_server(void* arg){
    int fd = *(int*)arg;
    char* buffer = malloc(1024*sizeof(char));
    ssize_t send_count=0;
    while (fgets(buffer, 1024, stdin) != NULL)
    {
        send(fd, buffer, 1024, 0);
        if (send_count < 0)
        {
            perror("send");
        }
    }
    
    printf("接收到命令行的终止信号，不再写入，关闭连接......\n");
    shutdown(fd, SHUT_WR);
    free(buffer);
    return NULL;

}
int main(int argc, char const *argv[])
{
    int sockfd,temp_result;
    pthread_t pid_read,pid_write;
    struct sockaddr_in server_addr,client_addr;
    server_addr.sin_family =AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("172.21.242.84");
    server_addr.sin_port =htons(6666);

    sockfd = socket(AF_INET,SOCK_STREAM,0);
    handle_error("socket",sockfd);

    temp_result = connect(sockfd,(struct sockaddr*)&server_addr,sizeof(server_addr));
    handle_error("connect",temp_result);

    pthread_create(&pid_read,NULL,read_from_server,&sockfd);
    pthread_create(&pid_write,NULL,write_to_server,&sockfd);

    pthread_join(pid_read,NULL);
    pthread_join(pid_write,NULL);
    printf("关闭资源\n");
    close(sockfd);

    return 0;
}
