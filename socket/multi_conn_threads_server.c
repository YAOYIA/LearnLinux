#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>


#define handle_error(cmd, result) \
if (result < 0)\
{ \
perror(cmd);\
return -1;\
}

void* read_from_client_then_write(void* arg){
    int sockfd = *(int*)arg;
    char* buffer_r = malloc(1024*sizeof(char));
    char* buffer_w = malloc(1024*sizeof(char));
    if(!buffer_r){
        printf("服务端读缓存创建异常");
        shutdown(sockfd,SHUT_WR);
        close(sockfd);
        perror("malloc servere readbuf");
        return NULL;
    }
    if(!buffer_w){
        printf("服务端写缓存创建异常");
        free(buffer_r);
        shutdown(sockfd,SHUT_WR);
        close(sockfd);
        perror("malloc servere writebuf");
        return NULL;
    }
    ssize_t count=0,send_count=0;
    while ((count = recv(sockfd,buffer_r,1024,0)))
    {
        if(count<0){
            perror("recv");
        }
        printf("receive message from clientfd:%d:%s\n",sockfd,buffer_r);

        strcpy(buffer_w,"received!!\n");
        // fgets(buffer_w,1024,stdin);
        send_count = send(sockfd,buffer_w,1024,0);
        if (send_count < 0){
            perror("send");
        }
        
    }
    printf("释放clientfd:%d资源\n",sockfd);
    shutdown(sockfd,SHUT_WR);
    close(sockfd);
    free(buffer_r);
    free(buffer_w);
    return NULL;
     

}


int main(int argc, char const *argv[])
{
    int sockfd,temp_result;
    
    struct sockaddr_in server_addr,client_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(6666);
    sockfd = socket(AF_INET,SOCK_STREAM,0);
    handle_error("socket",sockfd);

    temp_result = bind(sockfd,(struct sockaddr*)&server_addr,sizeof(server_addr));
    handle_error("bind",temp_result);

    temp_result = listen(sockfd,128);
    handle_error("listen",temp_result);
    socklen_t cli_len=sizeof(client_addr);
    while (1)
    {
        int client_fd;
        client_fd = accept(sockfd,(struct sockaddr*)&client_addr,&cli_len);
        handle_error("accept",client_fd);
        printf("与客户端from%s at port %d 文件描述符文为 %d",inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port),client_fd);
        
        pthread_t pid_read_write;
        pthread_create(&pid_read_write,NULL,read_from_client_then_write,(void*)&client_fd);

        pthread_detach(pid_read_write);
        printf("创建子线程并处理为detach状态\n");
    }
    
    

    
    return 0;
}
