#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>



#define handle_error(cmd, result) \
if (result < 0)\
{ \
perror(cmd);\
return -1;\
}


void* read_from_client(void* argv){
    //接收recv接收客户端的发送的数据，打印到控制台
    int fd = *(int*)argv;
    char* read_buf = malloc(sizeof(char)*1024);
    ssize_t count=0;

    if (!read_buf)
    {
        perror("malloc server read_buf");
        return NULL;
    }
    
    while (count = recv(fd,read_buf,1024,0))
    {
        if (count<0)
        {
            perror("recv");
        }
        fputs(read_buf,stdout);
    }
    printf("客户端请求关闭连接....\n");
    free(read_buf);
    return NULL;
    

}
void* write_to_client(void* argv){
    //接收控制台的输入的信息，写出去
    int fd = *(int*)argv;
    char* write_buf = malloc(sizeof(char)*1024);
    ssize_t count=0;

    if (!write_buf)
    {
        perror("malloc server write_buf");
        return NULL;
    }
    
    while (fgets(write_buf,1024,stdin)!=NULL)
    {
        count = send(fd,write_buf,1024,0);
        if (count<0)
        {
            perror("recv");
        }
       
    }
    printf("接收到控制台的关闭请求 不再写入....\n");
    shutdown(fd,SHUT_WR);
    free(write_buf);
    return NULL;

}

int main(int argc, char const *argv[])
{
    struct sockaddr_in server_addr,client_addr;
    int sockfd,temp_result,client_fd;
    pthread_t pid_read,pid_write;
    memset(&server_addr,0,sizeof(server_addr));
    memset(&client_addr,0,sizeof(client_addr));

    //声明IPV4协议
    server_addr.sin_family = AF_INET;
    //需要绑定0.0.0.0地址，转换成网络字节序后完成设置
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);;
    //端口号随便用一个，但是不要使用特权端口
    server_addr.sin_port = htons(6666);

    //创建socket
    sockfd = socket(AF_INET,SOCK_STREAM,0);
    handle_error("socket",sockfd);

    //绑定地址
    temp_result = bind(sockfd,(struct sockaddr *)&server_addr,sizeof(server_addr));
    handle_error("bind",temp_result);

    //进入监听模式
    temp_result = listen(sockfd,128);
    handle_error("listen",temp_result);

    //接收第一个client链接
    socklen_t cliaddr_len = sizeof(client_addr);

    client_fd = accept(sockfd,(struct sockaddr *)&client_addr,&cliaddr_len);
    handle_error("accept",client_fd);

    printf("与客户端 from %s at PORT %d 文件描述符 %d 建立连接\n",inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), client_fd);

    pthread_create(&pid_read,NULL,read_from_client,(void*)&client_fd);
    pthread_create(&pid_write,NULL,write_to_client,(void*)&client_fd);

    pthread_join(pid_read,NULL);
    pthread_join(pid_write,NULL);

    printf("释放资源");
    close(sockfd);
    close(client_fd);
    return 0;
}
