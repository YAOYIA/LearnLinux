#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

// 192.168.10.150 IP 地址的16进制表示
#define INADDR_LOCAL 0xC0A80A96

#define handle_error(cmd, result) \
if (result < 0) \
{ \
perror(cmd);    \
return -1;  \
}

void *read_from_server(void *argv)
{
    int sockfd = *(int *)argv;
    char *read_buf = NULL;
    ssize_t count = 0;

    read_buf = malloc(sizeof(char) * 1024);
    if (!read_buf)
    {
        perror("malloc client read_buf");
        return NULL;
    }

    while (count = recv(sockfd, read_buf, 1024, 0))
    {
        if (count < 0)
        {
        perror("recv");
        }
        fputs(read_buf, stdout);
    }

    printf("收到服务端的终止信号......\n");
    free(read_buf);

    return NULL;
}

void *write_to_server(void *argv)
{
    int sockfd = *(int *)argv;
    char *write_buf = NULL;
    ssize_t send_count = 0;

    write_buf = malloc(sizeof(char) * 1024);

    if (!write_buf)
    {
        printf("写缓存分配失败，断开连接\n");
        shutdown(sockfd, SHUT_WR);
        perror("malloc client write_buf");

        return NULL;
    }

    while (fgets(write_buf, 1024, stdin) != NULL)
    {
        send_count = send(sockfd, write_buf, 1024, 0);
        if (send_count < 0)
        {
        perror("send");
        }
    }

    printf("接收到命令行的终止信号，不再写入，关闭连接......\n");
    shutdown(sockfd, SHUT_WR);
    free(write_buf);

    return NULL;
}

int main(int argc, char const *argv[])
{
    int sockfd, temp_result;
    pthread_t pid_read, pid_write;

    struct sockaddr_in server_addr, client_addr;

    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    server_addr.sin_family = AF_INET;
    // 连接本机 127.0.0.1
    server_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    // 连接端口 6666
    server_addr.sin_port = htons(6666);

    client_addr.sin_family = AF_INET;
    // 连接本机 172.21.242.84
    client_addr.sin_addr.s_addr = inet_addr("172.21.242.84");
    // 连接端口 8888
    client_addr.sin_port = htons(8888);

    // 创建socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    handle_error("socket", sockfd);

    temp_result = bind(sockfd, (struct sockaddr *)&client_addr, sizeof(client_addr));
    handle_error("bind", temp_result);

    // 连接server
    temp_result = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    handle_error("connect", temp_result);

    // 启动一个子线程，用来读取服务端数据，并打印到 stdout
    pthread_create(&pid_read, NULL, read_from_server, (void *)&sockfd);
    // 启动一个子线程，用来从命令行读取数据并发送到服务端
    pthread_create(&pid_write, NULL, write_to_server, (void *)&sockfd);

    // 阻塞主线程
    pthread_join(pid_read, NULL);
    pthread_join(pid_write, NULL);

    printf("关闭资源\n");
    close(sockfd);

    return 0;
}