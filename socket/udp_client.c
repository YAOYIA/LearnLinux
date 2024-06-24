#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>

#define handle_error(cmd, result) \
if (result < 0)\
{ \
perror(cmd);\
return -1;\
}

int main(int argc, char const *argv[])
{
    int sockfd, temp_result, client_fd;
    struct sockaddr_in server_addr;
    char *buf = malloc(1024);

    memset(&server_addr, 0, sizeof(server_addr));

    // 声明IPV4通信协议
    server_addr.sin_family = AF_INET;
    // 我们需要绑定0.0.0.0地址，转换成网络字节序后完成设置
    server_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    // 端口随便用一个，但是不要用特权端口
    server_addr.sin_port = htons(6666);

    // 创建server socket，注意通信类型
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    handle_error("socket", sockfd);

    do
    {
        write(STDOUT_FILENO, "Type something you want to send: ", 34);
        // 从标准输入读取数据
        int buf_len = read(STDIN_FILENO, buf, 1023);

        temp_result = sendto(sockfd, buf, buf_len, 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
        handle_error("sendto", temp_result);

        // 清空缓冲区
        memset(buf, 0, 1024);

        // 尝试接受数据
        temp_result = recvfrom(sockfd, buf, 1024, 0, NULL, NULL);
        handle_error("recvfrom", temp_result);

        // 如果服务端发来的不是EOF
        if (strncmp(buf, "EOF", 3) != 0)
        {
            printf("received msg from %s at port %d: %s", inet_ntoa(server_addr.sin_addr), ntohs(server_addr.sin_port), buf);
        }
    } while (strncmp(buf, "EOF", 3) != 0);

    free(buf);

    return 0;
}