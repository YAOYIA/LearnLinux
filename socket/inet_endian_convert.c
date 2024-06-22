#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

// int main(int argc, char const *argv[])
// {
//     struct sockaddr_in server_addr;
//     struct in_addr server_in_addr;
//     in_addr_t server_in_addrt;

//     printf("192.168.6.101的16进制为:0x%X 0x%X 0x%X 0x%X\n",192,168,6,101);

//     inet_aton("192.168.6.101",&server_in_addr);
//     printf("inet_aton的地址为:0x%X\n",server_in_addr.s_addr);

//     server_in_addrt = inet_addr("192.168.6.101");
//     printf("server_in_addrt:0x%X\n",server_in_addrt);

//     inet_pton(AF_INET,"192.168.6.101",&server_addr);
//     printf("inet_pton 后 server_addr.sin_addr 的16进制表示为 0x%X\n", server_addr.sin_addr.s_addr);

//     return 0;
// }


int main(int argc, char const *argv[])
{
    struct sockaddr_in server_addr;
    struct in_addr server_in_addr;
    in_addr_t server_in_addrt;
    memset(&server_addr,0,sizeof(server_addr));
    memset(&server_in_addr,0,sizeof(server_in_addr));
    memset(&server_in_addrt,0,sizeof(server_in_addrt));

    printf("192.168.6.101的16进制为:0x%X 0x%X 0x%X 0x%X\n",192,168,6,101);

    server_in_addrt = inet_addr("192.168.6.101");
    printf("server_in_addrt:0x%X\n",server_in_addrt);

    inet_aton("192.168.6.101",&server_in_addr);
    printf("inet_aton convert: 0x%X\n", server_in_addr.s_addr);

    inet_pton(AF_INET,"192.168.6.101",&server_addr.sin_addr);

    printf("inet_pton 后 server_addr.sin_addr 的16进制表示为 0x%X\n", server_addr.sin_addr.s_addr);
    // 结构体转化为字符串
    printf("通过inet_ntoa打印inet_pton转化后的地址: %s\n", inet_ntoa(server_addr.sin_addr));
    // 打印本地网络地址部分
    printf("local net section: 0x%X\n", inet_lnaof(server_addr.sin_addr));
    // 打印网络号部分
    printf("netword number section: 0x%X\n", inet_netof(server_addr.sin_addr));
    // 使用本地网络地址和网络号可以拼接成in_addr
    server_addr.sin_addr = inet_makeaddr(inet_netof(server_addr.sin_addr), 102);
    // 以网络字节序16进制打印拼接的地址
    printf("inet_makeaddr: 0x%X\n", server_addr.sin_addr.s_addr);
    // 打印拼接的地址
    printf("通过inet_ntoa打印inet_makeaddr拼接后的地址%s\n", inet_ntoa(server_addr.sin_addr));


    return 0;
}

