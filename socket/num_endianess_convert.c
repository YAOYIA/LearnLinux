#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>



int main(int argc, char const *argv[])
{
    unsigned short local_num=0x1f,net_num=0;
    
    net_num = htons(local_num);
    printf("输入的16进制为%hX\n,输出的数为%hX\n",local_num,net_num); 
    return 0;
}
