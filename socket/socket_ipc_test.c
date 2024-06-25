#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#define SOCKET_PATH "unix_domain.socket"
#define SERVER_MODE 1
#define CLIENT_MODE 2
#define BUF_LEN 1024

static struct sockaddr_un socket_addr;
static char *buf;

void handle_error(char *err_msg){
   perror(err_msg);
   unlink(SOCKET_PATH);
   exit(-1);
}

void server_mode(int fd){
    int cliend_fd,msg_len;
    static struct sockaddr_un client_addr;
    if(bind(fd,(struct sockaddr*)&socket_addr,sizeof(socket_addr))<0){
        handle_error("bind");
    }
    if(listen(fd,128)<0){
        handle_error("listen");
    }
    socklen_t client_len = sizeof(client_addr);

    if ((cliend_fd = accept(fd,(struct sockaddr*)&client_addr,&client_len))<0)
    {
        handle_error("accept");
    }
    
    write(STDOUT_FILENO,"Connected to client!\n",21);

    do
    {
        memset(buf,0,BUF_LEN);
        msg_len = recv(cliend_fd,buf,BUF_LEN,0);
        printf("received msg %s\n",buf);
        if(strncmp(buf,"EOF",3)!=0){
            strcpy(buf,"ok!\n\0");
        }
        send(cliend_fd,buf,BUF_LEN,0);
        
    } while (strncmp(buf,"EOF",3)!=0);
    unlink(SOCKET_PATH);
    
}


void client_mode(int sockfd)
{
    int msg_len, header_len;
    if (connect(sockfd, (struct sockaddr *)&socket_addr, sizeof(socket_addr)))
    {
        handle_error("connect");
    }

    write(STDOUT_FILENO, "Connected to server!\n", 21);
    strcpy(buf, "Msg received: ");

    // 计算buf中头的长度
    header_len = strlen(buf);

    do
    {
        msg_len = read(STDIN_FILENO, buf + header_len, BUF_LEN - header_len);
        send(sockfd, buf + header_len, msg_len, 0);
        msg_len = recv(sockfd, buf + header_len, BUF_LEN - header_len, 0);
        write(STDOUT_FILENO, buf, msg_len + header_len);
    } while (strncmp(buf + header_len, "EOF", 3) != 0);
}
int main(int argc, char const *argv[])
{
    int fd=0,mode=0;
    if(argc==1 || strncmp(argv[1],"server",6)==0){
        mode = SERVER_MODE;
    }else if(strncmp(argv[1],"client",6)==0){
        mode = CLIENT_MODE;
    }else{
        perror("参数错误\n");
        exit(-1);
    }

    memset(&socket_addr,0,sizeof(struct sockaddr_un));
    buf = malloc(BUF_LEN);

    socket_addr.sun_family =AF_UNIX;
    strcpy(socket_addr.sun_path,SOCKET_PATH);

    if((fd=socket(AF_UNIX,SOCK_STREAM,0))<0){
        perror("socket");
    }

    switch (mode)
    {
    case SERVER_MODE:
        server_mode(fd);
        break;
    case CLIENT_MODE:
        client_mode(fd);
        break;
    }
    if (shutdown(fd, SHUT_RDWR) < 0)
    {
        handle_error("shutdown");
    }
    free(buf);

    exit(0);

    return 0;
}
