#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <syslog.h>

int sockfd;

void zombie_dealer(int sig)
{
    pid_t pid;
    int status;
    char buf[1024];
    memset(buf, 0, 1024);
    // 一个SIGCHLD可能对应多个子进程的退出
    // 使用while循环回收所有退出的子进程，避免僵尸进程的出现
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0)
    {
        if (WIFEXITED(status))
        {
            sprintf(buf, "子进程: %d 以 %d 状态正常退出，已被回收\n", pid, WEXITSTATUS(status));
            syslog(LOG_INFO, "%s", buf);
        }
        else if (WIFSIGNALED(status))
        {
            sprintf(buf, "子进程: %d 被 %d 信号杀死，已被回收\n", pid, WTERMSIG(status));
            syslog(LOG_INFO, "%s", buf);
        }
        else
        {
            sprintf(buf, "子进程: %d 因其它原因退出，已被回收\n", pid);
            syslog(LOG_WARNING, "%s", buf);
        }
    }
}

void sigterm_handler(int sig) {
    syslog(LOG_NOTICE, "服务端接收到守护进程发出的SIGTERM，准备退出...");
    syslog(LOG_NOTICE, "释放sockfd");
    close(sockfd);
    syslog(LOG_NOTICE, "释放syslog连接，服务端进程终止");
    closelog();

    // 退出
    exit(EXIT_SUCCESS);
}

void* read_from_client_then_write(void*arg){
    int client_fd = *(int*)arg;
    char* read_buffer = malloc(1024*sizeof(char));
    char* write_buffer = malloc(1024*sizeof(char));
    ssize_t count=0,send_count=0;
    char log_buf[1024];
    memset(log_buf,0,1024);
    if(!read_buffer){
        sprintf(log_buf,"服务端pid%d,读缓存常见异常，断开连接\n",getgid());
        syslog(LOG_ERR,"%s",log_buf);
        shutdown(client_fd,SHUT_WR);
        close(client_fd);
        return NULL;
    }

    if(!write_buffer){
        sprintf(log_buf,"服务端pid%d,写缓存常见异常，断开连接\n",getgid());
        syslog(LOG_ERR,"%s",log_buf);
        free(read_buffer);
        shutdown(client_fd,SHUT_WR);
        close(client_fd);
        return NULL;
    }


    while (count = recv(client_fd,read_buffer,1024,0))
    {   
        if(count<0){
            syslog(LOG_ERR,"server recv error");
        }
        sprintf(log_buf, "服务端pid: %d: reveive message from client_fd: %d: %s", getpid(), client_fd, read_buffer);
        syslog(LOG_INFO,"%s",log_buf);
        memset(log_buf,0,1024);
        sprintf(write_buffer, "服务端pid: %d: reveived~\n", getpid());
        send_count = send(client_fd,write_buffer,1024,0);
        fputs(read_buffer,stdout);
        
    }
    sprintf(log_buf, "服务端pid: %d: 客户端client_fd: %d请求关闭连接......\n", getpid(), client_fd);
    syslog(LOG_NOTICE, "%s", log_buf);
    sprintf(write_buffer, "服务端pid: %d: receive your shutdown signal\n", getpid());

    send_count = send(client_fd, write_buffer, 1024, 0);

    sprintf(log_buf, "服务端pid: %d: 释放client_fd: %d资源\n", getpid(), client_fd);
    syslog(LOG_NOTICE, "%s", log_buf);
    shutdown(client_fd, SHUT_WR);
    close(client_fd);
    free(read_buffer);
    free(write_buffer);

return NULL;
    


}


int main(int argc, char const *argv[])
{
    int temp_result;

    struct sockaddr_in server_addr, client_addr;

    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    // 声明IPV4通信协议
    server_addr.sin_family = AF_INET;
    // 我们需要绑定0.0.0.0地址，转换成网络字节序后完成设置
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    // 端口随便用一个，但是不要用特权端口
    server_addr.sin_port = htons(6666);

    // 创建server socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    // 绑定地址
    temp_result = bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));

    // 进入监听模式
    temp_result = listen(sockfd, 128);

    socklen_t cliaddr_len = sizeof(client_addr);

    // 注册信号处理函数，处理SIGCHLD信号，避免僵尸进程出现
    signal(SIGCHLD, zombie_dealer);
    // 处理SIGTERM函数，以优雅退出
    signal(SIGTERM, sigterm_handler);

    char log_buf[1024];
    memset(log_buf, 0, 1024);

    // 接受client连接
    while (1)
    {
        int client_fd = accept(sockfd,(struct sockaddr*)&client_addr,&cliaddr_len);

        pid_t pid = fork();
        if(pid>0){
            sprintf(log_buf,"this is father,pid is %d,continue accepting...\n",getpid());
            syslog(LOG_INFO,"%s",log_buf);
            memset(log_buf,0,1024);
            close(client_fd);
        }else if (pid == 0)
        {
            close(sockfd);
            sprintf(log_buf,"客户端form%s at port %d 文件描述符%d建立连接\n",inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port),client_fd);
            syslog(LOG_INFO,"%s",log_buf);
            memset(log_buf,0,1024);

            sprintf(log_buf,"新的服务端pid为:%d\n",getpid());
            syslog(LOG_INFO,"%s",log_buf);
            memset(log_buf,0,1024);
           read_from_client_then_write((void *)&client_fd);

            close(client_fd);
            exit(EXIT_SUCCESS);
        }
        
    }
    
    return 0;
}
