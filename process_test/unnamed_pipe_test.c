#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>


int main(int argc, char const *argv[])
{
   


    int pipefd[2];
    if (pipe(pipefd)==-1)
    {
        perror("pipe");
        exit(EXIT_FAILURE);
    }


    pid_t cpid = fork();

    if(cpid==-1){
        perror("fork");
    }else if (cpid == 0)
    {
        close(pipefd[1]);
        char buffer;
        while (read(pipefd[0],&buffer,1))
        {
            write(STDOUT_FILENO,&buffer,1);
        }
        
        write(STDOUT_FILENO,"\n",1);
        close(pipefd[0]);
        _exit(EXIT_SUCCESS);
        
    }else{
        close(pipefd[0]);
        printf("father tranlate information!!\n");
        write(pipefd[1],argv[1],strlen(argv[1]));
        close(pipefd[1]);
        waitpid(cpid,NULL,0);
        exit(EXIT_SUCCESS);
    }
    

    
    return 0;
}


// int main(int argc, char const *argv[])
// {

//     pid_t cpid;
//     int pipefd[2];
//     if(argc!=2){
//         fprintf(stderr,"%s:plase input the tranlate information\n",argv[0]);
//         exit(EXIT_FAILURE);
//     }
//     //创建管道
//     if(pipe(pipefd)){
//         perror("create pipe failed!!\n");
//     }
//     cpid = fork();
//     if(cpid==-1){
//         perror("fork");
//     }
//     if (cpid==0)
//     {
//         close(pipefd[1]);
//         char buffer;
//         while (read(pipefd[0],&buffer,1))
//         {
//             write(STDOUT_FILENO,&buffer,1);
//         }
//         write(STDOUT_FILENO,"\n",1);
//         close(pipefd[0]);
//         _exit(EXIT_SUCCESS);
        
        
//     }else{
//         close(pipefd[0]);
//         printf("father tranlate information!!\n");
//         write(pipefd[1],argv[1],strlen(argv[1]));
//         close(pipefd[1]);
//         waitpid(cpid,NULL,0);
//         exit(EXIT_SUCCESS);
//     }
    


//     return 0;
// }
