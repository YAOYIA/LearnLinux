#include <stdio.h>
#include <unistd.h>

int main(int argc, char const *argv[])
{
    FILE* file = fopen("testfile.txt","w");
    setvbuf(file,NULL,_IOFBF,0);
    fprintf(file,"hello");
    fflush(file);

    

    char* arg[] ={"usr/bin/ping","-c","1","www.baidu.com",NULL};
    char* env[]={NULL};
    execve("/usr/bin/ping",arg,env);


    return 0;
}
