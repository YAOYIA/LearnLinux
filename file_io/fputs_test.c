#include <stdio.h>


int main(int argc, char const *argv[])
{
    char* filename="io.txt";

    FILE* ioFile = fopen(filename,"a+");
    if(ioFile==NULL){
        printf("failed \n");
    }else{
        printf("success \n");
    }

    int wiriteflag = fputs("love dushuaiyao \n",ioFile);

    if(wiriteflag==EOF){
        printf("WRITE FAILED!\n");
    }else{
        printf("WRITE SUCCESS! %s\n",wiriteflag);
    }

    int flag = fclose(ioFile);
    if(flag==0){
        printf("success close! \n");
        
    }else{
        printf("failed close \n");
    }

    
    return 0;
}
