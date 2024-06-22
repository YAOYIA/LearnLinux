#include <stdio.h>


int main(int argc, char const *argv[])
{
    char* filename="io.txt";

    FILE* ioFile = fopen(filename,"r");
    if(ioFile=NULL){
        printf("failed \n");
    }else{
        printf("success \n");
    }
    

    int flag = fclose(ioFile);
    if(flag==0){
        printf("success close! \n");
        
    }else{
        printf("failed close \n");
    }

    return 0;
}
