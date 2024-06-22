#include <stdio.h>

int main(int argc, char const *argv[])
{
    FILE* ioFile = fopen("io.txt","r");

    if(ioFile==NULL){
        printf("FAILED!!!\n");
    }else{
        printf("SUCCESS open file!!\n");
    }

    char c = fgetc(ioFile);
    printf("%c\n",c);


    int flag = fclose(ioFile);
    if(flag==EOF){
        printf("close Failed!!!\n");
    }else{
        printf("close success!!\n");
    }
    return 0;
}
