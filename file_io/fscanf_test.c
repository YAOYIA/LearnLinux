#include <stdio.h>

int main(int argc, char const *argv[])
{
    FILE* ioFile = fopen("user.txt","r");

    if(ioFile==NULL){
        printf("FAILED!!!\n");
    }else{
        printf("SUCCESS open file!!\n");
    }
    char name[50];
    int age;
    char wife[50];
    int scanfR = fscanf(ioFile,"%s %d %s",name,&age,wife);
    // if(scanfR!=EOF){
    //     printf("success %d\n",scanfR);
    //     printf("%s在%d的时候爱上了%s\n",name,age,wife);
    // }
    while (scanfR= fscanf(ioFile,"%s %d %s",name,&age,wife)!=EOF)
    {
        printf("%s在%d的时候爱上了%s\n",name,age,wife);
    }
    


    int flag = fclose(ioFile);
    if(flag==EOF){
        printf("close Failed!!!\n");
    }else{
        printf("close success!!\n");
    }
    return 0;
}
