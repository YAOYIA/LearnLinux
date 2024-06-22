#include <stdio.h>


int main(int argc, char const *argv[])
{
    char* filename="io.txt";

    FILE* ioFile = fopen(filename,"w");
    if(ioFile=NULL){
        printf("failed");
    }else{
        printf("success");
    }
    return 0;
}
