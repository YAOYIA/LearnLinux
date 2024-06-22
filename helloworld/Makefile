#Makefile内容通常由一下三部分组成

object:=hello.o main.o
#<目标名称>：<前置依赖>
#\t<需要执行的命令>
#放在第一个的是默认目标
#目标是编译出的main文件，依赖hello.o和main.o文件
#编译的命令是 gcc hello.o main.o -o main
main:${object}
	gcc hello.o main.o -o main

#目标是main.o依赖是main.c和hello.h
#编译的命令是gcc -c main.c -o 
# main.o:main.c hello.h
# 	gcc -c main.c 

#目标是hello.c，依赖是hello.c helle.h
#编译的命令是gcc -c hello.c
# hello.o:hello.c hello.h
# 	gcc -c hello.c

clean:
	rm main main.o hello.o 