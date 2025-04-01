# 一、安装GDB
```apt -y install gdb```

# 二、调试前准备
用gcc编译源程序是，编译后的可执行文件不会包含源程序代码，如果打算编译后的程序可以被调试，在编译的时候需要添加 **-g** 参数，例如
```
gcc -g -o test test.c
```
在命令提示符下输入 gdb test就可以调试test.c程序了
```
gdb test
```

# 三、基本的调试命令
命令 | 命令缩写 |命令说明
--|--|--|
set args||设置程序的主要参数。设置的方法为 gdb test (gdb)set args argv1 argv2 ...
break|b|设置断点，b 20表示在第20行设置断点，可以设置多个断点
run|r|开始运行程序，程序运行到断点处时会停下来，如果没有遇到断点，程序会一直运行到程序结束为止
next|n|单步执行，执行当前行语句，执行到下一行代码，如果下一行代码是函数调用，则执行到函数调用结束为止,**不会进入函数的内部**
step|s|执行当前行语句，如果当前的语句为函数调用，则会进入函数执行其中的第一条语句。注意了，如果函数是库函数或第三方提供的函数，用**s**也进不去的，因为没有源代码，如果是自定义的函数，只要有源代码就可以进去
print|p|显示变量值，例如 p name表示显示变量name的值
continue|c|继续程序的运行，直到遇到下一个断点
set var name=value||设置变量的值，假设程序有两个变量：int i，char name[21];set var i=10,把i的值设置为10；set var name = "wang"
quit|q|退出gdb


# 四、调试core文件
程序挂掉时，系统缺省不会生成core文件
1. ulimit -a 查看系统参数
2. ulimit -c unlimited 设置core文件大小为无限大
3. 运行程序，生成core文件
4. gdb文件名 core文件名

```
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int bb(int bbb){
    int *p = 0;
    *p = bbb;
}

int aa(int aaa){
    bb(aaa);
}

int main() {
    aa(12);
    return 0;
}
```
当系统报错，出现 **段错误（核心已转存储）**
输入 **ulimit -a**
```
core file size          (blocks, -c) 0
data seg size           (kbytes, -d) unlimited
scheduling priority             (-e) 0
file size               (blocks, -f) unlimited
pending signals                 (-i) 256250
max locked memory       (kbytes, -l) 65536
max memory size         (kbytes, -m) unlimited
open files                      (-n) 1048576
pipe size            (512 bytes, -p) 8
POSIX message queues     (bytes, -q) 819200
real-time priority              (-r) 0
stack size              (kbytes, -s) 8192
cpu time               (seconds, -t) unlimited
max user processes              (-u) 256250
virtual memory          (kbytes, -v) unlimited
file locks                      (-x) unlimited
```
可以看到core file size 为0，所以输入以下命令
```
unlimit -c unlimited
```
```
core file size          (blocks, -c) unlimited
data seg size           (kbytes, -d) unlimited
scheduling priority             (-e) 0
file size               (blocks, -f) unlimited
pending signals                 (-i) 256250
max locked memory       (kbytes, -l) 65536
max memory size         (kbytes, -m) unlimited
open files                      (-n) 1048576
pipe size            (512 bytes, -p) 8
POSIX message queues     (bytes, -q) 819200
real-time priority              (-r) 0
stack size              (kbytes, -s) 8192
cpu time               (seconds, -t) unlimited
max user processes              (-u) 256250
virtual memory          (kbytes, -v) unlimited
file locks                      (-x) unlimited
```
此时的core file size 为unlimited，可以生成core文件了。
但是此时发现，在工作的目录下并没有core文件
**临时修改**
```
切换到root
sudo -s
echo "core.%e.%p.%t" > /proc/sys/kernel/core_pattern
echo "/home/zsf/5t/zsf/wkkkkky/LearnLinux/learn_gdb/core.%e.%p.%t" > /proc/sys/kernel/core_pattern

exit 退出root
./test  可以生成core文件了
```
**永久修改**
```
修改/etc/system.conf文件
kernel.core_pattern=/home/zsf/5t/zsf/wkkkkky/LearnLinux/learn_gdb/core.%e.%p.%t
```


# 五、正在运行中的程序调试
```
#include <unistd.h>
int bb(int bbb){
    int i = 0;
    for (i = 0; i < 1000; i++)
    {
        sleep(1);
        printf("i=%d\n",i);
    }
}

int aa(int aaa){
    bb(aaa);
}

int main() {
    aa(12);
    return 0;
}
```
1. 编译程序
2. 终端运行程序
3. 再开一个终端使用gdb，对该正在运行的程序进行调试.但是此时进入gdb的调试有点不同，要获取当前运行的程序的进程id，使用进程号进入gdb
   ```
   ps -ef | grep test
   gdb test -p pid
   ```
4. 此时发现，运行的程序也中止了运行，如果退出了gdb，程序会继续运行
   
但是，需要一些额外的设置


# 调试多进程服务程序
调试父进程
```
set follow-fork-mode parent (缺省)
```
调试子进程
```
set follow-fork-mode child
```
设置调试模式
```
set detach-on-fork [on|off] 缺省是on
表示调试当前进程的时候，其它的进程继续执行，如果使用off，调式当前前程的时候，其它的进程被gdb挂起
```
查看调式进程
```
info inferiors
```
切换当前调式进程
```
inferior 进程id
```