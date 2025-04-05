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


# 六、调试多进程服务程序
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

```
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
int main() {
    printf("begin\n");
    if (fork() != 0) {
        printf("parent:pid=%d,ppid=%d\n",getpid(),getppid());
        // sleep(2);
        for (size_t i = 0; i < 10; i++)
        {
            printf("i=%d\n",i);
            sleep(1);
        }
        printf("parent exit\n");
        exit(0);
       
    } else {
        printf("child:pid=%d,ppid=%d\n",getpid(),getppid());
        // sleep(1);
        for (size_t j = 0; j < 10; j++)
        {
            printf("j=%d\n",j);
            sleep(1);
        }
        printf("child exit\n");
        exit(0);
        
    }
}
```


# 七、调试多线程程序
```
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>


int x=0,y=0;

pthread_t pthid1,pyhid2;

void *thread1(void *arg) ;
void *thread2(void *arg) ;


int main() {
    if (pthread_create(&pthid1,NULL,thread1,(void*)0)!=0)
    {
        printf("create thread error\n");
        return -1;
    }

    if (pthread_create(&pyhid2,NULL,thread2,(void*)0)!=0)
    {
        printf("create thread error\n");
        return -1;
    }
    
    printf("main thread exit\n");
    pthread_join(pthid1,NULL);
    printf("222 \n");
    pthread_join(pyhid2,NULL);
    printf("333\n");
    return 0;
    
    
}

void *thread1(void *arg) {
    for (x = 0; x < 100; x++)
    {
        printf("x = %d\n",x);
        sleep(1);
    }
    pthread_exit(NULL);
    
}


void *thread2(void *arg) {
    for (y = 0; y < 100; y++)
    {
        printf("y = %d\n",y);
        sleep(1);
    }
    pthread_exit(NULL);
    
}
```
```
gcc -o -g test test.cc -lpthread
```
在shell中执行
```
查看当前运行的进程 ps aux | grep test
查看当前运行的轻量级进程 ps -aL | grep test
查看主线程和子线程的关系 pstree -p 主线程id
```

```
在gdb中执行
查看有多少线程：info threads
返回的信息中有一个Id的前面有*，表示当前线程
切换线程：thread 线程id
只运行当前线程：set scheduler-locking on
运行全部的线程：set scheduler-locking off
指定某线程执行某gdb命令：thread apply 线程id cmd
全部的线程执行某gdb命令：thread apply all cmd
```

# 八、输出日志
设置断点或者单步跟踪可能会严重干扰多进（线）程之间的竞争状态。导致我们看到一个假象。
一旦我们在某一个线程设置了断点，该线程在断点处挺住了，只剩下另一个线程在跑。这个时候，并发成精已经完全被破坏了，通过调试器看到的只是一个和谐的场景（理想状态）。
调试者的调试行为干扰了程序的运行，导致看到的是一个干扰后的现象。既然断点和单步执行不好用，怎么办呢？
输出log日志，可以避免断点和单步所导致的副作用。



https://www.bilibili.com/video/BV1ei4y1V758?spm_id_from=333.788.player.switch&vd_source=0ca3f400b830cbbd62a5c41090e1d87b&p=6