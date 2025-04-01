#pragma once

#include <iostream>
#include <vector>


class my_add
{
private:
    int a_;
    int b_;
public:
    my_add(int a, int b);
    ~my_add() = default;  // 提供默认析构函数
    void wprint();
    int getSum();  // 新增求和函数，方便测试
};

