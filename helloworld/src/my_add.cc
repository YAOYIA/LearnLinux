#include "my_add.h"


my_add::my_add(int a, int b) : a_(a), b_(b) {}

void my_add::wprint(){
    std::cout<<'a'<<a_<<std::endl;
    std::cout<<'b'<<b_<<std::endl;
}


int my_add::getSum() {
    return a_ + b_;
}