#include <gtest/gtest.h>
#include "my_add.h"



TEST(MyAddTest, ConstructorTest) {
    my_add obg(3,4);
    EXPECT_ANY_THROW(my_add(3,4));
}

TEST(MyAddTest, GetSumTest) {
    my_add obg(3,4);
    EXPECT_EQ(obg.getSum(), 7);
}

TEST(MyAddTest, WPrintTest){
    my_add obg(3,4);
    EXPECT_NO_THROW(obg.wprint());
}




