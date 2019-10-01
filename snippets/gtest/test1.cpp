#include <gtest/gtest.h>

#include <iostream>

void initZero(int &val)
{
    val = 0;
}

TEST(initZero, OneToZero) {
    int i = 1;
    initZero(i);
    EXPECT_EQ(0, i);
}

TEST(initZero, ThreeToZero) {
    int i = 3;
    initZero(i);
    EXPECT_EQ(0, i);
}

// The fixture for testing class Project1. From google test primer.
class MyTest : public ::testing::Test {
protected:
    MyTest() {
        // You can do set-up work for each test here.
    }

    virtual ~MyTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:
    virtual void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
};

// Test case must be called the class above
// Also note: use TEST_F instead of TEST to access the test fixture (from google test primer)
TEST_F(MyTest, SomeTest) {
    
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
