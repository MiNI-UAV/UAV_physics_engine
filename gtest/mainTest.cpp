#include <gtest/gtest.h>

TEST(TEST_THAT_ALWAYS_PASS, always_pass) {
    ASSERT_EQ(2+2, 4);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}