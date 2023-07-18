#include <AP_gtest.h>

#include <AP_ADSB/AP_ADSB.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(IsValidSquawk, Valid)
{
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(7777));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(777));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(77));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(7));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(0));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(1111));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(111));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(11));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(1));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(0));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(7654));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(321));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(23));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(5));
    EXPECT_TRUE(AP_ADSB::is_valid_squawk(5));
}

TEST(IsValidSquawk, Invalid)
{
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(17777));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(8888));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(888));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(88));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(8));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(9));

    EXPECT_FALSE(AP_ADSB::is_valid_squawk(7778));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(7788));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(7888));
    EXPECT_FALSE(AP_ADSB::is_valid_squawk(8888));
}

AP_GTEST_MAIN()

#pragma GCC diagnostic pop
