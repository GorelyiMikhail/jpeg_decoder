#include "../utils/init.h"

#include "../utils/test_commons.hpp"

TEST_CASE("jfif (grayscale)", "[jpg]") {
    CheckImage("progressive.jpg");
}

TEST_CASE("jfif (4:4:4)", "[jpg]") {
    CheckImage("progressive_small.jpg");
}

TEST_CASE("jfif/exif (4:2:2)", "[jpg]") {
    CheckImage("progressive-2.jpg", "such decoder");
}
