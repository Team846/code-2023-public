#include "transitionLib846/named.h"

#include <gtest/gtest.h>

// Test named join.
TEST(NamedTest, Join) {
  EXPECT_EQ(transitionLib846::Named::Join({"a"}), "a");
  EXPECT_EQ(transitionLib846::Named::Join({"a", "b"}), "a/b");
  EXPECT_EQ(transitionLib846::Named::Join({"a", "b", "c"}), "a/b/c");
}
