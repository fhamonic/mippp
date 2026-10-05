#include <gtest/gtest.h>

#include <stdexcept>

#include "mippp/detail/restore_guard.hpp"

using mippp::detail::restore_guard;

namespace {

struct call_failure : std::runtime_error {
    using runtime_error::runtime_error;
};
struct write_back_failure : std::runtime_error {
    using runtime_error::runtime_error;
};

}  // namespace

TEST(restore_guard, restore_writes_back_once) {
    int writes = 0;
    restore_guard guard([&] { ++writes; });
    EXPECT_EQ(writes, 0);
    guard.restore();
    EXPECT_EQ(writes, 1);
}

TEST(restore_guard, no_second_write_after_restore) {
    int writes = 0;
    {
        restore_guard guard([&] { ++writes; });
        guard.restore();
        guard.restore();
    }
    EXPECT_EQ(writes, 1);
}

// The caller sees the call's error, not the write-back's.
TEST(restore_guard, the_destructor_writes_back_after_an_exception) {
    int writes = 0;
    try {
        restore_guard guard([&] {
            ++writes;
            throw write_back_failure("write-back");
        });
        throw call_failure("call");
    } catch(const call_failure &) {
    }
    EXPECT_EQ(writes, 1);
}

TEST(restore_guard, an_error_in_restore_propagates_and_is_not_retried) {
    int writes = 0;
    {
        restore_guard guard([&] {
            ++writes;
            throw write_back_failure("write-back");
        });
        EXPECT_THROW(guard.restore(), write_back_failure);
    }
    EXPECT_EQ(writes, 1);
}
