#undef NDEBUG
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "mippp/container/unordered_dense_map.hpp"

using mippp::unordered_dense_map;

using map_t = unordered_dense_map<int, int>;

// Transparent functors so heterogeneous lookup on a std::string map can be
// driven by a std::string_view without materialising a std::string.
struct sv_hash {
    using is_transparent = void;
    std::size_t operator()(std::string_view s) const {
        return std::hash<std::string_view>{}(s);
    }
};
struct sv_eq {
    using is_transparent = void;
    bool operator()(std::string_view a, std::string_view b) const {
        return a == b;
    }
};
using string_map_t = unordered_dense_map<std::string, int, sv_hash, sv_eq>;

// --------------------------------------------------------------------------
// member typedefs
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, member_typedefs) {
    static_assert(std::is_same_v<map_t::key_type, int>);
    static_assert(std::is_same_v<map_t::mapped_type, int>);
    static_assert(std::is_same_v<map_t::value_type, std::pair<int, int>>);
    static_assert(std::is_same_v<map_t::size_type, std::size_t>);
    static_assert(std::is_same_v<map_t::difference_type, std::ptrdiff_t>);
    static_assert(std::is_same_v<map_t::hasher, std::hash<int>>);
    static_assert(std::is_same_v<map_t::key_equal, std::equal_to<int>>);
    static_assert(std::is_same_v<map_t::reference, std::pair<int, int> &>);
    static_assert(
        std::is_same_v<map_t::const_reference, const std::pair<int, int> &>);
}

// --------------------------------------------------------------------------
// construction
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, default_ctor) {
    map_t m;
    ASSERT_TRUE(m.empty());
    ASSERT_EQ(m.size(), 0u);
    ASSERT_EQ(m.begin(), m.end());
}

GTEST_TEST(unordered_dense_map, initializer_list_ctor) {
    map_t m{{1, 10}, {2, 20}, {3, 30}};
    ASSERT_EQ(m.size(), 3u);
    ASSERT_EQ(m.at(1), 10);
    ASSERT_EQ(m.at(2), 20);
    ASSERT_EQ(m.at(3), 30);
}

GTEST_TEST(unordered_dense_map, initializer_list_ctor_dedups) {
    // first occurrence of a key wins (try_emplace semantics)
    map_t m{{1, 10}, {1, 99}};
    ASSERT_EQ(m.size(), 1u);
    ASSERT_EQ(m.at(1), 10);
}

GTEST_TEST(unordered_dense_map, range_ctor) {
    std::vector<std::pair<int, int>> v{{1, 10}, {2, 20}};
    map_t m(v.begin(), v.end());
    ASSERT_EQ(m.size(), 2u);
    ASSERT_EQ(m.at(1), 10);
    ASSERT_EQ(m.at(2), 20);
}

GTEST_TEST(unordered_dense_map, copy_ctor_is_independent) {
    map_t a{{1, 10}, {2, 20}};
    map_t b(a);
    ASSERT_EQ(b.size(), 2u);
    b[1] = 111;
    b[3] = 30;
    ASSERT_EQ(a.at(1), 10);  // original untouched
    ASSERT_FALSE(a.contains(3));
    ASSERT_EQ(b.at(1), 111);
}

GTEST_TEST(unordered_dense_map, move_ctor) {
    map_t a{{1, 10}, {2, 20}};
    map_t b(std::move(a));
    ASSERT_EQ(b.size(), 2u);
    ASSERT_EQ(b.at(2), 20);
}

GTEST_TEST(unordered_dense_map, copy_assign_is_independent) {
    map_t a{{1, 10}};
    map_t b{{5, 50}, {6, 60}};
    b = a;
    ASSERT_EQ(b.size(), 1u);
    ASSERT_EQ(b.at(1), 10);
    b[1] = 999;
    ASSERT_EQ(a.at(1), 10);
}

GTEST_TEST(unordered_dense_map, move_assign) {
    map_t a{{1, 10}, {2, 20}};
    map_t b;
    b = std::move(a);
    ASSERT_EQ(b.size(), 2u);
    ASSERT_EQ(b.at(1), 10);
}

GTEST_TEST(unordered_dense_map, initializer_list_assign) {
    map_t m{{9, 90}};
    m = {{1, 10}, {2, 20}};
    ASSERT_EQ(m.size(), 2u);
    ASSERT_FALSE(m.contains(9));
    ASSERT_EQ(m.at(1), 10);
}

GTEST_TEST(unordered_dense_map, swap) {
    map_t a{{1, 10}};
    map_t b{{2, 20}, {3, 30}};
    a.swap(b);
    ASSERT_EQ(a.size(), 2u);
    ASSERT_EQ(b.size(), 1u);
    ASSERT_TRUE(a.contains(2));
    ASSERT_TRUE(b.contains(1));
    // lookup still works after swap -> internal store pointers stayed valid
    ASSERT_EQ(a.at(3), 30);
}

// --------------------------------------------------------------------------
// insertion
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, insert_returns_pair) {
    map_t m;
    auto [it, ok] = m.insert({1, 10});
    ASSERT_TRUE(ok);
    ASSERT_EQ(it->first, 1);
    ASSERT_EQ(it->second, 10);

    auto [it2, ok2] = m.insert({1, 99});
    ASSERT_FALSE(ok2);           // key already present
    ASSERT_EQ(it2->second, 10);  // value not overwritten
    ASSERT_EQ(m.size(), 1u);
}

GTEST_TEST(unordered_dense_map, try_emplace) {
    map_t m;
    auto [it, ok] = m.try_emplace(1, 10);
    ASSERT_TRUE(ok);
    ASSERT_EQ(it->second, 10);
    auto [it2, ok2] = m.try_emplace(1, 99);
    ASSERT_FALSE(ok2);
    ASSERT_EQ(it2->second, 10);
}

GTEST_TEST(unordered_dense_map, emplace) {
    map_t m;
    auto [it, ok] = m.emplace(1, 10);
    ASSERT_TRUE(ok);
    ASSERT_EQ(it->second, 10);
    ASSERT_FALSE(m.emplace(1, 99).second);
    ASSERT_EQ(m.at(1), 10);
}

GTEST_TEST(unordered_dense_map, range_insert) {
    std::vector<std::pair<int, int>> v{{1, 10}, {2, 20}, {2, 99}};
    map_t m;
    m.insert(v.begin(), v.end());
    ASSERT_EQ(m.size(), 2u);
    ASSERT_EQ(m.at(2), 20);
}

GTEST_TEST(unordered_dense_map, initializer_list_insert) {
    map_t m{{1, 10}};
    m.insert({{2, 20}, {3, 30}});
    ASSERT_EQ(m.size(), 3u);
    ASSERT_EQ(m.at(3), 30);
}

GTEST_TEST(unordered_dense_map, insert_or_assign) {
    map_t m;
    auto [it, ok] = m.insert_or_assign(1, 10);
    ASSERT_TRUE(ok);
    ASSERT_EQ(it->second, 10);

    auto [it2, ok2] = m.insert_or_assign(1, 99);
    ASSERT_FALSE(ok2);           // existed
    ASSERT_EQ(it2->second, 99);  // but value replaced
    ASSERT_EQ(m.at(1), 99);
    ASSERT_EQ(m.size(), 1u);
}

GTEST_TEST(unordered_dense_map, subscript_inserts_and_accesses) {
    map_t m;
    m[1] = 10;
    ASSERT_EQ(m.size(), 1u);
    ASSERT_EQ(m[1], 10);
    m[1] = 20;
    ASSERT_EQ(m[1], 20);
    ASSERT_EQ(m.size(), 1u);
    m[2];  // default-inserts
    ASSERT_EQ(m.size(), 2u);
    ASSERT_EQ(m.at(2), 0);
}

// --------------------------------------------------------------------------
// lookup
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, find) {
    map_t m{{1, 10}, {2, 20}};
    ASSERT_NE(m.find(1), m.end());
    ASSERT_EQ(m.find(1)->second, 10);
    ASSERT_EQ(m.find(3), m.end());

    const map_t & cm = m;
    ASSERT_NE(cm.find(2), cm.end());
    ASSERT_EQ(cm.find(2)->second, 20);
    ASSERT_EQ(cm.find(3), cm.end());
}

GTEST_TEST(unordered_dense_map, contains_and_count) {
    map_t m{{1, 10}};
    ASSERT_TRUE(m.contains(1));
    ASSERT_FALSE(m.contains(2));
    ASSERT_EQ(m.count(1), 1u);
    ASSERT_EQ(m.count(2), 0u);
}

GTEST_TEST(unordered_dense_map, at_throws_when_absent) {
    map_t m{{1, 10}};
    ASSERT_EQ(m.at(1), 10);
    ASSERT_THROW(m.at(2), std::out_of_range);
    const map_t & cm = m;
    ASSERT_EQ(cm.at(1), 10);
    ASSERT_THROW(cm.at(2), std::out_of_range);
}

GTEST_TEST(unordered_dense_map, equal_range) {
    map_t m{{1, 10}, {2, 20}};
    auto r = m.equal_range(1);
    ASSERT_NE(r.first, r.second);
    ASSERT_EQ(r.first->second, 10);
    ASSERT_EQ(std::next(r.first), r.second);

    auto miss = m.equal_range(3);
    ASSERT_EQ(miss.first, miss.second);
    ASSERT_EQ(miss.first, m.end());

    const map_t & cm = m;
    auto cr = cm.equal_range(2);
    ASSERT_NE(cr.first, cr.second);
    ASSERT_EQ(cr.first->second, 20);
}

// --------------------------------------------------------------------------
// erasure (swap-and-pop)
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, erase_by_key) {
    map_t m{{1, 10}, {2, 20}, {3, 30}};
    ASSERT_EQ(m.erase(2), 1u);
    ASSERT_EQ(m.erase(2), 0u);  // already gone
    ASSERT_EQ(m.erase(99), 0u);
    ASSERT_EQ(m.size(), 2u);
    ASSERT_FALSE(m.contains(2));
    ASSERT_TRUE(m.contains(1));
    ASSERT_TRUE(m.contains(3));
    ASSERT_EQ(m.at(1), 10);
    ASSERT_EQ(m.at(3), 30);
}

GTEST_TEST(unordered_dense_map, erase_last_element) {
    map_t m{{1, 10}, {2, 20}};
    // erase the element sitting in the last slot -> plain pop_back path
    auto last_key = std::prev(m.end())->first;
    ASSERT_EQ(m.erase(last_key), 1u);
    ASSERT_EQ(m.size(), 1u);
    ASSERT_FALSE(m.contains(last_key));
}

GTEST_TEST(unordered_dense_map, erase_keeps_other_keys_findable) {
    // swap-and-pop must re-index the moved element correctly
    map_t m;
    for(int i = 0; i < 100; ++i) m[i] = i * i;
    for(int i = 0; i < 100; i += 2) ASSERT_EQ(m.erase(i), 1u);
    ASSERT_EQ(m.size(), 50u);
    for(int i = 0; i < 100; ++i) {
        if(i % 2 == 0) {
            ASSERT_FALSE(m.contains(i));
        } else {
            ASSERT_TRUE(m.contains(i)) << "lost key " << i;
            ASSERT_EQ(m.at(i), i * i);
        }
    }
}

GTEST_TEST(unordered_dense_map, erase_iterator_returns_erased_slot) {
    map_t m{{1, 10}, {2, 20}, {3, 30}};
    auto it = m.find(1);
    auto slot = static_cast<std::size_t>(it - m.begin());
    auto next = m.erase(it);
    ASSERT_EQ(m.size(), 2u);
    ASSERT_FALSE(m.contains(1));
    // flat-map convention: returns iterator to the same slot (now the
    // swapped-in element), or end() if the last element was erased.
    ASSERT_EQ(next, m.begin() + static_cast<std::ptrdiff_t>(slot));
}

GTEST_TEST(unordered_dense_map, erase_all_via_iterator_idiom) {
    map_t m;
    for(int i = 0; i < 20; ++i) m[i] = i;
    // idiom for swap-and-pop: do NOT advance when erasing
    for(auto it = m.begin(); it != m.end();) {
        if(it->first % 3 == 0)
            it = m.erase(it);
        else
            ++it;
    }
    ASSERT_EQ(m.size(), 20u - 7u);  // 0,3,...,18 removed => 7 removed
    for(int i = 0; i < 20; ++i)
        ASSERT_EQ(m.contains(i), (i % 3 != 0)) << "key " << i;
}

GTEST_TEST(unordered_dense_map, erase_const_iterator) {
    map_t m{{1, 10}, {2, 20}};
    map_t::const_iterator cit = m.cbegin();
    auto key = cit->first;
    m.erase(cit);
    ASSERT_EQ(m.size(), 1u);
    ASSERT_FALSE(m.contains(key));
}

// --------------------------------------------------------------------------
// capacity / iteration / values
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, clear) {
    map_t m{{1, 10}, {2, 20}};
    m.clear();
    ASSERT_TRUE(m.empty());
    ASSERT_EQ(m.size(), 0u);
    ASSERT_FALSE(m.contains(1));
    m[5] = 50;  // usable after clear
    ASSERT_EQ(m.at(5), 50);
}

GTEST_TEST(unordered_dense_map, reserve_preserves_lookup) {
    map_t m{{1, 10}};
    m.reserve(1000);
    ASSERT_EQ(m.at(1), 10);
    m[2] = 20;
    ASSERT_EQ(m.at(2), 20);
}

GTEST_TEST(unordered_dense_map, dense_iteration_visits_all) {
    map_t m{{1, 10}, {2, 20}, {3, 30}};
    long sum_k = 0, sum_v = 0;
    int count = 0;
    for(const auto & [k, v] : m) {
        sum_k += k;
        sum_v += v;
        ++count;
    }
    ASSERT_EQ(count, 3);
    ASSERT_EQ(sum_k, 6);
    ASSERT_EQ(sum_v, 60);
}

GTEST_TEST(unordered_dense_map, values_is_backing_store) {
    map_t m{{1, 10}, {2, 20}};
    const auto & store = m.values();
    ASSERT_EQ(store.size(), 2u);
    long sum = 0;
    for(const auto & [k, v] : store) sum += v;
    ASSERT_EQ(sum, 30);
}

GTEST_TEST(unordered_dense_map, iterator_mutates_value) {
    map_t m{{1, 10}};
    m.find(1)->second = 42;
    ASSERT_EQ(m.at(1), 42);
}

// --------------------------------------------------------------------------
// move-only mapped type -> exercises single-key / piecewise construction
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, move_only_value) {
    unordered_dense_map<int, std::unique_ptr<int>> m;
    m.try_emplace(1, std::make_unique<int>(10));
    m[2] = std::make_unique<int>(20);
    m.insert({3, std::make_unique<int>(30)});
    ASSERT_EQ(m.size(), 3u);
    ASSERT_EQ(*m.at(1), 10);
    ASSERT_EQ(*m.at(2), 20);
    ASSERT_EQ(*m.at(3), 30);
    m.erase(2);
    ASSERT_FALSE(m.contains(2));
    ASSERT_EQ(*m.at(1), 10);
    ASSERT_EQ(*m.at(3), 30);
}

// --------------------------------------------------------------------------
// heterogeneous lookup (only when Hash & KeyEqual are transparent)
// --------------------------------------------------------------------------
GTEST_TEST(unordered_dense_map, heterogeneous_lookup) {
    string_map_t m;
    m["hello"] = 1;
    m["world"] = 2;

    std::string_view hello = "hello";
    ASSERT_NE(m.find(hello), m.end());
    ASSERT_EQ(m.find(hello)->second, 1);
    ASSERT_TRUE(m.contains(hello));
    ASSERT_EQ(m.count(hello), 1u);
    ASSERT_EQ(m.at(hello), 1);

    auto r = m.equal_range(std::string_view{"world"});
    ASSERT_NE(r.first, r.second);
    ASSERT_EQ(r.first->second, 2);

    const string_map_t & cm = m;
    ASSERT_EQ(cm.at(std::string_view{"world"}), 2);

    ASSERT_EQ(m.erase(std::string_view{"world"}), 1u);
    ASSERT_EQ(m.size(), 1u);
    ASSERT_FALSE(m.contains(std::string_view{"world"}));
}

// A non-transparent map must NOT expose heterogeneous overloads.
template <typename M, typename K, typename = void>
struct has_hetero_find : std::false_type {};
template <typename M, typename K>
struct has_hetero_find<
    M, K, std::void_t<decltype(std::declval<M &>().find(std::declval<K>()))>>
    : std::true_type {};

GTEST_TEST(unordered_dense_map, no_heterogeneous_lookup_by_default) {
    // std::hash<std::string> is not transparent -> find(string_view) disabled
    using plain = unordered_dense_map<std::string, int>;
    static_assert(has_hetero_find<plain, std::string>::value);
    static_assert(!has_hetero_find<plain, std::string_view>::value);
    // transparent map does expose it
    static_assert(has_hetero_find<string_map_t, std::string_view>::value);
}
