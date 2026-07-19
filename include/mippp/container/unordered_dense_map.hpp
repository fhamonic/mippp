#pragma once
// unordered_dense_map: hash map with cache-friendly iteration and single key
// storage.
//
// Layout:
//   std::vector<std::pair<Key, Value>>   -- dense storage, iterate over this
//   std::unordered_set<Index>        -- indices into the vector; hashing and
//                                       equality dereference through the
//                                       vector, so the Key is stored exactly
//                                       once. Index is a strong wrapper over
//                                       std::size_t so its transparent
//                                       overloads never collide with Key --
//                                       even when Key itself is std::size_t.
//
// Requires C++20 (heterogeneous lookup in unordered containers).
//
// STL compatibility:
//  The public interface mirrors std::unordered_map for the common operations,
//  BUT the following std::unordered_map guarantees are deliberately NOT met:
//   * value_type is std::pair<Key, Value> (mutable key), not
//     std::pair<const Key, Value>. Mutating a key through an iterator corrupts
//     the index -- don't.
//   * References/pointers/iterators are invalidated on insert (rehash/growth)
//     and on erase. std::unordered_map keeps references stable across inserts;
//     this container cannot, because elements live in a std::vector.
//   * erase() uses swap-and-pop: O(1) but does NOT preserve iteration order,
//     and invalidates the iterator/index of the last element. The
//     erase(iterator) overloads therefore return an iterator to the slot that
//     was erased (now holding the swapped-in element), following flat-map
//     convention, not the "next in order" element.
//
//  Heterogeneous lookup (find/contains/count/at/equal_range/erase taking a
//  non-Key argument) is only enabled when both Hash and KeyEqual are
//  transparent (define a nested is_transparent), matching the standard.

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mippp {

template <typename Key, typename Value, typename Hash = std::hash<Key>,
          typename KeyEqual = std::equal_to<Key>>
class unordered_dense_map {
    // Detect whether Hash/KeyEqual opt into heterogeneous lookup.
    template <typename T, typename = void>
    struct has_is_transparent : std::false_type {};
    template <typename T>
    struct has_is_transparent<T, std::void_t<typename T::is_transparent>>
        : std::true_type {};
    template <typename H, typename E>
    static constexpr bool is_transparent_v =
        has_is_transparent<H>::value && has_is_transparent<E>::value;

    using Entry = std::pair<Key, Value>;
    using Store = std::vector<Entry>;

    // Strong type wrapping std::size_t so that transparent overloads below
    // don't collide if Key==std::size_t
    struct Index {
        std::size_t value;
    };

    struct IndexHash {
        using is_transparent = void;
        const Store * store;
        std::size_t operator()(Index i) const {
            return Hash{}((*store)[i.value].first);
        }
        template <typename K>
        std::size_t operator()(const K & k) const {
            return Hash{}(k);
        }
    };

    struct IndexEqual {
        using is_transparent = void;
        const Store * store;
        bool operator()(Index a, Index b) const {
            return KeyEqual{}((*store)[a.value].first, (*store)[b.value].first);
        }
        template <typename K>
        bool operator()(Index a, const K & k) const {
            return KeyEqual{}((*store)[a.value].first, k);
        }
        template <typename K>
        bool operator()(const K & k, Index a) const {
            return KeyEqual{}(k, (*store)[a.value].first);
        }
    };

    using IndexSet = std::unordered_set<Index, IndexHash, IndexEqual>;

    // unique_ptr keeps the Store's address stable across moves so the pointers
    // captured by IndexHash and IndexEqual stays valid
    std::unique_ptr<Store> store_;
    IndexSet index_;

public:
    using key_type = Key;
    using mapped_type = Value;
    using value_type = Entry;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using hasher = Hash;
    using key_equal = KeyEqual;
    using reference = value_type &;
    using const_reference = const value_type &;
    using pointer = typename Store::pointer;
    using const_pointer = typename Store::const_pointer;
    using iterator = typename Store::iterator;
    using const_iterator = typename Store::const_iterator;

    unordered_dense_map()
        : store_(std::make_unique<Store>())
        , index_(0, IndexHash{store_.get()}, IndexEqual{store_.get()}) {}

    template <typename InputIt>
    unordered_dense_map(InputIt first, InputIt last) : unordered_dense_map() {
        insert(first, last);
    }
    unordered_dense_map(std::initializer_list<value_type> il)
        : unordered_dense_map() {
        insert(il.begin(), il.end());
    }

    unordered_dense_map(const unordered_dense_map & other)
        : store_(std::make_unique<Store>(*other.store_))
        , index_(other.index_.begin(), other.index_.end(),
                 other.index_.bucket_count(), IndexHash{store_.get()},
                 IndexEqual{store_.get()}) {}

    unordered_dense_map(unordered_dense_map &&) noexcept = default;

    unordered_dense_map & operator=(const unordered_dense_map & other) {
        if(this != &other) {
            unordered_dense_map tmp(other);
            swap(tmp);
        }
        return *this;
    }
    unordered_dense_map & operator=(unordered_dense_map &&) noexcept = default;

    unordered_dense_map & operator=(std::initializer_list<value_type> il) {
        clear();
        insert(il.begin(), il.end());
        return *this;
    }

    void swap(unordered_dense_map & other) noexcept {
        store_.swap(other.store_);
        index_.swap(other.index_);
    }

    iterator begin() { return store_->begin(); }
    iterator end() { return store_->end(); }
    const_iterator begin() const { return store_->begin(); }
    const_iterator end() const { return store_->end(); }
    const_iterator cbegin() const { return store_->cbegin(); }
    const_iterator cend() const { return store_->cend(); }

    size_type size() const { return store_->size(); }
    bool empty() const { return store_->empty(); }

    void reserve(size_type n) {
        store_->reserve(n);
        index_.reserve(n);
    }

    void clear() {
        index_.clear();
        store_->clear();
    }

    iterator find(const Key & k) {
        auto it = index_.find(k);
        return it == index_.end()
                   ? end()
                   : begin() + static_cast<difference_type>(it->value);
    }
    const_iterator find(const Key & k) const {
        auto it = index_.find(k);
        return it == index_.end()
                   ? end()
                   : begin() + static_cast<difference_type>(it->value);
    }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    iterator find(const K & k) {
        auto it = index_.find(k);
        return it == index_.end()
                   ? end()
                   : begin() + static_cast<difference_type>(it->value);
    }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    const_iterator find(const K & k) const {
        auto it = index_.find(k);
        return it == index_.end()
                   ? end()
                   : begin() + static_cast<difference_type>(it->value);
    }

    bool contains(const Key & k) const {
        return index_.find(k) != index_.end();
    }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    bool contains(const K & k) const {
        return index_.find(k) != index_.end();
    }

    size_type count(const Key & k) const { return contains(k) ? 1 : 0; }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    size_type count(const K & k) const {
        return contains(k) ? 1 : 0;
    }

    std::pair<iterator, iterator> equal_range(const Key & k) {
        auto it = find(k);
        return {it, it == end() ? it : std::next(it)};
    }
    std::pair<const_iterator, const_iterator> equal_range(const Key & k) const {
        auto it = find(k);
        return {it, it == end() ? it : std::next(it)};
    }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    std::pair<iterator, iterator> equal_range(const K & k) {
        auto it = find(k);
        return {it, it == end() ? it : std::next(it)};
    }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    std::pair<const_iterator, const_iterator> equal_range(const K & k) const {
        auto it = find(k);
        return {it, it == end() ? it : std::next(it)};
    }

    Value & at(const Key & k) {
        auto it = index_.find(k);
        if(it == index_.end())
            throw std::out_of_range("unordered_dense_map::at");
        return (*store_)[it->value].second;
    }
    const Value & at(const Key & k) const {
        auto it = index_.find(k);
        if(it == index_.end())
            throw std::out_of_range("unordered_dense_map::at");
        return (*store_)[it->value].second;
    }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    Value & at(const K & k) {
        auto it = index_.find(k);
        if(it == index_.end())
            throw std::out_of_range("unordered_dense_map::at");
        return (*store_)[it->value].second;
    }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<is_transparent_v<H, E>, int> = 0>
    const Value & at(const K & k) const {
        auto it = index_.find(k);
        if(it == index_.end())
            throw std::out_of_range("unordered_dense_map::at");
        return (*store_)[it->value].second;
    }

    template <typename... Args>
    std::pair<iterator, bool> try_emplace(const Key & k, Args &&... args) {
        return try_emplace_impl(k, std::forward<Args>(args)...);
    }
    template <typename... Args>
    std::pair<iterator, bool> try_emplace(Key && k, Args &&... args) {
        return try_emplace_impl(std::move(k), std::forward<Args>(args)...);
    }

    template <typename... Args>
    std::pair<iterator, bool> emplace(Args &&... args) {
        value_type v(std::forward<Args>(args)...);
        return try_emplace(std::move(v.first), std::move(v.second));
    }

    std::pair<iterator, bool> insert(const value_type & v) {
        return try_emplace(v.first, v.second);
    }
    std::pair<iterator, bool> insert(value_type && v) {
        return try_emplace(std::move(v.first), std::move(v.second));
    }
    template <typename InputIt>
    void insert(InputIt first, InputIt last) {
        for(; first != last; ++first) insert(*first);
    }
    void insert(std::initializer_list<value_type> il) {
        insert(il.begin(), il.end());
    }

    template <typename M>
    std::pair<iterator, bool> insert_or_assign(const Key & k, M && obj) {
        if(auto it = index_.find(k); it != index_.end()) {
            const size_type i = it->value;
            (*store_)[i].second = std::forward<M>(obj);
            return {begin() + static_cast<difference_type>(i), false};
        }
        return try_emplace(k, std::forward<M>(obj));
    }
    template <typename M>
    std::pair<iterator, bool> insert_or_assign(Key && k, M && obj) {
        if(auto it = index_.find(k); it != index_.end()) {
            const size_type i = it->value;
            (*store_)[i].second = std::forward<M>(obj);
            return {begin() + static_cast<difference_type>(i), false};
        }
        return try_emplace(std::move(k), std::forward<M>(obj));
    }

    Value & operator[](const Key & k) { return try_emplace(k).first->second; }
    Value & operator[](Key && k) {
        return try_emplace(std::move(k)).first->second;
    }

    size_type erase(const Key & k) { return erase_key(k); }
    template <typename K, typename H = Hash, typename E = KeyEqual,
              std::enable_if_t<
                  is_transparent_v<H, E> &&
                      !std::is_convertible_v<const K &, const_iterator> &&
                      !std::is_convertible_v<const K &, iterator>,
                  int> = 0>
    size_type erase(const K & k) {
        return erase_key(k);
    }

    iterator erase(const_iterator pos) {
        const size_type i = static_cast<size_type>(pos - cbegin());
        erase_slot(i);
        return begin() + static_cast<difference_type>(i);
    }
    iterator erase(iterator pos) {
        return erase(static_cast<const_iterator>(pos));
    }

    const Store & values() const { return *store_; }

private:
    template <typename K2, typename... Args>
    std::pair<iterator, bool> try_emplace_impl(K2 && k, Args &&... args) {
        if(auto it = index_.find(k); it != index_.end())
            return {begin() + static_cast<difference_type>(it->value), false};

        store_->emplace_back(
            std::piecewise_construct,
            std::forward_as_tuple(std::forward<K2>(k)),
            std::forward_as_tuple(std::forward<Args>(args)...));
        try {
            index_.insert(Index{store_->size() - 1});
        } catch(...) {
            store_->pop_back();
            throw;
        }
        return {std::prev(end()), true};
    }

    template <typename K>
    size_type erase_key(const K & k) {
        auto it = index_.find(k);
        if(it == index_.end()) return 0;
        erase_slot(it->value, it);
        return 1;
    }

    // swap-and-pop the element at slot i out of both store and index.
    void erase_slot(size_type i) { erase_slot(i, index_.find(Index{i})); }

    template <typename SetIt>
    void erase_slot(size_type i, SetIt it) {
        const size_type last = store_->size() - 1;
        index_.erase(it);
        if(i != last) {
            index_.erase(Index{last});  // un-index the element we move
            (*store_)[i] = std::move((*store_)[last]);
            store_->pop_back();
            index_.insert(Index{i});  // re-index it at its new slot
        } else {
            store_->pop_back();
        }
    }
};

}  // namespace mippp
