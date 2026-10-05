#pragma once

#include <concepts>
#include <utility>

namespace mippp::detail {

// Writes back what a call set on the native model for its own use. restore()
// writes back once and lets an error out. The destructor writes back only
// when restore() did not run, as after an exception, and swallows an error
// there, since it could not be reported over the one in flight. A write-back
// of several items must attempt them all before it raises the first error, or
// a rejected write leaves the later items set.
template <std::invocable WriteBack>
class restore_guard {
private:
    WriteBack _write_back;
    bool _armed = true;

public:
    // a discarded guard would write back at once, before the writes it undoes
    [[nodiscard]] explicit restore_guard(WriteBack write_back)
        : _write_back(std::move(write_back)) {}
    restore_guard(const restore_guard &) = delete;
    restore_guard & operator=(const restore_guard &) = delete;
    ~restore_guard() {
        if(!_armed) return;
        try {
            _write_back();
        } catch(...) {
        }
    }

    // disarms first: a write-back that throws here is not run again by the
    // destructor
    void restore() {
        if(std::exchange(_armed, false)) _write_back();
    }
};

}  // namespace mippp::detail
