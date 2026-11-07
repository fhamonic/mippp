#include "probe_xpress_hook.hpp"
int main() { M m; char v[32] = {}; m.native_api().getversion(v); std::println(stderr, "XPRSgetversion={} library_path={}", v, m.native_api().library_path().string()); }
