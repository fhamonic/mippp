// Shared library opened at runtime by test/dynamic_library.cpp; never linked.
// Built twice under the same file name (MIPPP_TEST_ANSWER differs) to stand in
// for two versions of one solver.
#if defined(_WIN32)
#define MIPPP_TEST_EXPORT __declspec(dllexport)
#else
#define MIPPP_TEST_EXPORT __attribute__((visibility("default")))
#endif

#ifndef MIPPP_TEST_ANSWER
#define MIPPP_TEST_ANSWER 42
#endif

extern "C" MIPPP_TEST_EXPORT int mippp_test_answer(void) {
    return MIPPP_TEST_ANSWER;
}
extern "C" MIPPP_TEST_EXPORT int mippp_test_add(int a, int b) { return a + b; }

// per-library state: two loaded copies must not share it
static int counter = 0;
extern "C" MIPPP_TEST_EXPORT int mippp_test_bump(void) { return ++counter; }
