#pragma once

#include <cassert>
#include <sstream>
#include <exception>

#include <minimesh/core/util/console_colors.hpp>

// An assertion that gets triggered even in optimized mode.
// Usage example:
// force_assert(i==2, "i is " << i << " which is not 2");
#define force_assert_msg(EXPR, MSG)                                                   \
  if(!(EXPR))                                                                         \
  {                                                                                   \
    std::ostringstream ss;                                                            \
    ss << std::scientific;                                                            \
    printf("Assertion in %s, %d \n", __func__, __LINE__);                             \
    ss.str("");                                                                       \
    ss << #EXPR << "\n";                                                              \
    ss << MSG;                                                                        \
    printf("%s %s %s\n", ::minimesh::consolecolors::red(), ss.str().c_str(), ::minimesh::consolecolors::reset()); \
    assert(EXPR);                                                                     \
    throw std::runtime_error("error occured");                                        \
  }

// Usage example:
// force_assert(i==2);
#define force_assert(EXPR) force_assert_msg(EXPR, "")

#if !defined(NDEBUG)
#define debug_assert(COND)              force_assert(COND)
#define debug_assert_msg(COND, MSG)     force_assert_msg(COND, MSG)
#else
#define debug_assert(COND)
#define debug_assert_msg(COND, MSG)
#endif

