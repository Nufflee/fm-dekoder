#include <iostream>

#define ASSERT(actual) assert_func(#actual, actual, __FILE__, __LINE__)
#define ASSERT_EQ(actual, expected) assert_eq_func(#actual, actual, #expected, expected, __FILE__, __LINE__)

template <typename TActual>
static inline void assert_func(const char *actualExpression, TActual &&actual, const char *file, int line)
{
  if ((bool)actual != true)
  {
    std::cerr << "Assert failed: " << actualExpression << ", " << file << ":" << line << std::endl;
    abort();
  }
}

template <typename TActual, typename TExpected>
static inline void assert_eq_func(const char *actualExpression, TActual &&actual, const char *expectedExpression, TExpected &&expected, const char *file, int line)
{
  if ((TExpected)actual != expected)
  {
    std::cerr << "Assert failed: " << actualExpression << " == " << expectedExpression << " (expected `" << expected << "`, gotten `" << (TExpected)actual << "`), " << file << ":" << line << std::endl;
    abort();
  }
}