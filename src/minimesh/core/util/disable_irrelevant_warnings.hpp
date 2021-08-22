// Code for disabling some annoying warnings
// No include guard
// Author: Shayan Hoshyari

#if defined(_MSC_VER)
#pragma warning( push )
#pragma warning (disable: 4244) // convert higher integer type, e.g. int64 to lower types, e.g. int32.
#pragma warning (disable: 4267) // convert size_t to int
#pragma warning (disable: 4305) // convert double to float
#elif defined(__GNUG__)
// TODO
#endif
