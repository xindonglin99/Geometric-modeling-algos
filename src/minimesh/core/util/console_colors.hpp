#pragma once

// Colors for printing to console. 
// It definitely works on Linux and MAC. On windows, sometimes it works, sometimes it doesn't.
// Thanks to
// https://github.com/dthuerck/mapmap_cpu/blob/master/mapmap/header/mapmap.h

namespace minimesh
{
namespace consolecolors
{ 
	inline const char * const black() { return  "\033[0;30m";}
	inline const char * const darkblue () { return "\033[0;34m";}
	inline const char * const darkgreen() { return "\033[0;32m";}
	inline const char * const darkteal() { return "\033[0;36m";}
	inline const char * const darkred() { return "\033[0;31m";}
	inline const char * const darkpink() { return "\033[0;35m";}
	inline const char * const darkyellow() { return "\033[0;33m";}
	inline const char * const gray() { return "\033[0;37m";}
	inline const char * const darkgray() { return "\033[1;30m";}
	inline const char * const blue() { return "\033[1;34m";}
	inline const char * const green() { return "\033[1;32m";}
	inline const char * const teal() { return "\033[1;36m";}
	inline const char * const red() { return "\033[1;31m";}
	inline const char * const pink() { return "\033[1;35m";}
	inline const char * const yellow() { return "\033[1;33m";}
	inline const char * const white() { return "\033[1;37m";}
	inline const char * const reset() { return "\033[0m";}
}
}
