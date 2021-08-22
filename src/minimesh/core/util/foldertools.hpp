#pragma once

// Create and change directories
// Author: Shayan Hoshyari

#include <string>

namespace minimesh
{
namespace foldertools
{

// Create a folder
bool makedir(const char * name);

// Go to a folder
bool setdir(const char * name);

// Create a folder and then go to it.
bool makeandsetdir(const char * name);

// Equivalent of the command pushd in bash.
void pushd();

// Equivalent of the command pwd in bash.
void popd();

// get current working directory
// the int is the maximum size of the array in the second version
std::string cwd();
void cwd(char[], const int); 

} // end of foldertools
} // End of minimesh

