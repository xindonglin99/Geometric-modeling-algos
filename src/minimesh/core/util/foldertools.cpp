// Create and change directories
// Author: Shayan Hoshyari

#include <cassert>
#include <cstring>
#include <string>

#include "foldertools.hpp"
#include "macros.hpp"

#define MAX_STRING_LEN 4048
#define MAX_STACK_DEPTH 20

namespace minimesh
{

namespace foldertools
{


#ifdef WIN32

#include "Windows.h"


static bool makesingledir(const char * path)
{
	CreateDirectory(path, NULL);
	return true;
}


bool setdir(const char * name)
{
	SetCurrentDirectory(name);
	return true;
}


void cwd(char ans[], const int max_size)
{
  GetCurrentDirectory(max_size, ans);
}

#else /*WIN32*/

// NOTE TESTED YET
// https://stackoverflow.com/questions/7430248/creating-a-new-directory-in-c
// https://stackoverflow.com/questions/2336242/recursive-mkdir-system-call-on-unix

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

static bool makesingledir(const char * name)
{
	char tmp[MAX_STRING_LEN];
	char * p = NULL;
	size_t len;

	snprintf(tmp, sizeof(tmp), "%s", name);
	len = strlen(tmp);
	if(tmp[len - 1] == '/')
		tmp[len - 1] = 0;
	for(p = tmp + 1; *p; p++)
		if(*p == '/')
		{
			*p = 0;
			mkdir(tmp, S_IRWXU);
			*p = '/';
		}
	mkdir(tmp, S_IRWXU);

	return true;
}

bool setdir(const char * name)
{
	int success = chdir(name);
	return (success == 0);
}

void cwd(char ans[], const int max_size)
{
	char * success = getcwd(ans, max_size);
	MINIMESH_UNUSED(success);
}
#endif

static bool is_valid_folder(std::string in)
{
	if(in == ".")
		return false;
	if(in == "")
		return false;
	if(in == "\\")
		return false;
	if(in == "/")
		return false;
	if(in == ".\\")
		return false;
	if(in == "./")
		return false;
	if(in == "")
		return false;

	return true;
}

static char stack[MAX_STACK_DEPTH][MAX_STRING_LEN];
static int stack_pos = 0;


// A fixed version of the buggy code in.
// https://stackoverflow.com/questions/1530760/how-do-i-recursively-create-a-folder-in-win32

bool makedir(const char * name)
{
	std::string path(name);
	std::string::size_type pos = 0;
	std::string::size_type pos_prev = 0;
	while(true)
	{
		pos_prev = pos;
		pos = path.find_first_of("\\/", pos + 1);
		const std::string current = path.substr(pos_prev, pos);

		// No slashes,
		if(pos == std::string::npos)
		{
			// valid dir, create and then get out.
			if(is_valid_folder(current))
			{
				makesingledir(path.c_str());
				break;
			}
			// Invalid nonesense, get out
			else
			{
				break;
			}
		}
		// Found a slash
		else
		{
			// valid dir, create
			if(is_valid_folder(current))
			{
				makesingledir(path.c_str());
			}
		}
	}

	return true;
}


bool makeandsetdir(const char * name)
{
	makedir(name);
	setdir(name);
	return true;
}


void pushd()
{
	assert(stack_pos < MAX_STACK_DEPTH - 1);
	cwd(stack[stack_pos], MAX_STRING_LEN);
	++stack_pos;
}


void popd()
{
	if(stack_pos > 0)
	{
		--stack_pos;
		setdir(stack[stack_pos]);
	}
}

std::string cwd()
{
	char ans[MAX_STRING_LEN];
	cwd(ans, MAX_STRING_LEN);
	return std::string(ans);
}

} // End of minimesh
} // end of foldertools
