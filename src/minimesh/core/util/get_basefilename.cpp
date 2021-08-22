#include "get_basefilename.hpp"

namespace minimesh
{
  
std::string get_basefilename(std::string in)
{
	std::size_t character_at;
  
	character_at = in.rfind('\\');
	if(character_at != std::string::npos)
		in = in.substr(character_at+1);

	character_at = in.rfind('/');
	if(character_at != std::string::npos)
		in = in.substr(character_at+1);

	character_at = in.rfind('.');
	if(character_at != std::string::npos)
		in = in.substr(0, character_at);

  return in;
}

}
