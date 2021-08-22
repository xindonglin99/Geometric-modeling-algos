#pragma once


#include <vector>

#include <minimesh/viz/opengl_headers.hpp>

namespace minimesh
{

struct Glui_slider
{
	GLUI_Translation *x_translation;
	GLUI_StaticText  *text;
	using User_callback = void (*)(void*);
  
	void init();
	void add_to_glui(GLUI*, const char* name);
	void add_to_glui_panel(GLUI*, GLUI_Panel*, const char* name);
	void set_limits(const int min_val, const int max_val);
	void set_user_callback( User_callback, void* );

	int get_live_var();

private:
	int _min_val;
	int _max_val;
	int _live_var;

	User_callback _user_callback;
	void* _user_callback_data;
	int _unique_id;

	void callback();
	void update_text();

	// Due to the fact that the callback has no access
	// to the object, we need some counter measures.
	static void static_callback(int);
	static std::vector<Glui_slider*> _all_present_sliders;
};


} // end of minimesh
