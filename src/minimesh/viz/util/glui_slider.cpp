#include <cmath>

#include <minimesh/core/util/assert.hpp>

#include <minimesh/viz/util/glui_slider.hpp>

namespace minimesh
{

std::vector<Glui_slider *> Glui_slider::_all_present_sliders;

void Glui_slider::init()
{
	x_translation = nullptr;
	text = nullptr;
	_min_val = 0;
	_max_val = 1;
	_live_var = 0;
	_user_callback = nullptr;
	_user_callback_data = nullptr;
	_unique_id = (int)_all_present_sliders.size();
	_all_present_sliders.push_back(this);
}


void Glui_slider::add_to_glui(GLUI * glui, const char* name)
{
	force_assert(!x_translation);
	force_assert(!text);
	x_translation = glui->add_translation(name, GLUI_TRANSLATION_X, nullptr, _unique_id, static_callback);
	text = glui->add_statictext("");
	update_text();
}


void Glui_slider::add_to_glui_panel(GLUI * glui, GLUI_Panel * panel, const char* name)
{
	force_assert(!x_translation);
	force_assert(!text);
	x_translation = glui->add_translation_to_panel(panel, name, GLUI_TRANSLATION_X, nullptr, _unique_id, static_callback);
	text = glui->add_statictext_to_panel(panel, "");
	update_text();
}


void Glui_slider::set_limits(const int min_val, const int max_val)
{
	_min_val = min_val;
	_max_val = max_val;
	_live_var = min_val;
	force_assert(_max_val > _min_val);
	update_text();
}


void Glui_slider::set_user_callback(User_callback user_callback, void * user_callback_data)
{
	_user_callback = user_callback;
	_user_callback_data = user_callback_data;
}


void Glui_slider::callback()
{
	force_assert(x_translation);
	force_assert(text);

	double value = x_translation->get_x();
	value = std::max(value, (double)_min_val);
	value = std::min(value, (double)_max_val);
	x_translation->set_x((float)value);
	_live_var = (int)std::round(value);

	update_text();

	if(_user_callback)
		_user_callback(_user_callback_data);
}


void Glui_slider::update_text()
{
	static char text_to_set[4096];

	if(text)
	{
		sprintf(text_to_set, "%05d, [", _live_var);
		const int max_stars = 10;
		const int n_stars = (int)std::floor(double(_live_var - _min_val) / (_max_val - _min_val) * max_stars);
		for(int i = 0; i < n_stars; ++i)
		{
			sprintf(text_to_set+strlen(text_to_set), "+");
		}
		for(int i = n_stars; i < max_stars; ++i)
		{
			sprintf(text_to_set+strlen(text_to_set), " ");
		}
		//sprintf(text_to_set+strlen(text_to_set), "");

		text->set_text(text_to_set);
	}
}


int Glui_slider::get_live_var()
{
	return _live_var;
}


void Glui_slider::static_callback(int unique_id)
{
	force_assert(unique_id < (int)_all_present_sliders.size());
	force_assert(_all_present_sliders[unique_id]);
	_all_present_sliders[unique_id]->callback();
}


} // end of minimesh
