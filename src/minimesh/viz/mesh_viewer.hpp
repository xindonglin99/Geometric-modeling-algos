#pragma once

#include <vector>

#include <Eigen/Core>

#include <minimesh/viz/mesh_buffer.hpp>
#include <minimesh/viz/opengl_headers.hpp>


namespace minimesh
{

// A very simple class to manage operations related to viewing and operating with
// a mesh.

class Mesh_viewer
{
public:

	constexpr static int invalid_index = -1;

	//
	// Init the viewer
	//
	// Initialize the viewer. bounding_box should be the bounding box of the mesh.
	void initialize(const Eigen::AlignedBox<float, 3> & bounding_box);

	//
	// Returns the mesh_buffer object associated with the viewer.
	// If the mesh changes, you should call get_mesh_buffer().rebuild(mesh)
	//
	Mesh_buffer & get_mesh_buffer() { return _mesh_buffer; }

	//
	// Useful for ARAP ASSINGMENT
	// Vertex selection and movement
	//

	//
	// return functionality of the mouse.
	// You can change this depending on what you want left clicks to do.
	//
	enum Mouse_function : int
	{
		MOUSE_VIEW = 0,
		MOUSE_SELECT,
		MOUSE_MOVE_VERTEX,
		MOUSE_INVALID,
	};
	int & get_mouse_function() { return _mouse_function; }

	// Whether user has just selected a vertex
	// was_selection_made: Did user just click on a vertex
	// selected_vertex: index of the clicked vertex (actual index in the mesh, and not the defragmented one)
	void get_and_clear_vertex_selection(bool &was_selection_made, int &selected_vertex);

	// Whether user has tried to displace a vertex
	// was_displaced: Did user just click pull a vertex
	// disp_amount: How much has the user pulled since the last call to get_and_clear_vertex_displacement()
	//              Note that every time you call this function the pulled amount is set to zero.
	// pulled_vertex: index of the pulled vertex (actual index in the mesh, and not the defragmented one)
	void get_and_clear_vertex_displacement(bool &was_displaced, Eigen::Vector3f& disp_amount, int &pulled_vertex);


	//
	// Viewing various elements of the mesh
	//
	int & get_has_lighting() { return _has_lighting; }
	int & get_draw_vertices() { return _draw_vertices; }
	int & get_draw_edges() { return _draw_edges; }
	int & get_draw_faces() { return _draw_faces; }
	int & get_draw_axis() { return _draw_axis; }
  
	//
	//  Call these withing freeglut event functions
	// 

	// Draw() function. Call this within your freeglut draw function.
	void draw();

	// Call this within your freeglut window reshape function.
	// return true, if the scene has to be re-rendered, false otherwise
	bool window_reshaped(const int w, const int h);

	// Call this within your freeglut mouse pushed function.
	// return true, if the scene has to be re-rendered, false otherwise
	bool mouse_pushed(int button, const int state, const int x, const int y);

	// Call this within your freeglut mouse moved function.
	// return true, if the scene has to be re-rendered, false otherwise
	bool mouse_moved(const int x, const int y);

	// Call this within your freeglut keyboard pressed function.
	// return true, if the scene has to be re-rendered, false otherwise
	bool keyboard_pressed(unsigned char c, int x, int y);


private:
	Mesh_buffer _mesh_buffer;

	int _has_lighting;
	int _draw_vertices;
	int _draw_edges;
	int _draw_faces;
	int _draw_axis;

	float _near;
	float _far;
	Eigen::Vector4f _view_quaternion;
	float _aspect;
	float _bbox_diameter;

	int _win_width;
	int _win_height;
	Eigen::Vector3f _obj_center;
	Eigen::Vector3f _center_offset;
	float _view_distance;

	Eigen::Vector4f _light_dir0;
	Eigen::Vector4f _light_dir1;
	Eigen::Vector4f _light_color;
	Eigen::Vector4f _specular_color;
	Eigen::Vector4f _amb_color;
	Eigen::Vector4f _black;
	Eigen::Vector4f _default_surface_color;
	Eigen::Vector4f _background_color;
	float _exponent;

	Eigen::Matrix4d _model_view_matrix;
	Eigen::Matrix4d _projection_matrix;
	Eigen::Vector4i _view_port;

	int _xPos;
	int _yPos;
	int _mouse_button;

	int _mouse_function;

	int _selected_vertex_mesh_buffer_id;
	bool _has_user_just_selected;

	Eigen::Vector2i _pull_current_pos;
	Eigen::Vector2i _pull_latest_querried_pos;
	Eigen::Vector2i _pull_initial_pos; 
	float _pulled_vertex_depth;
	int _pulled_vertex_buffer_id;
	bool _is_pulling_in_progress;
	bool _has_user_just_pulled;

	bool _is_in_gl_begin;

	Eigen::Vector2i freeglut2gl_screen(const Eigen::Vector2i&, const Eigen::Vector4i & view_port);
  
	static Eigen::Vector3d screen2world_generic(
		int gl_clickx,
		int gl_clicky,
		double depth,
		const Eigen::Matrix4d & model_view_matrix,
		const Eigen::Matrix4d & projection_matrix,
		const Eigen::Vector4i & view_port);
  
	Eigen::Vector3f screen2world_given_depth(int freeglut_clickx, int freeglut_clicky, float depth);

	void screen2world_find_depth(
		int freeglut_clickx, 
		int freeglut_clicky, 
		bool & is_mesh_clicked, 
		Eigen::Vector3f & global_pos, 
		float *depth);

	int screen2vertex(int clickx, int clicky, float* depth=nullptr);

	void record_mouse_pos(int clickx, int clicky, int button);
	Eigen::Quaternionf get_view_quaternion();

	void gl_begin(GLenum mode);
	void gl_end();
	void gl_check_error();
	static void static_gl_check_error();
};


} // end of minimesh


