#include <Eigen/Geometry>
#include <Eigen/LU>

#include <minimesh/core/util/assert.hpp>
#include <minimesh/core/util/disable_irrelevant_warnings.hpp>
#include <minimesh/core/util/numbers.hpp>

#include <minimesh/viz/mesh_viewer.hpp>

namespace minimesh
{

void Mesh_viewer::initialize(const Eigen::AlignedBox<float, 3> & bounding_box)
{
	_view_quaternion = Eigen::Vector4f(0, 0, 0, 1);
	_aspect = 1.0;
	_has_lighting = 1;
	_draw_vertices = 0;
	_draw_edges = 1;
	_draw_faces = 1;
	_draw_axis = 1;
	_mouse_button = invalid_index;

	_light_dir0 = Eigen::Vector4f((float)sqrt(1. / 3), (float)sqrt(1. / 3), (float)sqrt(1. / 3), 0.0f);
	_light_dir1 = Eigen::Vector4f(-(float)sqrt(1. / 3), (float)sqrt(1. / 3), (float)sqrt(1. / 3), 0.0f);

	_background_color = Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f);
	_light_color = Eigen::Vector4f(0.9f, 0.9f, 0.9f, 0.0f);
	_specular_color = Eigen::Vector4f(0.2f, 0.2f, 0.2f, 0.0f);
	_amb_color = Eigen::Vector4f(0.5f, 0.5f, 0.5f, 0.0f);
	_black = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
	//_default_surface_color = Eigen::Vector4f(0.3f, 0.3f, 0.5f, 1.0f); // purple
	_default_surface_color = Eigen::Vector4f(0.6f, 0.6f, 0.6f, 1.0f); // gray
	_exponent = 128;

	glEnable(GL_DEPTH_TEST);  gl_check_error();
	glFrontFace(GL_CCW);  gl_check_error();
	glDepthFunc(GL_LEQUAL);   gl_check_error();
	glPointSize(2.0);  gl_check_error();
	glPolygonOffset(0.5, 0.0);  gl_check_error();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  gl_check_error();
	glDisable(GL_CULL_FACE);  gl_check_error();
	glEnable(GL_BLEND);  gl_check_error();
	glShadeModel(GL_SMOOTH);  gl_check_error();

	// The stencil buffer is used for selecting vertices
	glEnable(GL_STENCIL_TEST); gl_check_error();
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE); gl_check_error();

	Eigen::Vector3f min, max;
	min = bounding_box.min();
	max = bounding_box.max();

	_obj_center = (min + max) / 2.0;
	_center_offset.setZero();
	_bbox_diameter = (max - min).norm();

	// assuming a 45 degree field of view, 3*radiu of a bounding sphere
	// is a good viewing distance
	_view_distance = 1.5f * _bbox_diameter;
	_near = _view_distance - 1.4999f * _bbox_diameter;
	_far = _view_distance + 4.5f * _bbox_diameter;

	// Lighting and shading settings
	_has_lighting = true;
	glLightfv(GL_LIGHT0, GL_DIFFUSE, _amb_color.data()); gl_check_error();
	glLightfv(GL_LIGHT0, GL_SPECULAR, _light_color.data()); gl_check_error();
	glLightfv(GL_LIGHT0, GL_AMBIENT, _amb_color.data()); gl_check_error();
	glEnable(GL_LIGHT0); gl_check_error();
	glLightfv(GL_LIGHT1, GL_DIFFUSE, _light_color.data()); gl_check_error();
	glLightfv(GL_LIGHT1, GL_SPECULAR, _black.data()); gl_check_error();
	glLightfv(GL_LIGHT1, GL_AMBIENT, _black.data()); gl_check_error();
	glEnable(GL_LIGHT1); gl_check_error();
	glEnable(GL_LIGHTING); gl_check_error();

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, _black.data()); gl_check_error();
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, _black.data()); gl_check_error();
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _specular_color.data()); gl_check_error();
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, _exponent); gl_check_error();
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE); gl_check_error();
	glEnable(GL_COLOR_MATERIAL); gl_check_error();

	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 1.0); gl_check_error();
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, _black.data()); gl_check_error();

	_mouse_function = MOUSE_VIEW;

	_selected_vertex_mesh_buffer_id = invalid_index;
	_has_user_just_selected = false;

	_pull_latest_querried_pos = _pull_initial_pos = _pull_current_pos = Eigen::Vector2i::Zero();
	_pulled_vertex_buffer_id = invalid_index;
	_has_user_just_pulled = false;
	_is_pulling_in_progress = false;
	_pulled_vertex_depth = 0;

	_is_in_gl_begin = false;
}


void Mesh_viewer::draw()
{
	// clear buffers
	// glClearColor(0.9f, 0.9f, 0.9f, 0.9f);
	glClearColor(_background_color[0], _background_color[1], _background_color[2], _background_color[3]);  gl_check_error();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);  gl_check_error();


	// ==== BEGIN 3-D Stuff

	// a generic perspective transform
	glMatrixMode(GL_PROJECTION);  gl_check_error();

	glLoadIdentity();  gl_check_error();
	gluPerspective(45, _aspect, _near, _far);  gl_check_error();

	// rotate geometry
	glMatrixMode(GL_MODELVIEW);  gl_check_error();

	glLoadIdentity();  gl_check_error();
	gluLookAt(0.0, 0.0, _view_distance, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  gl_check_error();

	// View offset controlled by arrow keys
	glTranslatef(-_center_offset[0], -_center_offset[1], -_center_offset[2]); gl_check_error();

	// update light direction
	glLightfv(GL_LIGHT0, GL_POSITION, _light_dir0.data()); gl_check_error();
	glLightfv(GL_LIGHT1, GL_POSITION, _light_dir1.data()); gl_check_error();

	Eigen::AngleAxis<float> aa(get_view_quaternion());
	glRotatef(aa.angle() / minimesh::numbers::pi * 180., aa.axis().x(), aa.axis().y(), aa.axis().z());  gl_check_error();

	// move object center to origin
	glTranslatef(-_obj_center[0], -_obj_center[1], -_obj_center[2]);  gl_check_error();

	// Draw trimesh faces
	glStencilFunc(GL_ALWAYS, 1, ~(0u));  gl_check_error();

	// Save the matrices and view port
	glGetDoublev(GL_MODELVIEW_MATRIX, _model_view_matrix.data()); gl_check_error();
	glGetDoublev(GL_PROJECTION_MATRIX, _projection_matrix.data()); gl_check_error();
	glGetIntegerv(GL_VIEWPORT, _view_port.data()); gl_check_error();


	if(_draw_faces)
	{

		Eigen::Vector3i vids;
		Eigen::Vector3f normal;
		Eigen::Vector3f vf0, vf1, vf2;
		Eigen::Vector4f vc0, vc1, vc2;

		if(_has_lighting)
		{
			glEnable(GL_LIGHTING); gl_check_error();
		}
		else
		{
			glDisable(GL_LIGHTING); gl_check_error();
		}

		// draw face geometry
		glEnable(GL_POLYGON_OFFSET_FILL); gl_check_error();
		gl_begin(GL_TRIANGLES); gl_check_error();
		for(unsigned tid = 0; tid < get_mesh_buffer().tri_conn.cols(); ++tid)
		{
			vids = get_mesh_buffer().tri_conn.col(tid);
			if(_mesh_buffer.vertex_colors.cols())
			{
				vc0 = _mesh_buffer.vertex_colors.col(tid * 3 + 0);
				vc1 = _mesh_buffer.vertex_colors.col(tid * 3 + 1);
				vc2 = _mesh_buffer.vertex_colors.col(tid * 3 + 2);
			}
			else
			{
				vc0 = vc1 = vc2 = _default_surface_color;
			}
			vf0 = get_mesh_buffer().vertices.col(vids[0]);
			vf1 = get_mesh_buffer().vertices.col(vids[1]);
			vf2 = get_mesh_buffer().vertices.col(vids[2]);
			normal = (vf1 - vf0).cross(vf2 - vf0).normalized();
      
			glNormal3fv(normal.data()); gl_check_error();
      
			glColor4fv(vc0.data()); gl_check_error();
			glVertex3fv(vf0.data()); gl_check_error();
      
			glColor4fv(vc1.data()); gl_check_error();
			glVertex3fv(vf1.data()); gl_check_error();
      
			glColor4fv(vc2.data()); gl_check_error();
			glVertex3fv(vf2.data()); gl_check_error();
		}
		gl_end(); gl_check_error();
	}
	glDisable(GL_POLYGON_OFFSET_FILL); gl_check_error();
	glDisable(GL_LIGHTING); gl_check_error();

	//glStencilFunc(GL_ALWAYS, 0, ~(0u));

	// Draw trimesh vertices
	if(_draw_vertices)
	{
		glPointSize(4); gl_check_error();
		glColor3f(0.0, 0.0, 1.0); gl_check_error();
		gl_begin(GL_POINTS); gl_check_error();
		for(unsigned vid = 0; vid < get_mesh_buffer().vertices.cols(); ++vid)
		{
			Eigen::Vector3f vf = get_mesh_buffer().vertices.col(vid);
			glVertex3fv(vf.data()); gl_check_error();
		}
		gl_end(); gl_check_error();
	}

	// Draw the selected vertex
	if((_selected_vertex_mesh_buffer_id != invalid_index) && (_mouse_function == MOUSE_SELECT))
	{
		// printf("kdsjf;alskfdajd \n");
		glPointSize(10); gl_check_error();
		glColor3f(1.0, 0.0, 0.0); gl_check_error();
		gl_begin(GL_POINTS); gl_check_error();
		Eigen::Vector3f vf = get_mesh_buffer().vertices.col(_selected_vertex_mesh_buffer_id);
		glVertex3fv(vf.data()); gl_check_error();
		gl_end(); gl_check_error();
	}

	// Draw trimesh edges
	if(_draw_edges)
	{
		glLineWidth(1); gl_check_error();
		glColor3f(0.0, 0.0, 0.0); gl_check_error();
		gl_begin(GL_LINES); gl_check_error();
		for(unsigned eid = 0; eid < get_mesh_buffer().edge_conn.cols(); ++eid)
		{
			Eigen::VectorXi vids = get_mesh_buffer().edge_conn.col(eid);
			Eigen::Vector3f v0, v1;
			v0 = get_mesh_buffer().vertices.col(vids[0]);
			v1 = get_mesh_buffer().vertices.col(vids[1]);
			glVertex3fv(v0.data()); gl_check_error();
			glVertex3fv(v1.data()); gl_check_error();
		}
		gl_end(); gl_check_error();
	}

	if(_draw_axis)
	{
		glPushAttrib(GL_LINE_BIT); gl_check_error();
		glLineWidth(4); gl_check_error();

		float axisLength = 1.;
		glDisable(GL_LIGHTING); gl_check_error();
		glEnable(GL_COLOR_MATERIAL); gl_check_error();

		gl_begin(GL_LINES); gl_check_error();
		for(int i = 0; i < 3; i++)
		{
			float color[3] = {0, 0, 0};
			color[i] = 1.0;
			glColor3fv(color); gl_check_error();

			float vertex[3] = {0, 0, 0};
			vertex[i] = axisLength;
			glVertex3fv(vertex); gl_check_error();
			glVertex3f(0, 0, 0); gl_check_error();
		}
		gl_end(); gl_check_error();

		glEnable(GL_LIGHTING); gl_check_error();
		glPopAttrib(); gl_check_error();
	}

	if(_is_pulling_in_progress)
	{
		glPointSize(10); gl_check_error();
		glColor3f(0, 0.9, 0.0); gl_check_error();
		gl_begin(GL_POINTS); gl_check_error();
		Eigen::Vector3f vv = _mesh_buffer.vertices.col(_pulled_vertex_buffer_id);
		glVertex3fv(vv.data()); gl_check_error();
		gl_end(); gl_check_error();
	}

	// Draw the spheres
	{
		glPointSize(15); gl_check_error();
		for(int i = 0; i < (int)_mesh_buffer.spheres_vertex_indices.size(); ++i)
		{
			Eigen::Vector3f vv = _mesh_buffer.vertices.col(_mesh_buffer.spheres_vertex_indices(i));
			gl_begin(GL_POINTS); gl_check_error();
			glColor4fv(_mesh_buffer.spheres_colors.col(i).data()); gl_check_error();
			glVertex3fv(vv.data()); gl_check_error();
			gl_end(); gl_check_error();
		}
	}

	// ==== BEGIN 2-D Stuff

	glMatrixMode(GL_PROJECTION); gl_check_error();
	glLoadIdentity(); gl_check_error();
	glMatrixMode(GL_MODELVIEW); gl_check_error();
	glLoadIdentity(); gl_check_error();

	if(_is_pulling_in_progress)
	{
		Eigen::Vector2i point0 = freeglut2gl_screen(_pull_initial_pos, _view_port);
		Eigen::Vector2i point1 = freeglut2gl_screen(_pull_current_pos, _view_port);
    
		Eigen::Matrix4d eye = Eigen::Matrix4d::Identity();
		Eigen::Vector3f point3 = screen2world_generic(point0.x(), point0.y(), 0, eye, eye, _view_port).cast<float>();
		Eigen::Vector3f point4 = screen2world_generic(point1.x(), point1.y(), 0, eye, eye, _view_port).cast<float>();

		glLineWidth(5); gl_check_error();
		glColor3f(0, 0.9, 0.0); gl_check_error();
		gl_begin(GL_LINES); gl_check_error();
		glVertex3fv(point3.data()); gl_check_error();
		glVertex3fv(point4.data()); gl_check_error();
		gl_end(); gl_check_error();
	}

	// ======  Move this to the first buffer
	glutSwapBuffers(); gl_check_error();
}


bool Mesh_viewer::window_reshaped(const int w, const int h)
{
	_win_width = w;
	_win_height = h;
	glViewport(0, 0, w, h); gl_check_error();
	_aspect = float(w) / h;
	return true;
}


bool Mesh_viewer::mouse_pushed(const int button, const int state, const int x, const int y)
{
	bool should_redraw = false;
	record_mouse_pos(x, y, button);

	if(state == GLUT_UP)
	{
		_mouse_button = invalid_index;
		if(_is_pulling_in_progress)
		{
			_is_pulling_in_progress = false;
			should_redraw = true;
		}
		else
		{
			should_redraw = false;
		}
	}
	else
	{
		switch(button)
		{
			case 4:
			{
				_view_distance += 0.2f * std::abs(_view_distance);
				if(_view_distance < 0.0)
					_view_distance = 0.0;
				_mouse_button = invalid_index;
				should_redraw = true;
				break;
			} // end of button 4 -- wheel
			case 3:
			{
				_view_distance -= 0.2f * std::abs(_view_distance);
				if(_view_distance < 0.0)
					_view_distance = 0.0;
				_mouse_button = invalid_index;
				should_redraw = true;
				break;
			} // end of button 3 -- wheel
			case 2:
			{
				should_redraw = false;
				break;
			} // end of button 2 -- right click
			case 0:
			{
				switch(_mouse_function)
				{
					case MOUSE_VIEW:
					{
						should_redraw = false;
						break;
					}
					case MOUSE_SELECT:
					{
						_selected_vertex_mesh_buffer_id = screen2vertex(x, y);
						_has_user_just_selected = true;
						should_redraw = true;
						break;
					}
					case MOUSE_MOVE_VERTEX:
					{
						_pulled_vertex_buffer_id = screen2vertex(x, y, &_pulled_vertex_depth);
						if(_pulled_vertex_buffer_id != invalid_index)
						{
							_pull_latest_querried_pos = _pull_current_pos = _pull_initial_pos = Eigen::Vector2i(x, y);
							_has_user_just_pulled = true;
							_is_pulling_in_progress = true;
							should_redraw = true;
						}
						else
						{
							should_redraw = false;
						}
						break;
					}
					default:;
				} // End of mouse funciton
				default:;
			} // End of button 0 -- left click
		} // End of switch(button)
	} // all done with MOUSE CLICKED
	return should_redraw;
} // All done


bool Mesh_viewer::mouse_moved(const int x, const int y)
{
	bool should_redraw = false;

	if(_mouse_button == 2)
	{
		_center_offset += Eigen::Vector3f(-(x - _xPos), +(y - _yPos), 0) * std::abs(_view_distance) / 500.;
		should_redraw = true;
	}
	else if(_mouse_button == 0)
	{
		switch(_mouse_function)
		{
			case MOUSE_VIEW: // Rotate geometry
			{
				Eigen::AngleAxis<float> aa_y((x - _xPos) / 250.0f, Eigen::Vector3f(0, 1, 0));
				Eigen::AngleAxis<float> aa_x((y - _yPos) / 250.0f, Eigen::Vector3f(1, 0, 0));
				_view_quaternion = (aa_y * aa_x * get_view_quaternion()).coeffs();
				should_redraw = true;
				break;
			}
			case MOUSE_SELECT:
			{
				// do nothing
				break;
			}
			case MOUSE_MOVE_VERTEX:
			{
				_pull_current_pos = Eigen::Vector2i(x, y);
				if(_is_pulling_in_progress)
				{
					_has_user_just_pulled = true;
					should_redraw = true;
				}
				else
				{
					should_redraw = false;
				}
				break;
			}
			default: printf("Mouse mode %d is not implemented yet. \n", _mouse_function);
		} // End of switch(mouse funciton)
	} // End of key number 0

	record_mouse_pos(x, y, _mouse_button);

	return should_redraw;
}


bool Mesh_viewer::keyboard_pressed(unsigned char c, int x, int y)
{
	bool should_redraw = false;

	switch(c)
	{
		case GLUT_KEY_LEFT:
			_center_offset += Eigen::Vector3f(-0.05, 0, 0)* std::abs(_view_distance);
			should_redraw = true;
			break;
		case GLUT_KEY_RIGHT:
			_center_offset += Eigen::Vector3f(0.05, 0, 0)* std::abs(_view_distance);
			should_redraw = true;
			break;
		case GLUT_KEY_UP:
			_center_offset += Eigen::Vector3f(0, 0.05, 0)* std::abs(_view_distance);
			should_redraw = true;
			break;
		case GLUT_KEY_DOWN:
			_center_offset += Eigen::Vector3f(0, -0.05, 0)* std::abs(_view_distance);
			should_redraw = true;
			break;
	}

	return should_redraw;
}


void Mesh_viewer::get_and_clear_vertex_selection(bool & was_selection_made, int & selected_vertex)
{
	if(_mouse_function == MOUSE_SELECT)
	{
		was_selection_made = _has_user_just_selected;
		if(_selected_vertex_mesh_buffer_id != invalid_index)
			selected_vertex = get_mesh_buffer().vertex_ids(_selected_vertex_mesh_buffer_id);
		else
			selected_vertex = _selected_vertex_mesh_buffer_id;
	}
	else
	{
		was_selection_made = false;
		selected_vertex = invalid_index;
	}

	// clear
	_has_user_just_selected = false;
}


void Mesh_viewer::get_and_clear_vertex_displacement(bool & was_displaced, Eigen::Vector3f & disp_amount, int & pulled_vertex)
{
	if(_is_pulling_in_progress)
	{
		was_displaced = _has_user_just_pulled;

		Eigen::Vector2i point0 = freeglut2gl_screen(_pull_latest_querried_pos, _view_port);
		Eigen::Vector2i point1 = freeglut2gl_screen(_pull_current_pos, _view_port);
    
		Eigen::Vector3f point3 = screen2world_generic(
			point0.x(), point0.y(), _pulled_vertex_depth, _model_view_matrix, _projection_matrix, _view_port)
                                 .cast<float>();
		Eigen::Vector3f point4 = screen2world_generic(
			point1.x(), point1.y(), _pulled_vertex_depth, _model_view_matrix, _projection_matrix, _view_port)
                                 .cast<float>();
		disp_amount = point4 - point3;

		force_assert(_pulled_vertex_buffer_id != invalid_index);
		pulled_vertex = get_mesh_buffer().vertex_ids(_pulled_vertex_buffer_id);
	}
	else
	{
		was_displaced = false;
		disp_amount = Eigen::Vector3f::Zero();
	}

	// clear
	_has_user_just_pulled = false;
	_pull_latest_querried_pos = _pull_current_pos;
}


Eigen::Quaternionf Mesh_viewer::get_view_quaternion()
{
	return Eigen::Quaternionf(_view_quaternion.data());
}


void Mesh_viewer::record_mouse_pos(int clickx, int clicky, int button)
{
	_xPos = clickx;
	_yPos = clicky;
	_mouse_button = button;
}


Eigen::Vector2i Mesh_viewer::freeglut2gl_screen(const Eigen::Vector2i & clickin, const Eigen::Vector4i & view_port)
{
	Eigen::Vector2i out;
	out.x() = clickin.x();
	out.y() = view_port[3] - 1 - clickin.y();

	return out;
}


Eigen::Vector3f Mesh_viewer::screen2world_given_depth(int clickx, int clicky, float depth)
{

	const Eigen::Vector2i gl_click = freeglut2gl_screen(Eigen::Vector2i(clickx, clicky), _view_port);

	// clang-format off
	return screen2world_generic(gl_click.x(), 
								gl_click.y(),
								double(depth),
								_model_view_matrix,
								_projection_matrix,
								_view_port).cast<float>();
	// clang-format on
}


void Mesh_viewer::screen2world_find_depth(
	int clickx,
    int clicky,
    bool & is_mesh_clicked,
    Eigen::Vector3f & world_xyzf,
    float * depth)
{
	const Eigen::Vector2i gl_click = freeglut2gl_screen(Eigen::Vector2i(clickx, clicky), _view_port);

	GLubyte stencil_value;
	glReadPixels(gl_click.x(), gl_click.y(), 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, &stencil_value); gl_check_error();

	float depth_value;
	glReadPixels(gl_click.x(), gl_click.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_value); gl_check_error();
	if(depth) *depth = depth_value;

	// clang-format off
	world_xyzf = screen2world_generic(gl_click.x(), 
									  gl_click.y(),
									  double(depth_value),
									  _model_view_matrix,
									  _projection_matrix,
									  _view_port).cast<float>();
	// clang-format on

	if(stencil_value)
	{
		is_mesh_clicked = true;
	}
	else
	{
		is_mesh_clicked = false;
	}
}


Eigen::Vector3d Mesh_viewer::screen2world_generic(
	int gl_clickx,
    int gl_clicky,
    double depth_value,
    const Eigen::Matrix4d & model_view_matrix,
    const Eigen::Matrix4d & projection_matrix,
    const Eigen::Vector4i & view_port)
{
	Eigen::Vector3d world_xyzd;
	gluUnProject(gl_clickx,
				 gl_clicky,
				 depth_value,
				 model_view_matrix.data(),
				 projection_matrix.data(),
				 view_port.data(),
				 &world_xyzd.x(),
				 &world_xyzd.y(),
				 &world_xyzd.z()); static_gl_check_error();
	return world_xyzd;
}


int Mesh_viewer::screen2vertex(int clickx, int clicky, float * depth)
{
	Eigen::Vector3f world_xyzf;
	bool is_mesh_clicked;
	int selection_vertex_buffer_id = invalid_index;

	screen2world_find_depth(clickx, clicky, is_mesh_clicked, world_xyzf, depth);

	// Brute force search
	if(is_mesh_clicked)
	{
		float closest_dist = 1e10;
		for(int i = 0; i < (int)_mesh_buffer.vertices.cols(); ++i)
		{
			const float current_dist = (_mesh_buffer.vertices.col(i) - world_xyzf).squaredNorm();
			if(current_dist < closest_dist)
			{
				selection_vertex_buffer_id = i;
				closest_dist = current_dist;
			}
		} // End of search
		// printf("Clicked on vertex: %d \n", get_selected_vertex());
	} // end of click on non-void
	else
	{
		// printf("Clicked on VOID, stencil value %d \n", (int)stencil_value);
	}

	return selection_vertex_buffer_id;
}


void Mesh_viewer::gl_check_error()
{
	if(!_is_in_gl_begin)
		static_gl_check_error();
}


void Mesh_viewer::static_gl_check_error()
{
	GLenum error_type = glGetError();

#define PROCESS_ERROR(ENUM) \
	case ENUM: force_assert_msg(error_type == GL_NO_ERROR, "GL Error: " << #ENUM); break

	switch(error_type)
	{
		PROCESS_ERROR(GL_NO_ERROR);
		PROCESS_ERROR(GL_INVALID_ENUM);
		PROCESS_ERROR(GL_INVALID_VALUE);
		PROCESS_ERROR(GL_INVALID_OPERATION);
		PROCESS_ERROR(GL_STACK_OVERFLOW);
		PROCESS_ERROR(GL_STACK_UNDERFLOW);
		PROCESS_ERROR(GL_OUT_OF_MEMORY);
	}

#undef PROCESS_ERROR
}


void Mesh_viewer::gl_begin(GLenum mode)
{
	_is_in_gl_begin = true;
	glBegin(mode);
}


void Mesh_viewer::gl_end()
{
	_is_in_gl_begin = false;
	glEnd();
}


} // End of minimesh

#include <minimesh/core/util/enable_irrelevant_warnings.hpp>
