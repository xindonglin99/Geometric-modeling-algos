// From standard library
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// core
#include <minimesh/core/mohe/mesh_connectivity.hpp>
#include <minimesh/core/mohe/mesh_io.hpp>
#include <minimesh/core/util/assert.hpp>
#include <minimesh/core/util/foldertools.hpp>
#include <minimesh/core/util/numbers.hpp>

// gui
#include <minimesh/viz/mesh_viewer.hpp>
#include <minimesh/viz/opengl_headers.hpp>


using namespace minimesh;

// ======================================================
// Global variables
// ======================================================
namespace globalvars
{
Mesh_viewer viewer;
mohe::Mesh_connectivity mesh;
//
int glut_main_window_id;
//
GLUI * glui;
//
int num_entities_to_simplify;
//
Eigen::Matrix3Xd displaced_vertex_positions;
}


// ======================================================
//              FREEGLUT CALL BACKS
// ======================================================
namespace freeglutcallback
{

void draw()
{
	globalvars::viewer.draw();
}


void window_reshaped(int w, int h)
{
	bool should_redraw = false;
	should_redraw = should_redraw || globalvars::viewer.window_reshaped(w, h);

	if(should_redraw)
		glutPostRedisplay();
}


void keyboard_pressed(unsigned char c, int x, int y)
{
	bool should_redraw = false;
	should_redraw = should_redraw || globalvars::viewer.keyboard_pressed(c, x, y);

	if(should_redraw)
		glutPostRedisplay();
}


void keyboard_arrows_pressed(int c, int x, int y)
{
	bool should_redraw = false;
	should_redraw = should_redraw || globalvars::viewer.keyboard_pressed(c, x, y);

	if(should_redraw)
		glutPostRedisplay();
}


void mouse_pushed(int button, int state, int x, int y)
{
	bool should_redraw = false;
	should_redraw = should_redraw || globalvars::viewer.mouse_pushed(button, state, x, y);

	//
	// NOTE: Sample of using Mesh_viewer for MESH DEFORMATION ASSINGMENT
	// Here is an example of how to use the selection feedback
	//
	{
		int clicked_on_vertex;
		bool did_user_click;
		globalvars::viewer.get_and_clear_vertex_selection(did_user_click, clicked_on_vertex);
		if(did_user_click)
			printf("User just clicked on vertex %d \n", clicked_on_vertex);
	}

	if(should_redraw)
		glutPostRedisplay();
}


void mouse_moved(int x, int y)
{
	bool should_redraw = false;
	should_redraw = should_redraw || globalvars::viewer.mouse_moved(x, y);


	//
	// NOTE: Sample of using Mesh_viewer for MESH DEFORMATION ASSINGMENT
	// Here is an example of how to use the output from the viewer
	// If the user is displacing, I will displace the vertex being pulled
	//
	{
		bool has_pull_performed;
		Eigen::Vector3f pull_amount;
		int pulled_vert;
		globalvars::viewer.get_and_clear_vertex_displacement(has_pull_performed, pull_amount, pulled_vert);

		if(has_pull_performed)
		{
			force_assert(pulled_vert != Mesh_viewer::invalid_index);

			// Get current displacement and apply the change to the mesh renderer

			globalvars::displaced_vertex_positions.col(pulled_vert) += pull_amount.cast<double>();
			// If the mesh was defragmented, you should have done:
			// Mesh_connectivity::Defragmentation_maps defrag;
			// globalvars::mesh.compute_defragmention_maps(defrag);
			// displaced_vertex_positions.col(defrag.old2new_vertex[j]) += pull_amount.cast<double>();

			// update positions (only the viewer)
			globalvars::viewer.get_mesh_buffer().set_vertex_positions(globalvars::displaced_vertex_positions.cast<float>());

			// Must rerender now.
			should_redraw = true;
		}
	}

	if(should_redraw)
		glutPostRedisplay();
}


void subdivide_pressed(int)
{
	printf("Subdivide button was pressed \n");
}


void simplify_pressed(int)
{
	printf("Simplify button was pressed to remove %d entities \n", globalvars::num_entities_to_simplify);
}


void show_spheres_pressed(int)
{
	// 
	// Sample of using Mesh_viewer for MESH DEFORMATION ASSIGNMENT
	// Here I color the vertices (draw spheres on them)
	// Note that if you call rebuild, you have to redraw everything.
	//
	Eigen::VectorXi sphere_indices(3);
	sphere_indices << 0, 1, 2;
	Eigen::Matrix4Xf sphere_colors(4, 3);
	sphere_colors.col(0) << 1, 1, 0, 1;
	sphere_colors.col(1) << 0, 1, 1, 1;
	sphere_colors.col(2) << 0, 0, 1, 1;

	globalvars::viewer.get_mesh_buffer().set_colorful_spheres(sphere_indices, sphere_colors);

	glutPostRedisplay();
}

}


int main(int argc, char * argv[])
{
	// Remember current folder
	foldertools::pushd();

	// If no command line argument is specified, load a hardcoded mesh.
	// Useful when debugging with visual studio.
	// Change the hardcoded address to your needs.
	if(argc == 1)
	{
		foldertools::makeandsetdir("D:/UBC/TA/2021Fall-524/code/CPSC524/mesh");
		mohe::Mesh_io(globalvars::mesh).read_auto("camel.obj");
	}
	else // otherwise use the address specified in the command line
	{
		mohe::Mesh_io(globalvars::mesh).read_auto(argv[1]);
	}

	// Initialize GLUT window
	glutInit(&argc, argv);
	glutInitWindowSize(800, 600);
	glutInitDisplayMode(GLUT_STENCIL | GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);
	globalvars::glut_main_window_id = glutCreateWindow("Mesh Viewer");

	// Initialize GLUI window for buttons and ...
	globalvars::glui = GLUI_Master.create_glui("Controls");
	globalvars::glui->set_main_gfx_window(globalvars::glut_main_window_id);

	// Register callbacks
	glutDisplayFunc(freeglutcallback::draw);
	GLUI_Master.set_glutReshapeFunc(freeglutcallback::window_reshaped);
	GLUI_Master.set_glutKeyboardFunc(freeglutcallback::keyboard_pressed);
	GLUI_Master.set_glutSpecialFunc(freeglutcallback::keyboard_arrows_pressed);
	GLUI_Master.set_glutMouseFunc(freeglutcallback::mouse_pushed);
	glutMotionFunc(freeglutcallback::mouse_moved);
	GLUI_Master.set_glutIdleFunc(NULL);

	// Initialize the viewer (it needs the bounding box of the mesh)
	Eigen::AlignedBox3f bbox;
	for(int v = 0; v < globalvars::mesh.n_total_vertices(); ++v)
	{
		mohe::Mesh_connectivity::Vertex_iterator vertex = globalvars::mesh.vertex_at(v);
		if(vertex.is_active())
		{
			bbox.extend(vertex.xyz().cast<float>());
		}
	}
	globalvars::viewer.initialize(bbox);

	// Load the mesh in the viewer
	{
		mohe::Mesh_connectivity::Defragmentation_maps defrag;
		globalvars::mesh.compute_defragmention_maps(defrag);
		globalvars::viewer.get_mesh_buffer().rebuild(globalvars::mesh, defrag);
	}

	//
	// Add radio buttons to see which mesh components to view
	// Please view GLUI's user manual to learn more.
	//

	GLUI_Panel * panel_view = globalvars::glui->add_panel("View mesh components");
	globalvars::glui->add_checkbox_to_panel(panel_view, "Show vertices", &globalvars::viewer.get_draw_vertices());
	globalvars::glui->add_checkbox_to_panel(panel_view, "Show edges", &globalvars::viewer.get_draw_edges());
	globalvars::glui->add_checkbox_to_panel(panel_view, "Show faces", &globalvars::viewer.get_draw_faces());
	globalvars::glui->add_checkbox_to_panel(panel_view, "Show axis", &globalvars::viewer.get_draw_axis());
	globalvars::glui->add_checkbox_to_panel(panel_view, "Show lighting", &globalvars::viewer.get_has_lighting());

	//
	// Add radio buttons to determine mouse left click functionality
	//
	GLUI_Panel * panel_mouse_func = globalvars::glui->add_panel("Mouse functionality");
	GLUI_RadioGroup * radio_group_mouse_func =   globalvars::glui->add_radiogroup_to_panel(panel_mouse_func, &globalvars::viewer.get_mouse_function());
	for(int i = 0; i < Mesh_viewer::MOUSE_INVALID; ++i)
	{
		if(i == Mesh_viewer::MOUSE_VIEW)
			globalvars::glui->add_radiobutton_to_group(radio_group_mouse_func, "Pan and zoom");
		if(i == Mesh_viewer::MOUSE_SELECT)
			globalvars::glui->add_radiobutton_to_group(radio_group_mouse_func, "Select vertex");
		if(i == Mesh_viewer::MOUSE_MOVE_VERTEX)
			globalvars::glui->add_radiobutton_to_group(radio_group_mouse_func, "Move vertex");
	}

	//
	// Add subdivide button
	//
	GLUI_Button* button_subdivide =  globalvars::glui->add_button("Subdivide Loop", -1, freeglutcallback::subdivide_pressed);
	button_subdivide->set_w(200);

	//
	// Add simplify button and a spinner to read how many entities to remove
	//
	globalvars::num_entities_to_simplify = 0;
	GLUI_Spinner* spinner_simplify = globalvars::glui->add_spinner("# of entities to simplify", GLUI_SPINNER_INT, &globalvars::num_entities_to_simplify);
	spinner_simplify->set_alignment(GLUI_ALIGN_CENTER);
	spinner_simplify->set_w(300);
	
	GLUI_Button* button_simplify = globalvars::glui->add_button("Simplify", -1, freeglutcallback::simplify_pressed);
	button_simplify->set_w(200);

	//
	// Add show spheres button to demo how to draw spheres on top of the vertices
	//
	globalvars::glui->add_button("Demo Showing Spheres", -1, freeglutcallback::show_spheres_pressed);

	//
	// Save the initial vertex positions
	//
	globalvars::displaced_vertex_positions.resize(3,globalvars::mesh.n_active_vertices() );
	for (int i = 0 ; i < globalvars::mesh.n_active_vertices() ; ++i)
	{
		globalvars::displaced_vertex_positions.col(i) = globalvars::mesh.vertex_at(i).xyz();
	}

	// Sync all glui variables
	globalvars::glui->sync_live();

	// Start main loop
	glutPostRedisplay(); // Draw everything again just for caution.
	glutMainLoop();

	// revert back to initial folder
	foldertools::popd();

	return 0;
}
