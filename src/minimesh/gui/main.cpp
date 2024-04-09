// From standard library
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <algorithm>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// core
#include <minimesh/core/mohe/mesh_connectivity.hpp>
#include <minimesh/core/mohe/mesh_io.hpp>
#include <minimesh/core/mohe/mesh_modifier.hpp>
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
mohe::Mesh_modifier modi(mesh);
//
int glut_main_window_id;
//
GLUI * glui;
//
int num_entities_to_simplify;
//
Eigen::Matrix3Xd displaced_vertex_positions;
bool is_visualizing = false;
//
int param_id = 0;
//
int is_deforming = 0;
}


// ======================================================
//              FREEGLUT CALL BACKS
// ======================================================
namespace freeglutcallback {

void
draw() {
  globalvars::viewer.draw();
}

void
window_reshaped(int w, int h) {
  bool should_redraw = false;
  should_redraw = should_redraw || globalvars::viewer.window_reshaped(w, h);

  if (should_redraw)
    glutPostRedisplay();
}

void
keyboard_pressed(unsigned char c, int x, int y) {
  bool should_redraw = false;
  should_redraw = should_redraw || globalvars::viewer.keyboard_pressed(c, x, y);

  if (should_redraw)
    glutPostRedisplay();
}

void
keyboard_arrows_pressed(int c, int x, int y) {
  bool should_redraw = false;
  should_redraw = should_redraw || globalvars::viewer.keyboard_pressed(c, x, y);

  if (should_redraw)
    glutPostRedisplay();
}

void
mouse_pushed(int button, int state, int x, int y) {
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
    if (did_user_click) {
      printf("User just clicked on vertex %d \n", clicked_on_vertex);
      globalvars::modi.set_anchor_vertex(clicked_on_vertex);
    }
  }

  if (should_redraw)
    glutPostRedisplay();
}

void
mouse_moved(int x, int y) {
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

    if (has_pull_performed) {
      force_assert(pulled_vert != Mesh_viewer::invalid_index);

      // Get current displacement and apply the change to the mesh renderer

      globalvars::displaced_vertex_positions.col(pulled_vert) += pull_amount.cast<double>();
      // If the mesh was defragmented, you should have done:
      // Mesh_connectivity::Defragmentation_maps defrag;
      // globalvars::mesh.compute_defragmention_maps(defrag);
      // displaced_vertex_positions.col(defrag.old2new_vertex[j]) += pull_amount.cast<double>();

      if (globalvars::is_deforming) {
        globalvars::displaced_vertex_positions = globalvars::modi.deform(pulled_vert, globalvars::displaced_vertex_positions.col(pulled_vert));
      }

      // update positions (only the viewer)
      globalvars::viewer.get_mesh_buffer().set_vertex_positions(globalvars::displaced_vertex_positions.cast<float>());

      // Must rerender now.
      should_redraw = true;
    }
  }

  if (should_redraw)
    glutPostRedisplay();
}

void
show_spheres_pressed(int) {
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

void
subdivide_pressed(int) {
  printf("Subdivide button was pressed \n");

  globalvars::modi.subdivide();
  mohe::Mesh_connectivity::Defragmentation_maps defrag;
  globalvars::mesh.compute_defragmention_maps(defrag);
  globalvars::viewer.get_mesh_buffer().rebuild(globalvars::mesh, defrag);
  glutPostRedisplay();
}

void
visualize_pressed(int) {
  printf("Visualize button was pressed to visualize the top %d entities \n", globalvars::num_entities_to_simplify);

  globalvars::is_visualizing = !globalvars::is_visualizing;
  Eigen::Matrix4Xf colors(4, globalvars::mesh.n_active_vertices());
  colors.setOnes();

  if (globalvars::is_visualizing) {
    std::vector<int> vert_ind = globalvars::modi.get_top_k_errors_edge_vertices(globalvars::num_entities_to_simplify);
    mohe::Mesh_connectivity::Defragmentation_maps defrag;
    globalvars::mesh.compute_defragmention_maps(defrag);
    if (!vert_ind.empty()) {
      for (int ind : vert_ind) {
        int defrag_vert_ind = defrag.old2new_vertices[ind];
        colors.col(defrag_vert_ind) << 1, 0, 0, 1;
      }
    }
  }

  globalvars::viewer.get_mesh_buffer().set_vertex_colors(colors);
  glutPostRedisplay();
}

void
simplify_pressed(int) {
  printf("Simplify button was pressed to remove %d entities \n", globalvars::num_entities_to_simplify);
  globalvars::modi.simplify(globalvars::num_entities_to_simplify);
  mohe::Mesh_connectivity::Defragmentation_maps defrag;
  globalvars::mesh.compute_defragmention_maps(defrag);
  globalvars::viewer.get_mesh_buffer().rebuild(globalvars::mesh, defrag);

  globalvars::is_visualizing = false;
  Eigen::Matrix4Xf colors(4, globalvars::mesh.n_active_vertices());
  colors.setOnes();
  globalvars::viewer.get_mesh_buffer().set_vertex_colors(colors);
  glutPostRedisplay();
}

void
parametrize(int)
{
  printf("Parametrize the current mesh.\n");
  if (globalvars::param_id == 0) {
    globalvars::modi.parametrize_tutte();
  } else {
    globalvars::modi.parametrize_LSCM();
  }

  mohe::Mesh_connectivity::Defragmentation_maps defrag;
  globalvars::mesh.compute_defragmention_maps(defrag);
  globalvars::viewer.get_mesh_buffer().rebuild(globalvars::mesh, defrag);
  glutPostRedisplay();

//  mohe::Mesh_io(globalvars::mesh).write_obj("C:/Users/Hans_/Documents/GitHub/CPSC524/mesh-open/man_out.obj");
}

}


int
main(int argc, char * argv[])
{
  // Remember current folder
  foldertools::pushd();

  // If no command line argument is specified, load a hardcoded mesh.
  // Useful when debugging with visual studio.
  // Change the hardcoded address to your needs.
  if(argc == 1)
  {
    foldertools::makeandsetdir("/Users/Hans/Documents/CPSC524/mesh");
    mohe::Mesh_io(globalvars::mesh).read_auto("cylinder.obj");
  }
  else // otherwise use the address specified in the command line
  {
    mohe::Mesh_io(globalvars::mesh).read_auto(argv[1]);
  }

  // Do simplify computation
  globalvars::modi.quadrics();

  // Calculate weights and initial Laplacian
  globalvars::modi.build_weights();
//  globalvars::modi.build_laplacian();

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
  GLUI_RadioGroup * radio_group_mouse_func =
      globalvars::glui->add_radiogroup_to_panel(panel_mouse_func, &globalvars::viewer.get_mouse_function());
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
  GLUI_Button * button_subdivide =
      globalvars::glui->add_button("Subdivide Loop", -1, freeglutcallback::subdivide_pressed);
  button_subdivide->set_w(200);

  //
  // Add simplify button and a spinner to read how many entities to remove
  //
  globalvars::num_entities_to_simplify = 0;
  GLUI_Spinner * spinner_simplify = globalvars::glui->add_spinner(
      "# of entities to simplify", GLUI_SPINNER_INT, &globalvars::num_entities_to_simplify);
  spinner_simplify->set_alignment(GLUI_ALIGN_CENTER);
  spinner_simplify->set_w(300);

  GLUI_Button * button_simplify = globalvars::glui->add_button("Simplify", -1, freeglutcallback::simplify_pressed);
  button_simplify->set_w(200);

  //
  // Add visualize button
  //
  GLUI_Button * button_visualize = globalvars::glui->add_button("Visualize", -1, freeglutcallback::visualize_pressed);
  button_visualize->set_w(200);

  //
  // Add the parametrization panel and parametrization button
  //
  GLUI_Panel * panel_parametrization = globalvars::glui->add_panel("Parametrization Methods");
  GLUI_RadioGroup* parametrization_group = globalvars::glui->add_radiogroup_to_panel(
      panel_parametrization,
      &globalvars::param_id);
  globalvars::glui->add_radiobutton_to_group(parametrization_group, "Harmonic");
  globalvars::glui->add_radiobutton_to_group(parametrization_group, "LSCM");
  globalvars::glui->add_button("Parametrization", -1, freeglutcallback::parametrize);


  //
  // Add show spheres button to demo how to draw spheres on top of the vertices
  //
  globalvars::glui->add_button("Demo Showing Spheres", -1, freeglutcallback::show_spheres_pressed);


  //
  // Add the deformation checkbox
  //
  GLUI_Panel * panel_deform = globalvars::glui->add_panel("Deformation");
  globalvars::glui->add_checkbox_to_panel(panel_deform, "Deform", &globalvars::is_deforming);

  //
  // Save the initial vertex positions
  //
  globalvars::displaced_vertex_positions.resize(3, globalvars::mesh.n_active_vertices());
  for(int i = 0; i < globalvars::mesh.n_active_vertices(); ++i)
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
