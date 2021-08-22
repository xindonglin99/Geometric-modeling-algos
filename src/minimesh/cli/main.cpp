//
// This is a bare executable that gets linked to the core library.
// You can use it to test your code, or start learning about the library.
//
// Here I have put an executable which reads a predefined mesh, flips a specific edge in that 
// mesh, and then writes it back to .vtk and .obj formats. Feel free to play with the example, change
// it, or move it to a completely different file.
//

#include <cstdio>

#include <minimesh/core/util/assert.hpp>
#include <minimesh/core/util/macros.hpp>

#include <minimesh/core/mohe/mesh_connectivity.hpp>
#include <minimesh/core/mohe/mesh_io.hpp>
#include <minimesh/core/mohe/mesh_modifier.hpp>

using namespace minimesh;

// ===================
// EXAMPLE UTILITY FUNCTIONS
// ===================

namespace 
{

//
// Create an example mesh file that we can read later.
//
void
write_example_mesh()
{
  FILE *fl = fopen("example_mesh.obj", "w");

  //
  //
  //  (6) ----- (7) --- (8) 
  //   |\      / \      /|
  //   | \    /   \    / |
  //   |  \  /     \  /  |
  //   |   (4)------(5)  |
  //   |   / \      / \  |
  //   |  /   \    /   \ |
  //   | /     \  /     \|
  //   (1)----(2)------(3)
  //

  // Write the vertex coordinates
  fprintf(fl, "v 0 0 0 \n");
  fprintf(fl, "v 1 0 0 \n");
  fprintf(fl, "v 2 0 0 \n");
  fprintf(fl, "v 0.5 0.5 0 \n");
  fprintf(fl, "v 1.5 0.5 0 \n");
  fprintf(fl, "v 0 1 0 \n");
  fprintf(fl, "v 1 1 0 \n");
  fprintf(fl, "v 2 1 0 \n");

  // Write the faces (vertices are index1-based in .obj format)
  fprintf(fl, "\n");
  fprintf(fl, "f 1 2 4 \n");
  fprintf(fl, "f 2 3 5 \n");
  fprintf(fl, "f 1 4 6 \n");
  fprintf(fl, "f 4 2 5 \n");
  fprintf(fl, "f 5 3 8 \n");
  fprintf(fl, "f 6 4 7 \n");
  fprintf(fl, "f 7 4 5 \n");
  fprintf(fl, "f 7 5 8 \n");

  fclose(fl);
}

} // end of anonymus namespace

int main(int argc, char **argv)
{
  // Create a mesh_connectivity and a mesh reader
  mohe::Mesh_connectivity mesh;
  mohe::Mesh_io io(mesh);
  mohe::Mesh_modifier modi(mesh);

  printf("=== MESH EDITTING EXAMPLE === \n");

  // Write the example mesh, the mesh is written with the 
  // name example_mesh.obj
  printf("writing example_mesh.obj \n");
  write_example_mesh();

  // Now read the example mesh
  printf("reading example_mesh.obj \n");
  io.read_obj_general("example_mesh.obj");

  // A lambda for checking the mesh sanity and writing it
  int writing_index = 0;
  auto check_sanity_and_write_mesh = [&io, &mesh, &writing_index]()
  {
    force_assert( mesh.check_sanity_slowly() );

    printf("writing out_%d.vtk and out_%d.obj \n", writing_index, writing_index);

    io.write_vtk( MINIMESH_STR("out_" << writing_index << ".vtk") );
    io.write_obj( MINIMESH_STR("out_" << writing_index << ".obj") );
    
    ++writing_index;
  };

  // Now check that the mesh is sane and write it  in both 
  // .vtk and .obj formats
  check_sanity_and_write_mesh();

  // Flip the edge between vertices 4 and 5 (the diagonal from lower right to upper left)
  // Note that the indices should become 0-index based rather than 1-index based.
  printf("flipping edge \n");
  modi.flip_edge( modi.get_halfedge_between_vertices(4-1, 5-1) );
  check_sanity_and_write_mesh();

  // Now flip back the edge again
  printf("flipping edge again ...\n");
  modi.flip_edge( modi.get_halfedge_between_vertices(2-1, 7-1) );
  check_sanity_and_write_mesh();

  return 0;
} // end of main()
