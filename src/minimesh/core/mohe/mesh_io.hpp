#pragma once

//
// mesh_io.hpp
//
// Some functionality for ready manifold triangle meshes.
//
// Author: Shayan Hoshyari
//

#include <string>

#include <minimesh/core/mohe/mesh_connectivity.hpp>

namespace minimesh
{
namespace mohe
{

class Mesh_io
{
public:
	// Trivial constructor
	Mesh_io(Mesh_connectivity & mesh_in);

	// Get the underlying mesh
	Mesh_connectivity & grid() { return _m; }
	const Mesh_connectivity & grid() const { return _m; }

	//
	// Functions to read a mesh.
	//

	// Guess the input format
	void read_auto(const std::string &);

	// Read a manifold obj mesh. The mesh does not have to be
	// triangular.
	void read_obj_general(const std::string &);

	// Read the off format
	void read_off(const std::string &);


	//
	// Functions to write a OBJ mesh
	//
	void write_obj(const std::string &);

	//
	// Functions to write a VTK mesh
	//

	// Write a vtk mesh. Vtk files can be openned by Paraview.
	void write_vtk(const std::string &);

	// Write a vtk mesh. Vtk files can be openned by Paraview.
	void write_vtk(FILE *, Mesh_connectivity::Defragmentation_maps&);

	//
	// You can use the following functions with  write_vtk(FILE *) to write
	// a vtk mesh and also write data for every vertex or triangle.
	// For example
	// FILE* fl = fopen('myfile.vtk','w');
	// Mesh_connectivity::Defragmentation_maps defrag;
	// 
	// my_mesh.compute_defragmentation_maps(defrag);
	// my_mesh_io.write_vtk(fl, defrag);
	//
	// my_mesh_io.write_vtk_vert_header(fl);
	// my_mesh_io.write_vtk_data(fl, some_data_for_vertices, "some_name_for_data", false);
	// my_mesh_io.write_vtk_data(fl, some_other_data_for_vertices, "some_name_for_data", false);
	//
	// my_mesh_io.write_vtk_cell_header(fl);
	// my_mesh_io.write_vtk_data(fl, some_data_for_faces, "some_name_for_data", false);
	// my_mesh_io.write_vtk_data(fl, some_other_data_for_faces, "some_name_for_data", false);
	//
	// fclose(fl);
	//

	// Call this once, before you write data for vertices
	void write_vtk_vert_header(FILE *);

	// Call this once, before you write data for cells
	void write_vtk_cell_header(FILE *);

	// Write data for either faces or vertices (integer or double)
	// Input:
	//  fl, the file to write to
	//   data, the data vector
	//   name, a name for this data
	//   is_vector, is this data a vector or a scalar field.
	//   if is_vector=true, we must have data.size() == (num_active_vertices or faces)*3
	//   if is_vector=false, we must have data.size() == (num_active_vertices or faces)
	void write_vtk_data(FILE * fl, std::vector<int> & data, const std::string name, const bool is_vector = false);
	void write_vtk_data(FILE * fl, std::vector<double> & data, const std::string name, const bool is_vector = false);

	//
	// Print mesh connectivity for debugging
	//
	void print_info(FILE * fl = stdout);

private:
	// Open a file, generate error if file is not found.
	static FILE * open_file(const std::string & fname, const std::string & format, const std::string & mode);

	// pointer to the mesh that we are working on.
	Mesh_connectivity & _m;


	// These are only used for writing a .vtk file
	enum
	{
		VTK_MODE_CELL = 0,
		VTK_MODE_VERTEX = 1
	};
	bool _vtk_have_written_cell_data_header;
	bool _vtk_have_written_vert_data_header;
	int _vtk_writing_mode;
};


} // end of mohe
} // end of minimesh
