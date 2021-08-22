#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>

#include <minimesh/core/mohe/mesh_io.hpp>
#include <minimesh/core/util/assert.hpp>
#include <minimesh/core/util/macros.hpp>

namespace minimesh
{
namespace mohe
{

FILE* Mesh_io::open_file(const std::string & fname, const std::string & format, const std::string & mode)
{
	// open the file
	FILE * fl;
	std::string final_fname;

	if(fname.find(format) == fname.length() - format.length())
	{
		final_fname = fname;
	}
	else
	{
		final_fname = fname + format;
	}

	fl = fopen(final_fname.c_str(), mode.c_str());
	if(!fl)
	{
		std::cout << "[Error] Could not find the file " << final_fname << "." << std::endl;
		assert(0);
		throw "Bad input file";
	}

	return fl;
}


Mesh_io::Mesh_io(Mesh_connectivity & mesh_in)
: _m(mesh_in)
, _vtk_have_written_cell_data_header(false)
, _vtk_have_written_vert_data_header(false)
, _vtk_writing_mode(-1)
{
}


void Mesh_io::read_auto(const std::string & fname)
{
	auto has_extension = [](const std::string & f, const std::string & e) -> bool {
		return f.find(e) == f.length() - e.length();
	};

	if(has_extension(fname, ".obj"))
	{
		this->read_obj_general(fname);
	}
	else if(has_extension(fname, ".off"))
	{
		this->read_off(fname);
	}
	else
	{
		std::cout << "Unrecognized format " << fname << std::endl;
		throw "Unsupported input file format";
	}
}


void Mesh_io::read_obj_general(const std::string & fname)
{
	// open the file
	FILE * fl = open_file(fname, ".obj", "r");

	//
	//  read one face
	//
	auto read_face = [](char * line) -> std::vector<int> {
		std::vector<int> answer;
		int current_pos = 0;

		//
		// read one vertex for face
		//
		auto read_vertex = [&line, &current_pos, &answer]() -> bool {
			const int vertex_begins_at = current_pos;

			//
			// read a pattern per vertex
			//
			auto read_pattern = [&line, &current_pos, &answer, &vertex_begins_at](const char * pattern, int n_expected_read) -> bool {
				int vertex_index(-1), tmp1(-1), tmp2(-1), n_chars_read(-1);
				int n_read;
				if(n_expected_read == 3)
					n_read = sscanf(line + current_pos, pattern, &vertex_index, &tmp1, &tmp2, &n_chars_read);
				if(n_expected_read == 2)
					n_read = sscanf(line + current_pos, pattern, &vertex_index, &tmp1, &n_chars_read);
				if(n_expected_read == 1)
					n_read = sscanf(line + current_pos, pattern, &vertex_index, &n_chars_read);
				if( (n_read == n_expected_read) && (n_chars_read >= 0) )
				{
					answer.push_back(vertex_index);
					current_pos += n_chars_read;
					return true;
				}
				else
				{
					current_pos = vertex_begins_at;
					return false;
				}
			};

			if( read_pattern("%d/%d/%d%n",3) ) return true;
			if( read_pattern("%d//%d%n",2) ) return true;
			if( read_pattern("%d/%d%n",2) ) return true;
			if( read_pattern("%d%n",1) ) return true;

			return false;
		}; // end of read vertex

		do
		{
		} while(read_vertex());

		return answer;
	}; // end of read face

	/*
	* read the file
	*/
	std::vector<double> vcoord;
	std::vector<int> face_verts_adj;
	std::vector<int> face_verts_xadj(1 /* len */, 0 /*value*/);
	std::vector<int> current_face;

	constexpr int max_line_size = 4096;
	char line[max_line_size];
	for(;;)
	{
		const int c = fgetc(fl);

		if(c == EOF)
		{
			break;
		}
		else if(c == (int)'v')
		{
			const int cnext = fgetc(fl);
			if(cnext == (int)' ')
			{
				double x, y, z;
				const int success = fscanf(fl, "%lf %lf %lf", &x, &y, &z);
				force_assert(success == 3);
				vcoord.push_back(x);
				vcoord.push_back(y);
				vcoord.push_back(z);
			}
		}
		else if(c == (int)'f')
		{
			// read the face data in a single line
			char * success = fgets(line, max_line_size, fl);
			force_assert(success == line);

			// read the vertices
			current_face = read_face(line);
			force_assert(current_face.size());

			// add them
			face_verts_xadj.push_back(face_verts_xadj.back() + (int)current_face.size());
			for(int vid : current_face)
				face_verts_adj.push_back(vid - 1); // one indexed vertices! be carefull.
		}
		else if((c == (int)'\n') || (c == (int)' '))
		{
			continue;
		}
		else // if ( c == (int)'#' )
		{
			char * success = fgets(line, max_line_size, fl);
			MINIMESH_UNUSED( success );
			force_assert(success == line);
		}
	}

	// close the file
	fclose(fl);

	// Remove the unused vertices
	const int n_uverts = (int)vcoord.size();
	std::vector<bool> is_vert_used(n_uverts, false);

	for(int face_v = 0; face_v < int(face_verts_adj.size()); ++face_v)
	{
		is_vert_used[face_verts_adj[face_v]] = true;
	}

	std::vector<int> uvert_to_vert(n_uverts);
	int n_verts = 0;
	for(int uv = 0; uv < n_uverts; uv++)
	{
		if(is_vert_used[uv])
		{
			uvert_to_vert[uv] = n_verts;
			++n_verts;
		}
		else
		{
			uvert_to_vert[uv] = -1;
		}
	}

	for(int i = 0; i < int(face_verts_adj.size()); i++)
	{
		// this are guarranteed not to be -1
		face_verts_adj[i] = uvert_to_vert[face_verts_adj[i]];
	}

	std::vector<double> vc2(n_verts * 3, -1);
	for(int i = 0; i < n_uverts; i++)
	{
		if(is_vert_used[i])
		{
			vc2[uvert_to_vert[i] * 3 + 0] = vcoord[i * 3 + 0];
			vc2[uvert_to_vert[i] * 3 + 1] = vcoord[i * 3 + 1];
			vc2[uvert_to_vert[i] * 3 + 2] = vcoord[i * 3 + 2];
		}
	}

	// Pass stuff to the grid
	grid().build_from_polygons(vc2, face_verts_adj, face_verts_xadj);
}


void Mesh_io::read_off(const std::string & fname)
{
	// open the file
	FILE * fl = open_file(fname, ".off", "r");

	auto skip_comment = [&fl]() {
		force_assert(fl);

		for(;;)
		{
			int c;
			c = fgetc(fl);

			if(c == EOF)
			{
				assert(0 && "Unexpected EOF");
				break;
			}
			else if(((c >= (int)'0') && (c <= (int)'9')) || (c == (int)'.') || (c == (int)'-'))
			{
				int success = fseek(fl, -1, SEEK_CUR);
				assert(success == 0);
				MINIMESH_UNUSED( success );
				break;
			}
		}
	};

	/*
	* read the file
	*/
	int n_verts, n_faces;
	std::vector<double> vcoord;
	std::vector<int> ftovert;
	int success;

	// Read the number of verts and faces and edges
	skip_comment();
	success = fscanf(fl, "%d %d %*d", &n_verts, &n_faces);
	assert(success = 2);
	MINIMESH_UNUSED( success );
	vcoord.reserve(n_verts * 3);
	ftovert.reserve(n_faces * 3);

	// Read vertex coordinates
	for(int v = 0; v < n_verts; v++)
	{
		skip_comment();
		double x, y, z;
		success = fscanf(fl, "%lf %lf %lf", &x, &y, &z);
		assert(success == 3);
		vcoord.push_back(x);
		vcoord.push_back(y);
		vcoord.push_back(z);
	}

	// Read face vertices
	for(int f = 0; f < n_faces; f++)
	{
		skip_comment();
		int nv, v0, v1, v2;
		success = fscanf(fl, "%d %d %d %d", &nv, &v0, &v1, &v2);
		assert(success == 4);
		assert(nv == 3 && "Only triangular faces are supported");
		ftovert.push_back(v0);
		ftovert.push_back(v1);
		ftovert.push_back(v2);
	}

	// close the file
	fclose(fl);

	// init the mesh
	grid().build_from_triangles(vcoord, ftovert);
}


void Mesh_io::write_obj(const std::string & fname)
{
	FILE * fl = open_file(fname, ".obj", "w");
	Mesh_connectivity::Defragmentation_maps defrag;
	grid().compute_defragmention_maps(defrag);

	// Write the vertices
	for(int vnidx = 0; vnidx < grid().n_active_vertices(); vnidx++)
	{
		int voidx = defrag.new2old_vertices[vnidx];
		Mesh_connectivity::Vertex_iterator vert = grid().vertex_at(voidx);
		fprintf(fl, "v %.12g %.12g %.12g \n", vert.xyz().x(), vert.xyz().y(), vert.xyz().z());
	}
	fprintf(fl, "\n");

	// Write the face connectivity (note that for obj vertices are index1-based
	for(int fn = 0; fn < grid().n_active_faces(); ++fn)
	{
		int fo = defrag.new2old_faces[fn];
		Mesh_connectivity::Face_iterator face = grid().face_at(fo);
		Mesh_connectivity::Half_edge_iterator he_end = face.half_edge();
		Mesh_connectivity::Half_edge_iterator he = face.half_edge();

		fprintf(fl, "f ");
		do
		{
		const int vert_old_index = he.origin().index();
		const int vert_new_index = defrag.old2new_vertices[vert_old_index];
		fprintf(fl, "%d ", vert_new_index + 1); // index1-based
		he = he.next();
		} while(!he.is_equal(he_end));
		fprintf(fl, "\n");
	}
	fprintf(fl, "\n");

	fclose(fl);
}


void Mesh_io::write_vtk(const std::string & fname)
{
	FILE * fl = open_file(fname, ".vtk", "w");
	Mesh_connectivity::Defragmentation_maps defrag;
	grid().compute_defragmention_maps(defrag);
	this->write_vtk(fl, defrag);
	fclose(fl);
}


void Mesh_io::write_vtk(FILE * fl, Mesh_connectivity::Defragmentation_maps &defrag)
{
	force_assert_msg(fl, "FILE should be open");

	/*
	* Write the vtk file.
	*/

	// write the header
	fprintf(fl, "# vtk DataFile Version 2.0\n");
	fprintf(fl, "Shayan's output mesh\n");
	fprintf(fl, "ASCII\n");
	fprintf(fl, "DATASET UNSTRUCTURED_GRID\n");
	fprintf(fl, "\n");

	// write the vertices
	fprintf(fl, "POINTS %d float\n", grid().n_active_vertices());
	for(int vnidx = 0; vnidx < grid().n_active_vertices(); vnidx++)
	{
		int voidx = defrag.new2old_vertices[vnidx];
		Mesh_connectivity::Vertex_iterator vert = grid().vertex_at(voidx);
		fprintf(fl, "%e %e %e \n", vert.xyz().x(), vert.xyz().y(), vert.xyz().z());
	}
	fprintf(fl, "\n");

	//
	// write the faces
	//

	// count their total number of vertices.
	int total_vert_duplicated_per_face = 0;
	for(int fn = 0; fn < grid().n_active_faces(); ++fn)
	{
		int fo = defrag.new2old_faces[fn];
		Mesh_connectivity::Face_iterator face = grid().face_at(fo);
		Mesh_connectivity::Half_edge_iterator he_end = face.half_edge();
		Mesh_connectivity::Half_edge_iterator he = face.half_edge();
		do
		{
			++total_vert_duplicated_per_face;
			he = he.next();
		} while(!he.is_equal(he_end));
	}

	int face_verts_cache[100];
	fprintf(fl, "CELLS %d %d \n", grid().n_active_faces(), grid().n_active_faces() + total_vert_duplicated_per_face);
	for(int fn = 0; fn < grid().n_active_faces(); ++fn)
	{
		int fo = defrag.new2old_faces[fn];
		Mesh_connectivity::Face_iterator face = grid().face_at(fo);
		int n_verts = 0;
		Mesh_connectivity::Half_edge_iterator he_end = face.half_edge();
		Mesh_connectivity::Half_edge_iterator he = face.half_edge();
		do
		{
			face_verts_cache[n_verts] = defrag.old2new_vertices[he.origin().index()];
			++n_verts;
			he = he.next();
		} while(!he.is_equal(he_end));

		fprintf(fl, "%d ", n_verts);
		for(int voffset = 0; voffset < n_verts; ++voffset)
		{
			fprintf(fl, "%d ", face_verts_cache[voffset]);
		}
		fprintf(fl, "\n");
	}
	fprintf(fl, "\n");

	// write the face types
	fprintf(fl, "CELL_TYPES %d \n", grid().n_active_faces());
	for(int f = 0; f < grid().n_active_faces(); f++)
	{
		fprintf(fl, "7 \n"); // VTK POLYGON
	}
	fprintf(fl, "\n");

	_vtk_have_written_cell_data_header = _vtk_have_written_vert_data_header = false;
}


void Mesh_io::write_vtk_vert_header(FILE * fl)
{
	if(!_vtk_have_written_vert_data_header)
	{
		fprintf(fl, "POINT_DATA %d \n", grid().n_active_vertices());
		_vtk_have_written_vert_data_header = true;
		_vtk_writing_mode = VTK_MODE_VERTEX;
	}
}


void Mesh_io::write_vtk_cell_header(FILE * fl)
{
	if(!_vtk_have_written_cell_data_header)
	{
		fprintf(fl, "CELL_DATA %d \n", grid().n_active_faces());
		_vtk_have_written_cell_data_header = true;
		_vtk_writing_mode = VTK_MODE_CELL;
	}
}


void Mesh_io::write_vtk_data(FILE * fl, std::vector<int> & data, const std::string name, const bool is_vector)
{
	const int bs = (is_vector ? 3 : 1);
	MINIMESH_UNUSED( bs );
	force_assert(((_vtk_writing_mode == (int)VTK_MODE_CELL) && ((int)data.size() == grid().n_active_faces() * bs)) ||
			((_vtk_writing_mode == (int)VTK_MODE_VERTEX) && ((int)data.size() == grid().n_active_vertices() * bs)));

	if(!is_vector)
	{
		fprintf(fl, "SCALARS %s int 1 \n", name.c_str());
		fprintf(fl, "LOOKUP_TABLE default \n");

		for(int i = 0; i < (int)data.size(); i++)
		{
			fprintf(fl, "%d \n", data[i]);
		}
	}
	else
	{
		fprintf(fl, "SCALARS %s int 3 \n", name.c_str());
		fprintf(fl, "LOOKUP_TABLE default \n");

		for(int i = 0; i < (int)(data.size() / 3); i++)
		{
			fprintf(fl, "%d %d %d\n", data[3 * i + 0], data[3 * i + 1], data[3 * i + 2]);
		}
	}
	fprintf(fl, "\n");
}


void Mesh_io::write_vtk_data(FILE * fl, std::vector<double> & data, const std::string name, const bool is_vector)
{
	const int bs = (is_vector ? 3 : 1);
	MINIMESH_UNUSED( bs );
	force_assert(((_vtk_writing_mode == (int)VTK_MODE_CELL) && ((int)data.size() == grid().n_active_faces() * bs)) ||
			((_vtk_writing_mode == (int)VTK_MODE_VERTEX) && ((int)data.size() == grid().n_active_vertices() * bs)));

	if(!is_vector)
	{
		fprintf(fl, "SCALARS %s float 1 \n", name.c_str());
		fprintf(fl, "LOOKUP_TABLE default \n");

		for(int i = 0; i < (int)data.size(); i++)
		{
			fprintf(fl, "%e \n", data[i]);
		}
	}
	else
	{
		fprintf(fl, "SCALARS %s float 3 \n", name.c_str());
		fprintf(fl, "LOOKUP_TABLE default \n");

		assert(data.size() % 3 == 0);
		for(int i = 0; i < (int)(data.size() / 3); i++)
		{
			fprintf(fl, "%e %e %e\n", data[3 * i + 0], data[3 * i + 1], data[3 * i + 2]);
		}
	}
	fprintf(fl, "\n");
}


void Mesh_io::print_info(FILE * fl)
{
	assert(fl);

	fprintf(fl, "%8s %8s %8s %8s %8s %8s %8s\n", "#HE", "vbegin", "vend", "face", "twin", "next", "prev");

	for(int i = 0; i < grid().n_active_half_edges(); i++)
	{
		Mesh_connectivity::Half_edge_iterator he = grid().half_edge_at(i);
		const int twin = he.twin().index();
		const int vbeg = he.origin().index();
		const int vend = he.dest().index();
		const int face = he.face().index();
		const int next = he.next().index();
		const int prev = he.prev().index();

		fprintf(fl, "%8d %8d %8d %8d %8d %8d %8d\n", i, vbeg, vend, face, twin, next, prev);
	}
}

} // end of mohe
} // end of minimesh
