#pragma once

//
// mesh_modifier.hpp
//
// Some functionality for modifying meshes.
//
// Author: Shayan Hoshyari
//


#include <string>
#include <vector>
#include <map>
#include <minimesh/core/mohe/mesh_connectivity.hpp>

namespace minimesh
{
namespace mohe
{


class Mesh_modifier
{
public:
	// Trivial constructor
	Mesh_modifier(Mesh_connectivity & mesh_in): _m(mesh_in) {}

	// Get the underlying mesh
	Mesh_connectivity & mesh() { return _m; }
	const Mesh_connectivity & mesh() const { return _m; }

	//
	// Given two vertices, this function return the index of the half-edge going from v0 to v1.
	// Returns mesh::invalid_index if no half-edge exists between the two vertices.
	//
	int get_halfedge_between_vertices(const int v0, const int v1);

	//
	// Flip an edge in a mesh
	// Input: The mesh, and the index of a half-edge on the edge we wish to flip
	// Return true if operation successful, and false if operation not possible
	//
	// Assumption: mesh is all triangles
	//
	// NOTE: To see how this method works, take a look at edge-flip.svg
	//
	bool flip_edge(const int he_index);

    // Subdivide a closed manifold mesh using Loop scheme
    void subdivide();

    typedef std::map<std::pair<int, int>, int> Edge_map;

private:
	// pointer to the mesh that we are working on.
	Mesh_connectivity & _m;
  
  // Compute coefficient for even vertices
  static float compute_old_coeff(const int n);
  void build_triangle(
          Mesh_connectivity::Vertex_iterator & v0,
          Mesh_connectivity::Vertex_iterator & v1,
          Mesh_connectivity::Vertex_iterator & v2,
          Mesh_connectivity & mesh,
          Edge_map & edge_map);
};


} // end of mohe
} // end of minimesh
