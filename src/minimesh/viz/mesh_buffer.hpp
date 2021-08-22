#pragma once

#include <Eigen/Core>


namespace minimesh
{

namespace mohe
{
class Mesh_connectivity;
class Mesh_connectivity_defragmentation_maps;
}

// An interim between a mesh and the viewer. To view any mesh, it should be converted to
// a mesh buffer first.
class Mesh_buffer
{
public:
	// Rebuild the mesh buffer from a mesh.
	// It needs both the mesh and a defragmentation map for the mesh.
	void rebuild(mohe::Mesh_connectivity &, mohe::Mesh_connectivity_defragmentation_maps & defrag);

	// Color the surface of the mesh, by interpolating colors at vertices of the mesh.
	// Size of colors should be 4[RGBA] * n_active_vertices().
	// The color of the jth vertex should be in the defrag.old2new_vertex[ j ]th column of colors.
	// call this after rebuild.
	void set_vertex_colors(const Eigen::Matrix4Xf & colors);

	// Color the faces of the mesh.
	// Size of colors should be 4[RGBA] * n_active_faces().
	// The color of the jth face should be in the defrag.old2new_face[ j ]th column of colors.
	// call this after rebuild.
	void set_face_colors(const Eigen::Matrix4Xf & colors);

	// Change the positions of the vertices without rebuilding the mesh
	// The position of the jth vertex should be in the defrag.old2new_face[ j ]th column of positions.
	// Note that if the mesh is not defragmented, then defrag.old2new_face[ j ] = j .
	void set_vertex_positions(const Eigen::Matrix3Xf & positions);

	//
	// Adds sphere on top of vertices in the mesh and colors them
	// vertex_indices: Index of vertices to draw a sphere on top. Indices are continuous
	//                 Again, recall that the cont. index of vertex j is defrag.old2new_face[ j ]
	// colors:         Colors for each sphere 4 (RGBA) * number_of_spheres
	void set_colorful_spheres(const Eigen::VectorXi & vertex_indices, const Eigen::Matrix4Xf & colors);

private:
	// The vertices of the mesh
	Eigen::Matrix3Xf vertices;

	// Ids of vertices in the actual mesh (not defragmented)
	Eigen::VectorXi vertex_ids;

	// The connectivity for every triangle
	Eigen::Matrix3Xi tri_conn;

	// The connectivity for every edge
	Eigen::Matrix2Xi edge_conn;

	// Surface colors interpolated at vertices
	Eigen::Matrix4Xf vertex_colors;

	// Colorful balls vertex indices
	Eigen::VectorXi spheres_vertex_indices;
	Eigen::Matrix4Xf spheres_colors;
	
	friend class Mesh_viewer;
};


} // end of minimesh

