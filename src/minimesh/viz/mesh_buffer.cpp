#include <iostream>

#include <minimesh/core/util/assert.hpp>
#include <minimesh/core/mohe/mesh_connectivity.hpp>

#include <minimesh/viz/mesh_buffer.hpp>

namespace minimesh
{

void Mesh_buffer::rebuild(mohe::Mesh_connectivity & mesh, mohe::Mesh_connectivity_defragmentation_maps & defrag)
{
	tri_conn.resize(3, mesh.n_active_faces());
	edge_conn.resize(2, mesh.n_active_half_edges() / 2);
	vertices.resize(3, mesh.n_active_vertices());
	vertex_colors.resize(4, 0);
	vertex_ids.resize(mesh.n_active_vertices());
	spheres_vertex_indices.resize(0);
	spheres_colors.resize(4,0);

	for(int i = 0; i < mesh.n_active_faces(); ++i)
	{
		auto f = mesh.face_at(defrag.new2old_faces[i]);
		assert((f.n_vertices()) == 3 && "Viewer only works for triangular meshes");
		tri_conn.col(i) << defrag.old2new_vertices[f.half_edge().origin().index()],
						   defrag.old2new_vertices[f.half_edge().next().origin().index()],
						   defrag.old2new_vertices[f.half_edge().next().next().origin().index()];
	}

	for(int i = 0; i < mesh.n_active_vertices(); ++i)
	{
		auto v = mesh.vertex_at(defrag.new2old_vertices[i]);
		vertex_ids[i] = v.index();
		vertices.col(i) = v.xyz().cast<float>();
	}

	int hecntr = 0;
	for(int i = 0; i < mesh.n_active_half_edges(); ++i)
	{
		auto he = mesh.half_edge_at(defrag.new2old_half_edges[i]);
		if(he.index() > he.twin().index())
		{
			edge_conn.col(hecntr) << defrag.old2new_vertices[he.origin().index()], defrag.old2new_vertices[he.dest().index()];
			++hecntr;
		}
	}
	assert(hecntr == mesh.n_active_half_edges() / 2);
}


void Mesh_buffer::set_vertex_colors(const Eigen::Matrix4Xf & colors)
{
	force_assert(colors.cols() == vertices.cols());

	vertex_colors.resize(4, tri_conn.cols() * 3);
	for(int fid = 0; fid < (int)tri_conn.cols(); ++fid)
	{
		vertex_colors.col(fid * 3 + 0) = colors.col(tri_conn(0, fid));
		vertex_colors.col(fid * 3 + 1) = colors.col(tri_conn(1, fid));
		vertex_colors.col(fid * 3 + 2) = colors.col(tri_conn(2, fid));
	}
}


void Mesh_buffer::set_face_colors(const Eigen::Matrix4Xf & colors)
{
	force_assert(colors.cols() == tri_conn.cols());

	vertex_colors.resize(4, tri_conn.cols() * 3);
	for(int fid = 0; fid < (int)tri_conn.cols(); ++fid)
	{
		vertex_colors.col(fid * 3 + 0) = colors.col(fid);
		vertex_colors.col(fid * 3 + 1) = colors.col(fid);
		vertex_colors.col(fid * 3 + 2) = colors.col(fid);
	}
}


void Mesh_buffer::set_vertex_positions(const Eigen::Matrix3Xf & positions)
{
	force_assert(positions.cols() == vertices.cols());
	vertices = positions;
}


void Mesh_buffer::set_colorful_spheres(const Eigen::VectorXi & vertex_indices, const Eigen::Matrix4Xf & colors)
{
	force_assert(vertex_indices.size() == colors.cols());
	force_assert(vertex_indices.maxCoeff() < (int)vertices.cols());
	spheres_vertex_indices = vertex_indices;
	spheres_colors = colors;
}


} // end of minimesh
