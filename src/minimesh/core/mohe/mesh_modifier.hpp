#pragma once

//
// mesh_modifier.hpp
//
// Some functionality for modifying meshes.
//
// Author: Shayan Hoshyari
//

#include <Eigen/core>
#include <Eigen/Sparse>
#include <minimesh/core/mohe/mesh_connectivity.hpp>
#include <queue>
#include <unordered_map>

namespace minimesh
{
	namespace mohe
	{
		class Mesh_modifier
		{
		public:
			// Trivial constructor
			explicit Mesh_modifier(Mesh_connectivity& mesh_in);

			// Get the underlying mesh
			Mesh_connectivity& mesh()
			{
				return _m;
			}
			const Mesh_connectivity& mesh() const
			{
				return _m;
			}

			// Get the underlying remeshed mesh
//			Mesh_connectivity& original_mesh()
//			{
//				return _m_original;
//			}
//			const Mesh_connectivity& original_mesh() const
//			{
//				return _m_original;
//			}

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

			// Simplify by k edges
			void simplify(int k);

			// Initialize simplify Quadrics
			void quadrics();

			std::vector<int> get_top_k_errors_edge_vertices(int k);

			// Parametrize with tutte criteria
			void parametrize_tutte();

			// Parametrize with LSCM criteria
			void parametrize_LSCM();

			// Set anchor vertex
			void set_anchor_vertex(int id);

			// Build original Laplacian Matrix
			void build_L();

			// Build the cotangent weight vector
			void build_weights();

			// Calculate deformed vertex positions
			Eigen::Matrix3Xd deform(int deform_id, const Eigen::Vector3d& pos);

			// Perform area equalizing remeshing
			void remesh();

		private:
			// pointer to the mesh that we are working on.
			Mesh_connectivity& _m;
//			Mesh_connectivity& _m_original;

			int _anchor_id = -1;
			int _deform_id = -1;

			Eigen::SparseLU<Eigen::SparseMatrix<double>> _m_solver;
			Eigen::SparseMatrix<double> _m_L;
			std::vector<double> _cot_weights;
			std::unordered_map<int, int> _free_to_original;
			std::unordered_map<int, int> _original_to_free;

			struct Edge_distance
			{
				Mesh_connectivity::Half_edge_iterator he;
				double delta;
				Eigen::Vector3d v;
			};

			struct cmp
			{
				bool operator()(const Edge_distance& e1, const Edge_distance& e2)
				{
					return e1.delta > e2.delta;
				}
			};

			std::priority_queue<Edge_distance, std::vector<Edge_distance>, cmp> errors;

			std::vector<Eigen::Matrix4d> _q;

			static std::tuple<std::vector<double>,
							  std::vector<int>,
							  std::unordered_map<int, int>,
							  std::unordered_map<int, int>>
			generate_coords_traingles(
					Mesh_connectivity::Vertex_ring_iterator v1_ring,
					Mesh_connectivity::Vertex_ring_iterator v2_ring
			);

			// Compute coefficient for even vertices
			static float compute_old_coeff(int n);

			// Initialize the Quadrics for all vertices
			Eigen::Matrix4d compute_Qs(int k);

			// Compute the errors for all the valid pairs, i.e. edges
			Edge_distance compute_errors(int k);

			// Update the Quadrics for the affected vertices
			void update_Qs(
					Mesh_connectivity::Vertex_iterator new_v
			);

			// Update the priority queue with errors
			void update_errors(Mesh_connectivity::Vertex_iterator new_v);

			// Connect the new vertex to its neighbours
			static void collapse_one_edge(
					Mesh_connectivity& temp_mesh,
					Eigen::Vector3d new_v_xyz,
					int v1_index,
					int v2_index
			);

			// Calculate the angles between the two vector in radians
			static double calculate_angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

			// Find boundaries vertices in a counter-clockwise order (assume no interior holes)
			std::pair<std::vector<Mesh_connectivity::Vertex_iterator>, std::vector<int>> find_boundary_verts();

			// Find farthest points in the boundary
			static std::pair<int, int> find_farthest(std::vector<Mesh_connectivity::Vertex_iterator> verts);

			// Return a scaled rotation matrix that taks P1P2 to P1P3
			static Eigen::Matrix2d
			LSCM_coeff_M(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, const Eigen::Vector3d& P3);

			// Calculate cotangent weights for an edge
			double cot(Mesh_connectivity::Half_edge_iterator he);

			// Update the rotation matrices in the deformation
			void update_rotations(std::vector<Eigen::Matrix3d>& m_rots, Eigen::Matrix3Xd& deformed_pos);

			// Remeshing step 1: split long edge
			void split_long_edge(double max_target_length);

			// Remeshing step 2: collapse short edge
			void collapse_short_edge(double min_target_length, double max_target_length);

			// Remeshing step 3: flip edges
			void flip_edges();

			// Remeshing step 4: shift vertices
			void shift_vertices();

			// Remeshing step 5: project vertices
			void project_vertices();

			// Break edge in the mid-point given a half-edge
			void break_half_edge(int id);

			// Break face
			void break_face(
					Mesh_connectivity::Face_iterator old_face,
					Mesh_connectivity::Half_edge_iterator old_half_edge,
					Mesh_connectivity::Vertex_iterator new_vertex,
					std::vector<Mesh_connectivity::Half_edge_iterator> & right_face_half_edges,
					std::vector<Mesh_connectivity::Half_edge_iterator> & left_face_half_edges
					);

			// Helper for connecting the half edges in a triangle (face)
			static void connect_face_half_edges(std::vector<Mesh_connectivity::Half_edge_iterator> & half_edges);

			// Helper function for linking the faces
			static void link_face_to_half_edges(
					std::vector<Mesh_connectivity::Half_edge_iterator> & half_edges,
					Mesh_connectivity::Face_iterator face);

			// Helper function for linking twins
			static void link_twins(Mesh_connectivity::Half_edge_iterator he1, Mesh_connectivity::Half_edge_iterator he2);

			// Check if the edges are too long after collapse
			bool check_max_length_after_collapse(
					Mesh_connectivity::Half_edge_iterator half_edge_to_collapse,
					const Eigen::Vector3d& new_vertex,
					double max_length);

			// Helper for flip edge
			bool flip_edge_remeshing_wrapper(int id);

			// Find valence
			int vertex_valence(Mesh_connectivity::Vertex_iterator vertex, bool & is_boundary);

			// Compute face normals
			void update_face_normals(std::vector<Eigen::Vector3d> & face_normals);

			// Compute vertex normals based on triangle areas
			void update_vertex_normals(std::vector<Eigen::Vector3d>& vertex_normals,
					std::vector<Eigen::Vector3d>& face_normals);
		};
	} // end of mohe
} // end of minimesh
