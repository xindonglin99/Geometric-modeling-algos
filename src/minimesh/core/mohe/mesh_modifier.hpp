#pragma once

//
// mesh_modifier.hpp
//
// Some functionality for modifying meshes.
//
// Author: Shayan Hoshyari
//

#include <Eigen/core>
#include <minimesh/core/mohe/mesh_connectivity.hpp>
#include <queue>
#include <unordered_map>


namespace minimesh {
    namespace mohe {
        class Mesh_modifier {
        public:
            // Trivial constructor
            explicit Mesh_modifier(Mesh_connectivity &mesh_in);

            // Get the underlying mesh
            Mesh_connectivity &mesh() { return _m; }
            const Mesh_connectivity &mesh() const { return _m; }

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

        private:
            // pointer to the mesh that we are working on.
            Mesh_connectivity &_m;

            struct Edge_distance {
                Mesh_connectivity::Half_edge_iterator he;
                double delta;
                Eigen::Vector3d v;
            };

            struct cmp {
                bool operator()(const Edge_distance &e1, const Edge_distance &e2) { return e1.delta > e2.delta; }
            };

            std::priority_queue<Edge_distance, std::vector<Edge_distance>, cmp> errors;

            std::vector<Eigen::Matrix4d> Qs;

            static std::tuple<std::vector<double>, std::vector<int>, std::unordered_map<int, int>, std::unordered_map<int, int>>
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
            static void connect_with_neighbours(
                Mesh_connectivity & temp_mesh,
                Eigen::Vector3d new_v_xyz,
                int v1_index,
                int v2_index
            );

            // Calculate the angles between the two vector in radians
            static double calculate_angle(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
        };
    } // end of mohe
} // end of minimesh
