#include "iostream"
#include <map>
#include <minimesh/core/mohe/mesh_modifier.hpp>
#include <minimesh/core/util/assert.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <unordered_set>

namespace minimesh {
    namespace mohe {
        Mesh_modifier::
        Mesh_modifier(Mesh_connectivity &mesh_in)
            : _m(mesh_in) {
        }

        //
        // Given two vertices, this function return the index of the half-edge going from v0 to v1.
        // Returns -1 if no half-edge exists between the two vertices.
        //
        int
        Mesh_modifier::get_halfedge_between_vertices(const int v0, const int v1) {
            // Get a ring iterator for v0
            Mesh_connectivity::Vertex_ring_iterator ring_iter = mesh().vertex_ring_at(v0);

            int answer = mesh().invalid_index;

            // Loop over all half-edges that end at v0.
            do {
                // Make sure that the half-edge does end at v0
                assert(ring_iter.half_edge().dest().index() == v0);

                // If the half-edge also starts and v1, then it's twin
                // goes from v0 to v1. This would be the half-edge that
                // we were looking for
                if (ring_iter.half_edge().origin().index() == v1) {
                    answer = ring_iter.half_edge().twin().index();
                }
            } while (ring_iter.advance());

            if (answer != mesh().invalid_index) {
                assert(mesh().half_edge_at(answer).origin().index() == v0);
                assert(mesh().half_edge_at(answer).dest().index() == v1);
            }

            return answer;
        }

        bool
        Mesh_modifier::flip_edge(const int he_index) {
            //
            // Take a reference to all involved entities
            //

            // HALF-EDGES
            Mesh_connectivity::Half_edge_iterator he0 = mesh().half_edge_at(he_index);
            Mesh_connectivity::Half_edge_iterator he1 = he0.twin();

            // meshes on the boundary are not flippable
            if (he0.face().is_equal(mesh().hole()) || he1.face().is_equal(mesh().hole())) {
                return false;
            }

            Mesh_connectivity::Half_edge_iterator he2 = he0.next();
            Mesh_connectivity::Half_edge_iterator he3 = he2.next();
            Mesh_connectivity::Half_edge_iterator he4 = he1.next();
            Mesh_connectivity::Half_edge_iterator he5 = he4.next();

            // VERTICES
            Mesh_connectivity::Vertex_iterator v0 = he1.origin();
            Mesh_connectivity::Vertex_iterator v1 = he0.origin();
            Mesh_connectivity::Vertex_iterator v2 = he3.origin();
            Mesh_connectivity::Vertex_iterator v3 = he5.origin();

            // FACES
            Mesh_connectivity::Face_iterator f0 = he0.face();
            Mesh_connectivity::Face_iterator f1 = he1.face();

            //
            // Now modify the connectivity
            //

            // HALF-EDGES
            he0.data().next = he3.index();
            he0.data().prev = he4.index();
            he0.data().origin = v3.index();
            //
            he1.data().next = he5.index();
            he1.data().prev = he2.index();
            he1.data().origin = v2.index();
            //
            he2.data().next = he1.index();
            he2.data().prev = he5.index();
            he2.data().face = f1.index();
            //
            he3.data().next = he4.index();
            he3.data().prev = he0.index();
            //
            he4.data().next = he0.index();
            he4.data().prev = he3.index();
            he4.data().face = f0.index();
            //
            he5.data().next = he2.index();
            he5.data().prev = he1.index();

            // VERTICES
            v0.data().half_edge = he2.index();
            v1.data().half_edge = he4.index();
            v2.data().half_edge = he1.index();
            v3.data().half_edge = he0.index();

            // FACES
            f0.data().half_edge = he0.index();
            f1.data().half_edge = he1.index();

            // operation successful
            return true;
        } // All done

        // borrowed from https://github.com/icemiliang/loop_subdivision
        float
        Mesh_modifier::compute_old_coeff(int n) {
            float beta;
            if (n > 3) {
                float center = (0.375f + (0.25f * cos(6.2831853f / (float) n))); // 2.0f * 3.1415926f
                beta = (0.625f - (center * center)) / (float) n;
            } else {
                beta = 0.1875f; // 3.0f / 16.0f;
            }
            return beta;
        }

        void Mesh_modifier::quadrics() {
            for (int i = 0; i < mesh().n_active_vertices(); ++i) {
                this->Qs.push_back(this->compute_Qs(i));
            }
            // Keep a O(1) HashSet for visited half-edge's twins
            std::unordered_set<int> visited;
            for (int i = 0; i < mesh().n_active_half_edges(); ++i) {
                if (visited.find(i) == visited.end()) {
                    visited.insert(mesh().half_edge_at(i).twin().index());
                    this->errors.push(this->compute_errors(i));
                }
            }
        }

        Eigen::Matrix4d
        Mesh_modifier::compute_Qs(const int k) {
            Eigen::Matrix4d Q;
            Q.setZero();
            Mesh_connectivity::Vertex_ring_iterator curr = mesh().vertex_ring_at(k);
            do {
                Mesh_connectivity::Vertex_iterator a = curr.half_edge().origin();
                Mesh_connectivity::Vertex_iterator b = curr.half_edge().dest();
                Mesh_connectivity::Vertex_iterator c = curr.half_edge().next().dest();

                Eigen::Vector3d e1 = b.xyz() - a.xyz();
                Eigen::Vector3d e2 = c.xyz() - b.xyz();

                Eigen::Vector3d n = e1.cross(e2);
                n.normalize();

                Eigen::Vector4d target;
                target << n.head(3), -n.dot(b.xyz());
                Eigen::Matrix4d tmpQ = target * target.transpose();

                Q += tmpQ;
            } while (curr.advance());

            return Q;
        }

        Mesh_modifier::Edge_distance
        Mesh_modifier::compute_errors(const int k) {
            // v1,v2
            Mesh_connectivity::Vertex_iterator v1 = mesh().half_edge_at(k).origin();
            Mesh_connectivity::Vertex_iterator v2 = mesh().half_edge_at(k).dest();

            // Calculate the Q hat for the two vertices
            Eigen::Matrix4d Q_hat = this->Qs[v1.index()] + this->Qs[v2.index()];

            // Set the linear system
            Eigen::Matrix4d Q_hat_d = Q_hat;
            Q_hat_d(3, 0) = 0;
            Q_hat_d(3, 1) = 0;
            Q_hat_d(3, 2) = 0;
            Q_hat_d(3, 3) = 1;
            const Eigen::Vector4d b(0, 0, 0, 1.0);

            // Use FullPivLU to check invertibility first
            const Eigen::FullPivLU<Eigen::Matrix4d> lu(Q_hat_d);
            Eigen::Vector4d v;

            // If invertible, use PLU to solve, faster, else use mid-point
            if (lu.isInvertible()) {
                const Eigen::PartialPivLU<Eigen::Matrix4d> plu(Q_hat_d);
                v = plu.solve(b);
            } else {
                Eigen::Vector3d tmp = (v1.xyz() + v2.xyz()) / 2;
                v << tmp.head(3), 1.0;
            }
            // Calculate and push the error into the heap
            const double error = v.transpose() * Q_hat * v;
            Edge_distance edge_distance{mesh().half_edge_at(k), error, Eigen::Vector3d(v.head(3))};

            return edge_distance;
        }

        // Should pass in v1, as we are deactivating v2
        void
        Mesh_modifier::update_Qs(Mesh_connectivity::Vertex_iterator new_v) {
            this->Qs[new_v.index()] = this->compute_Qs(new_v.index());
        }

        void Mesh_modifier::update_errors(Mesh_connectivity::Vertex_iterator new_v) {
            Mesh_connectivity::Vertex_ring_iterator new_v_ring = mesh().vertex_ring_at(new_v.index());
            std::vector<int> neighbour_edges;
            do {
                neighbour_edges.push_back(new_v_ring.half_edge().index());
            } while (new_v_ring.advance());
            for (const int ind: neighbour_edges) {
                this->errors.push(this->compute_errors(ind));
            }
        }

        void Mesh_modifier::connect_with_neighbours(
            Mesh_connectivity::Vertex_ring_iterator &original_v,
            Mesh_connectivity::Vertex_iterator &other_v,
            Mesh_connectivity::Vertex_iterator &new_v_itr
        ) {
            // Loop over the half-edge that goes into the original v
            do {
                if (!original_v.half_edge().origin().is_equal(other_v)) {
                    original_v.half_edge().twin().data().origin = new_v_itr.index();
                    if (new_v_itr.data().half_edge == Mesh_connectivity::invalid_index) {
                        new_v_itr.data().half_edge = original_v.half_edge().index();
                    }
                }
            } while (original_v.advance());
        }

        std::tuple<std::vector<double>, std::vector<int>, int>
        Mesh_modifier::generate_coords_traingles(
            Mesh_connectivity::Vertex_ring_iterator v1_ring,
            Mesh_connectivity::Vertex_ring_iterator v2_ring,
            Mesh_connectivity::Vertex_iterator v1,
            Mesh_connectivity::Vertex_iterator v2
        ) {
            std::vector<double> coords;
            std::vector<int> triangles;
            std::unordered_map<int, int> old_to_new;
            std::unordered_map<int, int> oldf_to_new;

            int vertex_count = 0;
            do {
                std::vector<Mesh_connectivity::Vertex_iterator> verts;
                Mesh_connectivity::Vertex_iterator a = v1_ring.half_edge().origin();
                Mesh_connectivity::Vertex_iterator b = v1_ring.half_edge().dest();
                Mesh_connectivity::Vertex_iterator c = v1_ring.half_edge().next().dest();
                verts.push_back(a);
                verts.push_back(b);
                verts.push_back(c);

                // Add all the vertices relatd to v1
                for (auto & vert : verts) {
                    if (old_to_new.find(vert.index()) == old_to_new.end()) {
                        old_to_new.insert(std::make_pair(vert.index(), vertex_count));

                        coords.push_back(vert.data().xyz[0]);
                        coords.push_back(vert.data().xyz[1]);
                        coords.push_back(vert.data().xyz[2]);
                        vertex_count += 1;
                    }
                }

                Mesh_connectivity::Face_iterator

                // Add all the faces related to v1
                triangles.push_back(face_counter * 3);
                triangles.push_back(face_counter * 3 + 1);
                triangles.push_back(face_counter * 3 + 2);
                face_counter += 1;
            } while (v1_ring.advance());

            do {
                Mesh_connectivity::Vertex_iterator a = v2_ring.half_edge().origin();
                Mesh_connectivity::Vertex_iterator b = v2_ring.half_edge().dest();
                Mesh_connectivity::Vertex_iterator c = v2_ring.half_edge().next().dest();

                coords.push_back(a.data().xyz[0]);
                coords.push_back(a.data().xyz[1]);
                coords.push_back(a.data().xyz[2]);

                coords.push_back(b.data().xyz[0]);
                coords.push_back(b.data().xyz[1]);
                coords.push_back(b.data().xyz[2]);

                coords.push_back(c.data().xyz[0]);
                coords.push_back(c.data().xyz[1]);
                coords.push_back(c.data().xyz[2]);

                triangles.push_back(face_counter * 3);
                triangles.push_back(face_counter * 3 + 1);
                triangles.push_back(face_counter * 3 + 2);
                face_counter += 1;
            } while (v2_ring.advance());

            return std::make_pair(coords, triangles);
        }

        void
        Mesh_modifier::simplify(int k) {
            while (k > 0 && !this->errors.empty()) {
                Edge_distance to_pop = this->errors.top();
                if (!to_pop.he.is_active()) {continue;}
                Mesh_connectivity::Vertex_iterator v1 = to_pop.he.origin();
                Mesh_connectivity::Vertex_iterator v2 = to_pop.he.dest();

                Mesh_connectivity::Vertex_ring_iterator v1_ring = mesh().vertex_ring_at(v1.index());
                Mesh_connectivity::Vertex_ring_iterator v2_ring = mesh().vertex_ring_at(v2.index());

                Mesh_connectivity temp_mesh;
                std::pair<std::vector<double>, std::vector<int>> result = generate_coords_traingles(v1_ring, v2_ring, v1, v2);
                temp_mesh.build_from_triangles(result.first, result.second);

                connect_with_neighbours();
                if (!temp_mesh.check_sanity_slowly()) {
                    printf("Current edge to collapse creates invalid topology. Skipping..");
                    continue;
                }

                connect_with_neighbours();
                this->errors.pop();
                this->update_Qs(v1);
                this->update_errors(v1);
                k--;
            }

            if (k > 0) {printf("No valid edge to simplify. Simplification process will stop.");}
        }

        void
        Mesh_modifier::subdivide() {
            std::vector<double> coords;
            std::vector<int> triangles;
            coords.reserve(3 * mesh().n_total_vertices() + int(3 * mesh().n_total_half_edges() / 2));
            triangles.reserve(4 * mesh().n_total_faces());

            // Loop through each vertex, compute its new position according to its neighbours
            for (int i = 0; i < mesh().n_total_vertices(); i++) {
                Eigen::Vector3d vNew;
                Mesh_connectivity::Vertex_ring_iterator curr = mesh().vertex_ring_at(i);
                std::vector<Mesh_connectivity::Vertex_iterator> neighbours;
                neighbours.reserve(6);
                int k = 0;
                do {
                    neighbours.push_back(curr.half_edge().origin());
                    k++;
                } while (curr.advance());
                float beta = compute_old_coeff(k);
                vNew = curr.half_edge().dest().xyz() * (1.0f - (float(k) * beta));
                for (int j = 0; j < k; j++) {
                    vNew += (neighbours[j].data().xyz * beta);
                }
                coords.push_back(vNew.x());
                coords.push_back(vNew.y());
                coords.push_back(vNew.z());
            }

            // Loop through all edges, compute new vertex
            std::map<int, int> he_newV_indices;
            for (int i = 0; i < mesh().n_total_half_edges(); i++) {
                Mesh_connectivity::Half_edge_iterator curr_edge = mesh().half_edge_at(i);
                if (curr_edge.is_active()) {
                    Eigen::Vector3d vNew;
                    vNew = (curr_edge.origin().xyz() + curr_edge.dest().xyz()) * (3.0f / 8.0f);
                    vNew += (curr_edge.next().dest().xyz() + curr_edge.twin().next().dest().xyz()) * (1.0f / 8.0f);

                    he_newV_indices[curr_edge.index()] = int(coords.size() / 3);
                    he_newV_indices[curr_edge.twin().index()] = int(coords.size() / 3);
                    coords.push_back(vNew.x());
                    coords.push_back(vNew.y());
                    coords.push_back(vNew.z());

                    curr_edge.twin().deactivate();
                }
            }

            // Add new faces and half edges
            for (int i = 0; i < mesh().n_total_faces(); i++) {
                // 3 half edges related to the face
                Mesh_connectivity::Half_edge_iterator he0 = mesh().face_at(i).half_edge();
                Mesh_connectivity::Half_edge_iterator he1 = he0.next();
                Mesh_connectivity::Half_edge_iterator he2 = he1.next();

                // 3 vertices related to the face
                int v0 = he0.origin().index();
                int v1 = he1.origin().index();
                int v2 = he2.origin().index();

                // 3 newly added vertices corresponding to the 3 half edges
                int v0_new = he_newV_indices[he0.index()];
                int v1_new = he_newV_indices[he1.index()];
                int v2_new = he_newV_indices[he2.index()];

                // f0
                triangles.push_back(v0);
                triangles.push_back(v0_new);
                triangles.push_back(v2_new);

                // f1
                triangles.push_back(v0_new);
                triangles.push_back(v1);
                triangles.push_back(v1_new);

                // f3
                triangles.push_back(v0_new);
                triangles.push_back(v1_new);
                triangles.push_back(v2_new);

                // f4
                triangles.push_back(v2_new);
                triangles.push_back(v1_new);
                triangles.push_back(v2);
            }

            mesh().clear();
            mesh().build_from_triangles(coords, triangles);
        }

        std::vector<int> Mesh_modifier::get_top_k_errors_edge_vertices(int k) {
            size_t count = k;
            if (this->errors.size() < k) { count = this->errors.size(); }

            std::vector<int> results;
            results.reserve(count);
            std::priority_queue<Edge_distance, std::vector<Edge_distance>, cmp> tmpQ = this->errors;

            while (!tmpQ.empty() && count > 0) {
                Edge_distance top = tmpQ.top();
                results.push_back(top.he.origin().index());
                results.push_back(top.he.dest().index());
                tmpQ.pop();
                count--;
            }
            return results;
        }
    } // end of mohe
} // end of minimesh
