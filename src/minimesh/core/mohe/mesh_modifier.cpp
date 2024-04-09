#define _USE_MATH_DEFINES
#include "iostream"
#include <map>
#include <minimesh/core/mohe/mesh_modifier.hpp>
#include <minimesh/core/util/assert.hpp>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <Eigen/SparseCholesky>

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
    this->_q.push_back(this->compute_Qs(i));
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
  Eigen::Matrix4d Q_hat = this->_q[v1.index()] + this->_q[v2.index()];

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
    v = lu.solve(b);
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
  this->_q[new_v.index()] = this->compute_Qs(new_v.index());
}

void Mesh_modifier::update_errors(Mesh_connectivity::Vertex_iterator new_v) {
  Mesh_connectivity::Vertex_ring_iterator new_v_ring = mesh().vertex_ring_at(new_v.index());
  std::vector<int> neighbour_edges;
  do {
    neighbour_edges.push_back(new_v_ring.half_edge().index());
  } while (new_v_ring.advance());
  for (const int ind : neighbour_edges) {
    this->errors.push(this->compute_errors(ind));
  }
}

void Mesh_modifier::connect_with_neighbours(
    Mesh_connectivity &temp_mesh,
    Eigen::Vector3d new_v_xyz,
    int v1_index,
    int v2_index
) {
  temp_mesh.vertex_at(v1_index).data().xyz = new_v_xyz;
  Mesh_connectivity::Vertex_ring_iterator v2 = temp_mesh.vertex_ring_at(v2_index);
  Mesh_connectivity::Vertex_ring_iterator v2_rest = temp_mesh.vertex_ring_at(v2_index);
  int he_at_v1_v2_ind = -10;

  do {
    if (v2.half_edge().origin().is_equal(temp_mesh.vertex_at(v1_index))) {
      he_at_v1_v2_ind = v2.half_edge().index();
      break;
    }
  } while (v2.advance());

  Mesh_connectivity::Half_edge_iterator he_at_v1_v2 = temp_mesh.half_edge_at(he_at_v1_v2_ind);
  // Deactivating 6 edges associating to the 2 deleted faces
  he_at_v1_v2.deactivate();
  he_at_v1_v2.twin().deactivate();

  he_at_v1_v2.next().deactivate();
  he_at_v1_v2.next().twin().deactivate();

  he_at_v1_v2.twin().prev().deactivate();
  he_at_v1_v2.twin().prev().twin().deactivate();

  he_at_v1_v2.face().deactivate();
  he_at_v1_v2.twin().face().deactivate();

  temp_mesh.vertex_at(v2_index).deactivate();
  // Do the rest of triangles
  do {
    if (v2_rest.half_edge().is_active()) {
      v2_rest.half_edge().twin().data().origin = v1_index;
    }
  } while (v2_rest.advance());

  // Need to fix connectivity
  Mesh_connectivity::Half_edge_iterator he_need_to_change_1 = he_at_v1_v2.prev();
  Mesh_connectivity::Half_edge_iterator he_need_to_change_2 = he_at_v1_v2.twin().next();

  Mesh_connectivity::Half_edge_iterator he_need_to_change_3 = he_at_v1_v2.next().twin().next();
  Mesh_connectivity::Half_edge_iterator he_need_to_change_4 = he_need_to_change_3.next();

  Mesh_connectivity::Half_edge_iterator he_need_to_change_5 = he_at_v1_v2.twin().prev().twin().prev();
  Mesh_connectivity::Half_edge_iterator he_need_to_change_6 = he_need_to_change_5.prev();

  // If the face contains deleted edges, use the prev one
  if (he_at_v1_v2.next().twin().face().half_edge().is_equal(he_at_v1_v2.next().twin())) {
    he_at_v1_v2.next().twin().face().data().half_edge = he_at_v1_v2.next().twin().prev().index();
  }

  if (he_at_v1_v2.twin().prev().twin().face().half_edge().is_equal(he_at_v1_v2.twin().prev().twin())) {
    he_at_v1_v2.twin().prev().twin().face().data().half_edge = he_at_v1_v2.twin().prev().twin().prev().index();
  }

  temp_mesh.vertex_at(v1_index).data().half_edge = he_need_to_change_2.index();
  he_need_to_change_1.origin().data().half_edge = he_need_to_change_1.index();
  he_need_to_change_2.dest().data().half_edge = he_need_to_change_6.index();

  he_need_to_change_1.data().prev = he_need_to_change_4.index();
  he_need_to_change_1.data().next = he_need_to_change_3.index();

  he_need_to_change_2.data().prev = he_need_to_change_5.index();
  he_need_to_change_2.data().next = he_need_to_change_6.index();

  he_need_to_change_3.data().prev = he_need_to_change_1.index();

  he_need_to_change_4.data().next = he_need_to_change_1.index();

  he_need_to_change_5.data().next = he_need_to_change_2.index();

  he_need_to_change_6.data().prev = he_need_to_change_2.index();

  he_need_to_change_3.data().origin = v1_index;
  // Give the left edge correct faces
  he_need_to_change_1.data().face = he_need_to_change_3.face().index();
  he_need_to_change_2.data().face = he_need_to_change_6.face().index();
}

std::tuple<std::vector<double>, std::vector<int>, std::unordered_map<int, int>, std::unordered_map<int, int>>
Mesh_modifier::generate_coords_traingles(
    Mesh_connectivity::Vertex_ring_iterator v1_ring,
    Mesh_connectivity::Vertex_ring_iterator v2_ring
) {
  std::vector<double> coords;
  std::vector<int> triangles;
  std::unordered_map<int, int> old_to_new;
  std::unordered_map<int, int> oldf_to_new;

  int vertex_count = 0;
  int face_counter = 0;
  do {
    std::vector<Mesh_connectivity::Vertex_iterator> verts;
    Mesh_connectivity::Vertex_iterator a = v1_ring.half_edge().origin();
    Mesh_connectivity::Vertex_iterator b = v1_ring.half_edge().dest();
    Mesh_connectivity::Vertex_iterator c = v1_ring.half_edge().next().dest();
    verts.push_back(a);
    verts.push_back(b);
    verts.push_back(c);

    // Add all the vertices related to v1
    for (auto &vert : verts) {
      if (old_to_new.find(vert.index()) == old_to_new.end()) {
        old_to_new.insert(std::make_pair(vert.index(), vertex_count));

        coords.push_back(vert.data().xyz[0]);
        coords.push_back(vert.data().xyz[1]);
        coords.push_back(vert.data().xyz[2]);
        vertex_count += 1;
      }
    }

    // Add all the faces related to v1
    Mesh_connectivity::Face_iterator face = v1_ring.half_edge().face();
    if (oldf_to_new.find(face.index()) == oldf_to_new.end()) {
      oldf_to_new.insert(std::make_pair(face.index(), face_counter));
      // Add all the faces related to v1
      triangles.push_back(old_to_new[face.half_edge().origin().index()]);
      triangles.push_back(old_to_new[face.half_edge().dest().index()]);
      triangles.push_back(old_to_new[face.half_edge().next().dest().index()]);
      face_counter += 1;
    }
  } while (v1_ring.advance());

  do {
    std::vector<Mesh_connectivity::Vertex_iterator> verts;
    Mesh_connectivity::Vertex_iterator a = v2_ring.half_edge().origin();
    Mesh_connectivity::Vertex_iterator b = v2_ring.half_edge().dest();
    Mesh_connectivity::Vertex_iterator c = v2_ring.half_edge().next().dest();
    verts.push_back(a);
    verts.push_back(b);
    verts.push_back(c);

    // Add all the vertices related to v1
    for (auto &vert : verts) {
      if (old_to_new.find(vert.index()) == old_to_new.end()) {
        old_to_new.insert(std::make_pair(vert.index(), vertex_count));

        coords.push_back(vert.data().xyz[0]);
        coords.push_back(vert.data().xyz[1]);
        coords.push_back(vert.data().xyz[2]);
        vertex_count += 1;
      }
    }

    // Add all the faces related to v1
    Mesh_connectivity::Face_iterator face = v2_ring.half_edge().face();
    if (oldf_to_new.find(face.index()) == oldf_to_new.end()) {
      oldf_to_new.insert(std::make_pair(face.index(), face_counter));
      // Add all the faces related to v1
      triangles.push_back(old_to_new[face.half_edge().origin().index()]);
      triangles.push_back(old_to_new[face.half_edge().dest().index()]);
      triangles.push_back(old_to_new[face.half_edge().next().dest().index()]);
      face_counter += 1;
    }
  } while (v2_ring.advance());

  return std::make_tuple(coords, triangles, old_to_new, oldf_to_new);
}

void
Mesh_modifier::simplify(int k) {
  while (k > 0 && !this->errors.empty()) {
    Edge_distance to_pop = this->errors.top();
    this->errors.pop();
    if (!to_pop.he.is_active()) {
      continue;
    }

    // v1 will keep, v2 will deactivate
    Mesh_connectivity::Vertex_iterator v1 = to_pop.he.origin();
    Mesh_connectivity::Vertex_iterator v2 = to_pop.he.dest();

    Mesh_connectivity::Vertex_ring_iterator v1_ring = mesh().vertex_ring_at(v1.index());
    Mesh_connectivity::Vertex_ring_iterator v2_ring = mesh().vertex_ring_at(v2.index());

    Mesh_connectivity temp_mesh;
    auto result = generate_coords_traingles(v1_ring, v2_ring);
    temp_mesh.build_from_triangles(std::get<0>(result), std::get<1>(result));

    connect_with_neighbours(temp_mesh, to_pop.v, std::get<2>(result)[v1.index()], std::get<2>(result)[v2.index()]);
    if (!temp_mesh.check_sanity_slowly(true)) {
      printf("Current edge to collapse creates invalid topology. Skipping..");
      continue;
    }
    connect_with_neighbours(mesh(), to_pop.v, v1.index(), v2.index());
    this->update_Qs(v1);
    this->update_errors(v1);
    k--;
  }

  if (k > 0) { printf("No valid edge to simplify. Simplification process will stop."); }
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

void Mesh_modifier::parametrize_tutte() {
  const int N = mesh().n_total_vertices();

  //Get the boundary vertices in counter-clockwise order
  const auto p = find_boundary_verts();
  std::vector<Mesh_connectivity::Vertex_iterator> boundary_v = p.first;
  std::vector<int> boundary_ind = p.second;

  // Boundary vertices size K
  const int K = (int) boundary_v.size();

  std::vector<double> U(K);
  std::vector<double> V(K);

  // Map the boundary to unit circle
  for (int i = 0; i < K; ++i) {
    // Upper half of the circle
    U[i] = cos(2 * M_PI * i / K);
    V[i] = sin(2 * M_PI * i / K);
  }

  // Calculate Laplacian matrix
  std::vector<Eigen::Triplet<double>> W_elem;
  W_elem.reserve(N);
  Eigen::SparseMatrix<double> W(N, N);
  Eigen::VectorXd b_u = Eigen::VectorXd::Zero(N);
  Eigen::VectorXd b_v = Eigen::VectorXd::Zero(N);
  for (int i = 0; i < N; ++i) {
    auto it = std::find(boundary_ind.begin(), boundary_ind.end(), i);
    if (it != boundary_ind.end()) {
      W_elem.emplace_back(i, i, 1.0);
      const auto index = std::distance(boundary_ind.begin(), it);
      b_u[i] = U[index];
      b_v[i] = V[index];
    } else {
      Mesh_connectivity::Vertex_ring_iterator ring = mesh().vertex_ring_at(i);
      double sum = 0.0;
      do {
        Mesh_connectivity::Vertex_iterator jth_vertex = ring.half_edge().origin();
        int jth = jth_vertex.index();

        // This triangle (current face)
        Mesh_connectivity::Vertex_iterator kth_curr = ring.half_edge().next().dest();
        Eigen::Vector3d vector_ij = (jth_vertex.xyz() - ring.half_edge().dest().xyz());
        Eigen::Vector3d vector_ik_curr = (kth_curr.xyz() - ring.half_edge().dest().xyz());

        // Neighbor triangle (twin face)
        Mesh_connectivity::Vertex_iterator kth_twin = ring.half_edge().twin().next().dest();
        Eigen::Vector3d vector_ik_twin = (kth_twin.xyz() - ring.half_edge().dest().xyz());

        double theta_curr = calculate_angle(vector_ij, vector_ik_curr);
        double theta_twin = calculate_angle(vector_ij, vector_ik_twin);

        double unormalized_lambda = (tan(0.5 * theta_curr) + tan(0.5 * theta_twin)) / vector_ij.norm();
        W_elem.emplace_back(i, jth, unormalized_lambda);
        sum += unormalized_lambda;
      } while (ring.advance());
      W_elem.emplace_back(i, i, -sum);
    }
  }

  // Solve the two systems using a sparseLU solver
  W.setFromTriplets(W_elem.begin(), W_elem.end());
  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
  solver.compute(W);

  Eigen::VectorXd solved_u = solver.solve(b_u);
  Eigen::VectorXd solved_v = solver.solve(b_v);

  for (int i = 0; i < N; ++i) {
    mesh().vertex_at(i).data().xyz[0] = solved_u[i];
    mesh().vertex_at(i).data().xyz[1] = solved_v[i];
    mesh().vertex_at(i).data().xyz[2] = 0.0;
  }
}

double Mesh_modifier::calculate_angle(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return acos(a.normalized().dot(b.normalized()));
}

std::pair<std::vector<Mesh_connectivity::Vertex_iterator>, std::vector<int>> Mesh_modifier::find_boundary_verts() {
  int first_boundary_he_index = -3;
  // Total number of vertices

  for (int i = 0; i < mesh().n_total_half_edges(); ++i) {
    if (mesh().half_edge_at(i).face().is_equal(mesh().hole())) {
      first_boundary_he_index = i;
      break;
    }
  }

  //Get the boundary vertices in counter-clockwise order
  std::vector<Mesh_connectivity::Vertex_iterator> boundary_v;
  std::vector<int> boundary_ind;
  boundary_v.reserve(mesh().n_total_vertices() / 3);
  boundary_ind.reserve(mesh().n_total_vertices() / 3);
  Mesh_connectivity::Half_edge_iterator tmp = mesh().half_edge_at(first_boundary_he_index);
  do {
    // prev and origin for counter-clockwise order
    boundary_v.push_back(tmp.origin());
    boundary_ind.push_back(tmp.origin().index());
    tmp = tmp.prev();
  } while (!tmp.is_equal(mesh().half_edge_at(first_boundary_he_index)));

  return std::make_pair(boundary_v, boundary_ind);
}

std::pair<int, int> Mesh_modifier::find_farthest(std::vector<Mesh_connectivity::Vertex_iterator> verts) {
  double max_dist = -1;
  std::pair<int, int> curr_largest_ind = std::make_pair(-1, -1);

  for (int i = 0; i < verts.size(); ++i) {
    for (int j = 0; j < verts.size(); ++j) {
      const double curr_dist = (verts[i].xyz() - verts[j].xyz()).norm();
      if (curr_dist > max_dist) {
        max_dist = curr_dist;
        curr_largest_ind = std::make_pair(i, j);
      }
    }
  }

  return curr_largest_ind;
}

Eigen::Matrix2d Mesh_modifier::LSCM_coeff_M(const Eigen::Vector3d &P1,
                                            const Eigen::Vector3d &P2,
                                            const Eigen::Vector3d &P3) {
  const Eigen::Vector3d p1p2 = P2 - P1;
  const Eigen::Vector3d p1p3 = P3 - P1;
  const Eigen::Vector3d p2p1 = -p1p2;
  const Eigen::Vector3d p2p3 = P3 - P2;
  const Eigen::Vector3d p3p1 = -p1p3;
  const Eigen::Vector3d p3p2 = -p2p3;
  const double cos_angle_p1 = p1p2.normalized().dot(p1p3.normalized());
  const double sin_angle_p1 = sin(acos(cos_angle_p1));
  Eigen::Matrix2d rot_M;
  rot_M << cos_angle_p1, sin_angle_p1, -sin_angle_p1, cos_angle_p1;

  const double sin_angle_p2 = sin(acos(p2p1.normalized().dot(p2p3.normalized())));
  const double sin_angle_p3 = sin(acos(p3p1.normalized().dot(p3p2.normalized())));

  return (sin_angle_p2 / sin_angle_p3) * rot_M;
}

void Mesh_modifier::parametrize_LSCM() {
  // Get boundaries
  const auto p = find_boundary_verts();
  std::vector<Mesh_connectivity::Vertex_iterator> boundary_v = p.first;
  std::vector<int> boundary_ind = p.second;

  const int N = mesh().n_total_vertices();
  const int F = mesh().n_total_faces();

  // Fix the two vertices that has the largest distance
  const std::pair<int, int> ind = find_farthest(boundary_v);
  std::pair<int, int> fixed_verts_ind = std::make_pair(boundary_v[ind.first].index(), boundary_v[ind.second].index());
  int small_fix_ind = fixed_verts_ind.first < fixed_verts_ind.second ? fixed_verts_ind.first : fixed_verts_ind.second;
  int large_fix_ind = fixed_verts_ind.first > fixed_verts_ind.second ? fixed_verts_ind.first : fixed_verts_ind.second;
  std::vector<int> defrag_map(N);
  for (int i = 0; i < N; ++i) {
    if (i < small_fix_ind) {
      defrag_map[i] = i;
    } else if (i > small_fix_ind && i < large_fix_ind) {
      defrag_map[i] = i - 1;
    } else {
      defrag_map[i] = i - 2;
    }
  }
  defrag_map[small_fix_ind] = -3;
  defrag_map[large_fix_ind] = -3;

  // Declare the solvables
  Eigen::SparseMatrix<double> W(6 * F, 2 * (N - 2));
  Eigen::SparseMatrix<double> B(6 * F, 4);
  Eigen::Vector4d b(0, 0, 0, 1);
  std::vector<Eigen::Triplet<double>> W_elem;
  std::vector<Eigen::Triplet<double>> B_elem;

  // Loop through all the faces
  for (int i = 0; i < F; ++i) {
    Mesh_connectivity::Half_edge_iterator he = mesh().face_at(i).half_edge();
    int corner_count = 0;
    do {
      Mesh_connectivity::Vertex_iterator P1 = he.dest();
      Mesh_connectivity::Vertex_iterator P2 = he.origin();
      Mesh_connectivity::Vertex_iterator P3 = he.next().dest();

      Eigen::Matrix2d rot_M = LSCM_coeff_M(P1.xyz(), P2.xyz(), P3.xyz());
      // First equation of this corner
      if (P1.index() == small_fix_ind) {
        B_elem.emplace_back(i * 6 + 2 * corner_count, 0, -(rot_M(0, 0) - 1));
        B_elem.emplace_back(i * 6 + 2 * corner_count, 1, -(rot_M(0, 1)));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 0, -(rot_M(1, 0)));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 1, -(rot_M(1, 1) - 1));
      } else if (P1.index() == large_fix_ind) {
        B_elem.emplace_back(i * 6 + 2 * corner_count, 2, -(rot_M(0, 0) - 1));
        B_elem.emplace_back(i * 6 + 2 * corner_count, 3, -(rot_M(0, 1)));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 2, -(rot_M(1, 0)));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 3, -(rot_M(1, 1) - 1));
      } else {
        W_elem.emplace_back(i * 6 + 2 * corner_count, 2 * defrag_map[P1.index()], rot_M(0, 0) - 1); // u1 * R11-1
        W_elem.emplace_back(i * 6 + 2 * corner_count, 2 * defrag_map[P1.index()] + 1, rot_M(0, 1)); //v1 * R12
        W_elem.emplace_back(i * 6 + 2 * corner_count + 1, 2 * defrag_map[P1.index()], rot_M(1, 0)); // u1 * R21
        W_elem.emplace_back(i * 6 + 2 * corner_count + 1, 2 * defrag_map[P1.index()] + 1, rot_M(1, 1) - 1);//v1 * R22 -1
      }

      if (P2.index() == small_fix_ind) {
        B_elem.emplace_back(i * 6 + 2 * corner_count, 0, rot_M(0, 0));
        B_elem.emplace_back(i * 6 + 2 * corner_count, 1, rot_M(0, 1));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 0, rot_M(1, 0));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 1, rot_M(1, 1));
      } else if (P2.index() == large_fix_ind) {
        B_elem.emplace_back(i * 6 + 2 * corner_count, 2, rot_M(0, 0));
        B_elem.emplace_back(i * 6 + 2 * corner_count, 3, rot_M(0, 1));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 2, rot_M(1, 0));
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 3, rot_M(1, 1));
      } else {
        W_elem.emplace_back(i * 6 + 2 * corner_count, 2 * defrag_map[P2.index()], -rot_M(0, 0)); // -R11 * u2
        W_elem.emplace_back(i * 6 + 2 * corner_count, 2 * defrag_map[P2.index()] + 1, -rot_M(0, 1)); // -R12 * v2
        W_elem.emplace_back(i * 6 + 2 * corner_count + 1, 2 * defrag_map[P2.index()], -rot_M(1, 0));// -R21 * u2
        W_elem.emplace_back(i * 6 + 2 * corner_count + 1, 2 * defrag_map[P2.index()] + 1, -rot_M(1, 1));// -R22 * v2
      }

      if (P3.index() == small_fix_ind) {
        B_elem.emplace_back(i * 6 + 2 * corner_count, 0, -1.0);
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 1, -1.0);
      } else if (P3.index() == large_fix_ind) {
        B_elem.emplace_back(i * 6 + 2 * corner_count, 2, -1.0);
        B_elem.emplace_back(i * 6 + 2 * corner_count + 1, 3, -1.0);
      } else {
        W_elem.emplace_back(i * 6 + 2 * corner_count, 2 * defrag_map[P3.index()], 1.0); // u3
        W_elem.emplace_back(i * 6 + 2 * corner_count + 1, 2 * defrag_map[P3.index()] + 1, 1.0);// v3
      }

      corner_count++;
      he = he.next();
    } while (!he.is_equal(mesh().face_at(i).half_edge()));
  }

//  // Extra two rows for the constraints on the fixed boundary vertex
//  // i.e. u_fixed = 0, v_fixed = 0;
//  W_elem.emplace_back(6 * F, 2*fixed_verts_ind.first, 1.0);
//  W_elem.emplace_back(6 * F + 1, 2*fixed_verts_ind.first+1, 1.0);
//  W_elem.emplace_back(6 * F + 2, 2*fixed_verts_ind.second, 1.0);
//  W_elem.emplace_back(6 * F + 3, 2*fixed_verts_ind.second+1, 1.0);
//  b[6 * F + 3] = 1.0;

  W.setFromTriplets(W_elem.begin(), W_elem.end());
  B.setFromTriplets(B_elem.begin(), B_elem.end());

  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
  solver.compute(W.transpose() * W);
  Eigen::VectorXd uv = solver.solve(W.transpose() * B * b);

  for (int i = 0; i < N; ++i) {
    if (i == small_fix_ind) {
      mesh().vertex_at(i).data().xyz[0] = 0;
      mesh().vertex_at(i).data().xyz[1] = 0.0;
      mesh().vertex_at(i).data().xyz[2] = 0.0;
    } else if (i == large_fix_ind) {
      mesh().vertex_at(i).data().xyz[0] = 0;
      mesh().vertex_at(i).data().xyz[1] = 1.0;
      mesh().vertex_at(i).data().xyz[2] = 0.0;
    } else {
      mesh().vertex_at(i).data().xyz[0] = uv[2 * defrag_map[i]];
      mesh().vertex_at(i).data().xyz[1] = uv[2 * defrag_map[i] + 1];
      mesh().vertex_at(i).data().xyz[2] = 0.0;
    }

  }

}

std::vector<int> Mesh_modifier::get_top_k_errors_edge_vertices(int k) {
  size_t count = k;
  if (this->errors.size() < k) { count = this->errors.size(); }

  std::vector<int> results;
  results.reserve(count);
  std::priority_queue<Edge_distance, std::vector<Edge_distance>, cmp> tmpQ = this->errors;

  while (!tmpQ.empty() && count > 0) {
    Edge_distance top = tmpQ.top();
    if (top.he.is_active()) {
      results.push_back(top.he.origin().index());
      results.push_back(top.he.dest().index());
    }
    tmpQ.pop();
    count--;
  }
  return results;
}

void Mesh_modifier::set_anchor_vertex(int id) {
  this->_anchor_id = id;
}

Eigen::Matrix3Xd Mesh_modifier::deform(int deform_id, const Eigen::Vector3d &pos) {
//  force_assert(_anchor_id != -1 && _anchor_id != -3);
  int N = mesh().n_total_vertices();
  Eigen::Matrix3Xd pos_deformed(3, N);
  Eigen::Matrix3Xd free_pos_deformed(3, N-2);

  // Initialize all Rotations to be identity
  std::vector<Eigen::Matrix3d> m_rotation;
  m_rotation.reserve(N);
  for (int i = 0; i < N; i++) {
    Eigen::Matrix3d id_rotation = Eigen::Matrix3d::Identity();
    m_rotation.push_back(id_rotation);
  }

  // free index;
  int free_index = 0;
  // For building the index maps
  std::unordered_map<int, int> free_to_original;
  std::unordered_map<int, int> original_to_free;
  for (int i=0; i<N; ++i) {
    if (i != _anchor_id && i != deform_id) {
      free_to_original[free_index] = i;
      original_to_free[i] = free_index;
      free_index++;
    }
  }

  // Building the L matrix and the solver
  Eigen::SparseMatrix<double> L(N, N-2);
  std::vector<Eigen::Triplet<double>> L_elem;
  L_elem.reserve(N * (N-2));
  for (int i = 0; i < N; ++i) {
    Mesh_connectivity::Vertex_ring_iterator ring = mesh().vertex_ring_at(i);
    double L_sum = 0.0;
    do {
      int neighbour_id = ring.half_edge().origin().index();
      double w_ij = _cot_weights[ring.half_edge().index()];
      L_sum += w_ij;

      // handle neighbour case
      if (neighbour_id != _anchor_id && neighbour_id != deform_id) {
        L_elem.emplace_back(i, original_to_free[neighbour_id], -w_ij);
      }
    } while (ring.advance());
    // handle ith vertex case
    if (i != _anchor_id && i != deform_id) {
      L_elem.emplace_back(i, original_to_free[i], L_sum);
    }
  }

  L.setFromTriplets(L_elem.begin(), L_elem.end());
  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
  solver.compute(L.transpose() * L);
  if (solver.info() != Eigen::Success) {
    std::cout << "Decomposition failed!" << std::endl;
    return pos_deformed;
  }

  // Set the number of iterations
  int num_iter = 4;
  int itr = 0;
  while (itr < num_iter) {
    if (itr > 0) update_rotations(m_rotation, pos_deformed);

    // Update the B
    Eigen::MatrixX3d B(N, 3);
    for (int i = 0; i < N; ++i) {
      Eigen::Vector3d sum(0.0f, 0.0f, 0.0f);
      double L_sum = 0.0;
      Mesh_connectivity::Vertex_ring_iterator ring = mesh().vertex_ring_at(i);
      do {
        double w_ij = _cot_weights[ring.half_edge().index()];
        int neighbour_id = ring.half_edge().origin().index();
        L_sum += w_ij;
        // For normal right side
        sum += w_ij * 0.5
            * (m_rotation[ring.half_edge().dest().index()] + m_rotation[ring.half_edge().origin().index()])
            * (ring.half_edge().dest().xyz() - ring.half_edge().origin().xyz());

        // handle neighbour case
        if ( neighbour_id == _anchor_id) {
          sum += w_ij * mesh().vertex_at(_anchor_id).xyz();
        } else if ( neighbour_id == deform_id) {
          sum += w_ij * pos;
        }
      } while (ring.advance());

      // handle ith case
      if ( i == _anchor_id) {
        sum -= L_sum * mesh().vertex_at(_anchor_id).xyz();
      } else if ( i == deform_id) {
        sum -= L_sum * pos;
      }

      B.row(i) = sum;
    }

    // Solve for P_prime
    free_pos_deformed = solver.solve(L.transpose() * B).transpose();

    // Update deformed in non-fragmented way
    for (int i=0; i<N-2; ++i) {
      pos_deformed.col(free_to_original[i]) = free_pos_deformed.col(i);
    }
    pos_deformed.col(_anchor_id) = mesh().vertex_at(_anchor_id).xyz();
    pos_deformed.col(deform_id) = pos;
    itr++;
  }
  return pos_deformed;
}

double Mesh_modifier::cot(Mesh_connectivity::Half_edge_iterator he) {
  double result = 0.0;

  if (!he.face().is_equal(mesh().hole())) {
    Eigen::Vector3d CA = he.origin().xyz() - he.next().dest().xyz();
    Eigen::Vector3d CB = he.dest().xyz() - he.next().dest().xyz();

    result += CA.dot(CB) / CA.cross(CB).norm();
  }

  Mesh_connectivity::Half_edge_iterator twin = he.twin();

  if (!twin.face().is_equal(mesh().hole())) {
    Eigen::Vector3d DB = twin.origin().xyz() - twin.next().dest().xyz();
    Eigen::Vector3d DA = twin.dest().xyz() - twin.next().dest().xyz();

    result += DB.dot(DA) / DB.cross(DA).norm();
  }

  result *= 0.5;
  return result;
}

void Mesh_modifier::build_weights() {
  int M = mesh().n_total_half_edges();
  _cot_weights.reserve(M);
  for (int i = 0; i < M; ++i) {
    if (mesh().half_edge_at(i).twin().index() < i) {
      _cot_weights.push_back(_cot_weights[mesh().half_edge_at(i).twin().index()]);
    }
    _cot_weights.push_back(cot(mesh().half_edge_at(i)));
  }
}

void Mesh_modifier::build_L() {

}

void Mesh_modifier::update_rotations(std::vector<Eigen::Matrix3d> &m_rots, Eigen::Matrix3Xd &deformed_pos) {
  for (int i = 0; i < mesh().n_total_vertices(); ++i) {
    std::vector<Mesh_connectivity::Vertex_iterator> neighbours;
    Mesh_connectivity::Vertex_ring_iterator ring = mesh().vertex_ring_at(i);
    do {
      neighbours.push_back(ring.half_edge().origin());
    } while (ring.advance());

    int num_neighbour = (int) neighbours.size();
    Eigen::MatrixXd PPrime = Eigen::MatrixXd::Zero(3, num_neighbour);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, num_neighbour);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_neighbour, num_neighbour);

    for (int j = 0; j < num_neighbour; ++j) {
      PPrime.col(j) = deformed_pos.col(i) - deformed_pos.col(neighbours[j].index());
      P.col(j) = mesh().vertex_at(i).xyz() - neighbours[j].xyz();
      D(j, j) = _cot_weights[get_halfedge_between_vertices(i, neighbours[j].index())];
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(P * D * PPrime.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd rotation = svd.matrixV() * svd.matrixU().transpose();

    if (rotation.determinant() < 0) {
      Eigen::MatrixXd svd_u = svd.matrixU();
      svd_u.rightCols(1) = svd_u.rightCols(1) * -1;
      rotation = svd.matrixV() * svd_u.transpose();
    }

    assert(rotation.determinant() > 0);
    m_rots[i] = rotation;
  }
}

} // end of mohe
} // end of minimesh
