#include "iostream"
#include <map>
#include <minimesh/core/mohe/mesh_modifier.hpp>
#include <minimesh/core/util/assert.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <unordered_set>
#include <algorithm>

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
  Mesh_connectivity::Vertex_iterator first_boundary_vertex{};
  std::vector<Mesh_connectivity::Half_edge_data> boundaries;
  std::vector<int> ind;
  ind.reserve(int(mesh().n_total_half_edges()/2));
  boundaries.reserve(mesh().n_active_half_edges());
  for (int i=0; i < mesh().n_total_half_edges(); ++i) {
    if (mesh().half_edge_at(i).face().is_equal(mesh().hole())) {
      boundaries.push_back(mesh().half_edge_at(i).data());
      ind.push_back(i);
    }
  }

}

void Mesh_modifier::parametrize_LSCM() {

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
} // end of mohe
} // end of minimesh
