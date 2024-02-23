#include "iostream"
#include <map>
#include <minimesh/core/mohe/mesh_modifier.hpp>
#include <minimesh/core/util/assert.hpp>
#include <unordered_set>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>

namespace minimesh {
namespace mohe {
Mesh_modifier::
Mesh_modifier(Mesh_connectivity &mesh_in)
    : _m(mesh_in) {}

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
  this->initialize_Qs();
  this->compute_errors();
}

void
Mesh_modifier::initialize_Qs() {
  for (int i = 0; i < mesh().n_active_vertices(); ++i) {
    Eigen::Matrix4d Q;
    Q.setZero();
    Mesh_connectivity::Vertex_ring_iterator curr = mesh().vertex_ring_at(i);
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

    this->Qs.push_back(Q);
  }
}

void
Mesh_modifier::compute_errors() {
  // Keep a O(1) HashSet for visited half-edge's twins
  std::unordered_set<int> visited;

  for (int i = 0; i < mesh().n_active_half_edges(); ++i) {
    // If not visited
    if (visited.find(i) == visited.end()) {
      visited.insert(mesh().half_edge_at(i).twin().index());

      // v1,v2
      Mesh_connectivity::Vertex_iterator v1 = mesh().half_edge_at(i).origin();
      Mesh_connectivity::Vertex_iterator v2 = mesh().half_edge_at(i).dest();

      // Calculate the Q hat for the two vertices
      Eigen::Matrix4d Q_hat = this->Qs[v1.index()] + this->Qs[v2.index()];

      // Set the linear system
      Eigen::Matrix4d Q_hat_d = Q_hat;
      Q_hat_d(3, 0) = 0;
      Q_hat_d(3, 1) = 0;
      Q_hat_d(3, 2) = 0;
      Q_hat_d(3, 3) = 1;
      Eigen::Vector4d b(0, 0, 0, 1.0);

      // Use FullPivLU to check invertibility first
      Eigen::FullPivLU<Eigen::Matrix4d> lu(Q_hat_d);
      Eigen::Vector4d v;

      // If invertible, use PLU to solve, faster, else use mid-point
      if (lu.isInvertible()) {
        Eigen::PartialPivLU<Eigen::Matrix4d> plu(Q_hat_d);
        v = plu.solve(b);
      } else {
        Eigen::Vector3d tmp = (v1.xyz() + v2.xyz()) / 2;
        v << tmp.head(3), 1.0;
      }
      // Calculate and push the error into the heap
      const double error = v.transpose() * Q_hat * v;
      Edge_distance edge_distance{mesh().half_edge_at(i), error};

      this->errors.push(edge_distance);
    }
  }
}

void
Mesh_modifier::update_Qs() {
  //Stub
}

bool
Mesh_modifier::check_validity(Mesh_connectivity::Half_edge_iterator he) {
  return true;
}

void Mesh_modifier::connect_with_neighbours() {

}

void
Mesh_modifier::simplify(int k) {
//  if (k >= (mesh().n_active_half_edges() / 2)) {
//    printf("Error! Stop simplifying.");
//    return;
//  }

  bool valid = true;
  while (k > 0 && valid) {
    Edge_distance to_pop = this->errors.top();
    Mesh_connectivity::Vertex_iterator v1 = to_pop.he.origin();
    Mesh_connectivity::Vertex_iterator v2 = to_pop.he.dest();
    std::vector<Mesh_connectivity::Vertex_iterator> v1_neighbours, v2_neighbours;

    Mesh_connectivity::Vertex_ring_iterator v1_ring = mesh().vertex_ring_at(v1.index());
    Mesh_connectivity::Vertex_ring_iterator v2_ring = mesh().vertex_ring_at(v2.index());

    do {
      v1_neighbours.push_back(v1.half_edge().origin());
    } while (v1_ring.advance());

    do {
      v2_neighbours.push_back(v2.half_edge().origin());
    } while (v2_ring.advance());


    if (check_validity(to_pop.he)) {
      this->errors.pop();
      to_pop.he.deactivate();
      this->update_Qs();
    } else {
      // TODO: Need to do some error handling
    }
    k--;
  }
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
  if (this->errors.size() < k) {count = this->errors.size();}

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
//  std::sort(results.begin(), results.end());
  return results;
}

} // end of mohe
} // end of minimesh
