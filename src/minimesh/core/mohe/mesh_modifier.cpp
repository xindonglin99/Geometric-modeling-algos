#include <minimesh/core/mohe/mesh_modifier.hpp>
#include <minimesh/core/util/assert.hpp>
#include <minimesh/core/util/macros.hpp>
#include "iostream"

namespace minimesh
{
namespace mohe
{


//
// Given two vertices, this function return the index of the half-edge going from v0 to v1.
// Returns -1 if no half-edge exists between the two vertices.
//
int Mesh_modifier::get_halfedge_between_vertices(const int v0, const int v1)
{
	// Get a ring iterator for v0
	Mesh_connectivity::Vertex_ring_iterator ring_iter = mesh().vertex_ring_at(v0);

	int answer = mesh().invalid_index;

	// Loop over all half-edges that end at v0.
	do
	{
		// Make sure that the half-edge does end at v0
		assert(ring_iter.half_edge().dest().index() == v0);

		// If the half-edge also starts and v1, then it's twin
		// goes from v0 to v1. This would be the half-edge that
		// we were looking for
		if(ring_iter.half_edge().origin().index() == v1)
		{
			answer = ring_iter.half_edge().twin().index();
		}
	} while(ring_iter.advance());

	if(answer != mesh().invalid_index)
	{
		assert(mesh().half_edge_at(answer).origin().index() == v0);
		assert(mesh().half_edge_at(answer).dest().index() == v1);
	}

	return answer;
}


bool Mesh_modifier::flip_edge(const int he_index)
{
	//
	// Take a reference to all involved entities
	//

	// HALF-EDGES
	Mesh_connectivity::Half_edge_iterator he0 = mesh().half_edge_at(he_index);
	Mesh_connectivity::Half_edge_iterator he1 = he0.twin();

	// meshes on the boundary are not flippable
	if(he0.face().is_equal(mesh().hole()) || he1.face().is_equal(mesh().hole()))
	{
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
float Mesh_modifier::compute_old_coeff(const int n)
{
  float beta;
	if (n > 3){
		float center = (0.375f + (0.25f * cos(6.2831853f / (float)n))); // 2.0f * 3.1415926f
		beta = (0.625f - (center * center)) / (float)n;
	}
	else {
		beta = 0.1875f; // 3.0f / 16.0f;
	}
	return beta;
}

void Mesh_modifier::subdivide() 
{
  Mesh_connectivity newMesh;
  Edge_map edge_map;

  // Loop through each vertex, compute its new position according to its neighbours
  for (int i=0; i < mesh().n_total_vertices(); i++)
  {
    Mesh_connectivity::Vertex_iterator vNew = newMesh.add_vertex(false);
    Mesh_connectivity::Vertex_ring_iterator curr = mesh().vertex_ring_at(i);
    const Mesh_connectivity::Half_edge_iterator firstEdge = curr.half_edge();
    std::vector<Mesh_connectivity::Vertex_iterator> neighbours;
    int k = 0;
    do 
    {
      neighbours.push_back(curr.half_edge().origin());
      curr.advance();
      k++;
    } while (!curr.half_edge().is_equal(firstEdge));
    float beta = compute_old_coeff(k);
    vNew.data().xyz = curr.half_edge().dest().xyz() * (1.0f - (float(k) * beta));
    for (int j = 0; j < k; j++)
    {
        vNew.data().xyz += (neighbours[j].data().xyz * beta);
    }
  }

  // Loop through all edges, compute new vertex
  std::map<int, int> he_newV_indices;
  for (int i=0; i < mesh().n_total_half_edges(); i++)
  {
      Mesh_connectivity::Half_edge_iterator curr_edge = mesh().half_edge_at(i);
      if (curr_edge.is_active())
      {
          Mesh_connectivity::Vertex_iterator vNew = newMesh.add_vertex(false);
          vNew.data().xyz = (curr_edge.origin().xyz() + curr_edge.dest().xyz()) * (3.0f / 8.0f);
          vNew.data().xyz += (curr_edge.next().dest().xyz() + curr_edge.twin().next().dest().xyz()) * (1.0f / 8.0f);
          he_newV_indices[curr_edge.index()] = vNew.index();
          curr_edge.twin().deactivate();
      }
  }

  // Debugging purpose
  for (int i=0; i < mesh().n_total_half_edges(); i++)
  {
      if (!mesh().half_edge_at(i).is_active()) {
          std::cout << mesh().half_edge_at(i).data().origin << std::endl;
      }
  }

  // Add new faces and half edges
  for (int i=0; i < mesh().n_total_faces(); i++)
  {
      // 3 half edges related to the face
      Mesh_connectivity::Half_edge_iterator he0 = mesh().face_at(i).half_edge();
      Mesh_connectivity::Half_edge_iterator he1 = he0.next();
      Mesh_connectivity::Half_edge_iterator he2 = he1.next();

      // 3 vertices related to the face
      Mesh_connectivity::Vertex_iterator v0 = he0.origin();
      Mesh_connectivity::Vertex_iterator v1 = he1.origin();
      Mesh_connectivity::Vertex_iterator v2 = he2.origin();

      // 3 newly added vertices corresponding to the 3 half edges
      Mesh_connectivity::Vertex_iterator v0_new = newMesh.vertex_at(he_newV_indices[he0.index()]);
      Mesh_connectivity::Vertex_iterator v1_new = newMesh.vertex_at(he_newV_indices[he1.index()]);
      Mesh_connectivity::Vertex_iterator v2_new = newMesh.vertex_at(he_newV_indices[he2.index()]);

      // Build half edges
      build_triangle(v0, v0_new, v2_new, newMesh, edge_map);
      build_triangle(v0_new, v1, v1_new, newMesh, edge_map);
      build_triangle(v0_new, v1_new, v2_new, newMesh, edge_map);
      build_triangle(v2_new, v1_new, v2, newMesh, edge_map);
  }
  std::cout << "The number of activate half edges in old mesh is" << mesh().n_total_half_edges() - mesh().n_active_half_edges() << std::endl;

  mesh().clear();
  mesh().copy(newMesh);

  std::cout << "-----HERE------" << std::endl;
}

void Mesh_modifier::build_triangle(Mesh_connectivity::Vertex_iterator & v0,
                                   Mesh_connectivity::Vertex_iterator & v1,
                                   Mesh_connectivity::Vertex_iterator & v2,
                                   Mesh_connectivity & mesh,
                                   Edge_map & edge_map)
{
    Mesh_connectivity::Half_edge_iterator he0 = mesh.add_half_edge(false);
    Mesh_connectivity::Half_edge_iterator he1 = mesh.add_half_edge(false);
    Mesh_connectivity::Half_edge_iterator he2 = mesh.add_half_edge(false);

    he0.data().origin = v0.index();
    he1.data().origin = v1.index();
    he2.data().origin = v2.index();

    he0.data().next = he1.index();
    he1.data().next = he2.index();
    he2.data().next = he0.index();

    he0.data().prev = he2.index();
    he1.data().prev = he0.index();
    he2.data().prev = he1.index();

    Mesh_connectivity::Face_iterator f = mesh.add_face(false);
    f.data().half_edge = he0.index();

    he0.data().face = f.index();
    he1.data().face = f.index();
    he2.data().face = f.index();

    v0.data().half_edge = he0.index();
    v1.data().half_edge = he1.index();
    v2.data().half_edge = he2.index();

    int n_polygon_vertices = 3;

    // Helpers
    auto get_origin = [](Edge_map::const_iterator in) -> int { return in->first.first; };
    auto get_dest = [](Edge_map::const_iterator in) -> int { return in->first.second; };
    auto get_half_edge = [](Edge_map::const_iterator in) -> int { return in->second; };
    auto create_edge_map_member = [](int beg, int end, int he) -> Edge_map::value_type
    {
        return std::make_pair(std::make_pair(beg, end), he);
    };
    MINIMESH_UNUSED(get_origin);

    std::vector<Mesh_connectivity::Vertex_iterator> vertices{v0, v1, v2};
    std::vector<Mesh_connectivity::Half_edge_iterator> half_edges{he0, he1, he2};


    for(int voffset = 0; voffset < n_polygon_vertices; ++voffset)
    {
        int voffsetp1 = (voffset + 1) % n_polygon_vertices;

        // try to find your twin
        auto it =
                edge_map.find(std::make_pair(vertices[voffsetp1].index(), vertices[voffset].index()));


        if((it != edge_map.end()))
        {
            assert(get_origin(it) == vertices[voffsetp1].index());
            assert(get_dest(it) == vertices[voffset].index());
            half_edges[voffset].data().twin = get_half_edge(it);
            mesh.half_edge_at(get_half_edge(it)).data().twin = half_edges[voffset].index();
            edge_map.erase(it);
        }
        else
        {
            edge_map.insert(create_edge_map_member(
                    vertices[voffset].index(), vertices[voffsetp1].index(), half_edges[voffset].index()));
        }
    }
}


} // end of mohe
} // end of minimesh
