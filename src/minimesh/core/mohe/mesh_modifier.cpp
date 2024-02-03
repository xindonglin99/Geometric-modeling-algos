#include <minimesh/core/mohe/mesh_modifier.hpp>
#include <minimesh/core/util/assert.hpp>

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

  // Loop through each vertex, compute its new position according to its neighbours
  for (int i=0; i < mesh().n_active_vertices(); i++) 
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
  for (int i=0; i < mesh().n_total_half_edges(); i++)
  {
      Mesh_connectivity::Half_edge_iterator curr_edge = mesh().half_edge_at(i);
      if (curr_edge.is_active())
      {
          Mesh_connectivity::Vertex_iterator vNew = newMesh.add_vertex(false);
          vNew.data().xyz = (curr_edge.origin().xyz() + curr_edge.dest().xyz()) * (3.0f / 8.0f);
          vNew.data().xyz += (curr_edge.next().dest().xyz() + curr_edge.twin().next().dest().xyz()) * (1.0f / 8.0f);
          curr_edge.twin().deactivate();
      }
  }

  // Add new faces and half edges
  for (int i=0; i < mesh().n_total_faces(); i++)
  {
      Mesh_connectivity::Half_edge_iterator he0 = mesh().face_at(i).half_edge();
      Mesh_connectivity::Half_edge_iterator he1 = he0.next();
      Mesh_connectivity::Half_edge_iterator he2 = he1.next();

      Mesh_connectivity::Vertex_iterator v0 = he0.origin();
      Mesh_connectivity::Vertex_iterator v1 = he1.origin();
      Mesh_connectivity::Vertex_iterator v2 = he2.origin();

      Mesh_connectivity::Vertex_iterator v0_new = mesh().vertex_at();
      Mesh_connectivity::Vertex_iterator v1_new = he1.origin();
      Mesh_connectivity::Vertex_iterator v2_new = he2.origin();
  }
}


} // end of mohe
} // end of minimesh
