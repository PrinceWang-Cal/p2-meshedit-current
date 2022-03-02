#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
Vector2D lerp(Vector2D point_a, Vector2D point_b, double t) {
    return ((1.0 - t) * point_a) + (t * point_b);
}

  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      
      int num_points = points.size();
      if (num_points == 1) {
          return points;
      } else {
          std::vector<Vector2D> next_level_points = vector<Vector2D>();
          for (int i = 0; i < num_points - 1; i++) {
              Vector2D point_a = points[i];
              Vector2D point_b = points[i+1];
              Vector2D new_point = lerp(point_a, point_b, t);
              next_level_points.push_back(new_point);
          }
          return next_level_points;
      }
    
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */

Vector3D lerp(Vector3D point_a, Vector3D point_b, double t) {
    return ((1.0 - t) * point_a) + (t * point_b);
}

  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      int num_points = points.size();
      if (num_points == 1) {
          return points;
      } else {
          std::vector<Vector3D> next_level_points = vector<Vector3D>();
          for (int i = 0; i < num_points - 1; i++) {
              Vector3D point_a = points[i];
              Vector3D point_b = points[i+1];
              Vector3D new_point = lerp(point_a, point_b, t);
              next_level_points.push_back(new_point);
          }
          return next_level_points;
      }
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      int num_points = points.size();
      if (num_points == 1) {
          return points[0];
      } else {
          std::vector<Vector3D> next_level_points = points;
          while (next_level_points.size() > 1) {
              next_level_points = evaluateStep(next_level_points, t);
          }
          return next_level_points[0];
      }
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {
      std::vector<Vector3D> control_line_points = vector<Vector3D>();
      for (int i = 0; i < controlPoints.size(); i++) {
          Vector3D v_point = evaluate1D(controlPoints[i], u);
          control_line_points.push_back(v_point);
      }
      return evaluate1D(control_line_points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      
    //store the total area of all neighboring faces
      double A_total = 0.;
      Size num_neighbor_faces = degree();
      HalfedgeCIter curr_half_edge = halfedge();
      
      Vector3D vertex_normal = Vector3D();
      
      for (int i = 0; i < num_neighbor_faces; i++) {
          Vector3D curr_face_v1 = curr_half_edge->vertex()->position;
          Vector3D curr_face_v2 = curr_half_edge->next()->vertex()->position;
          Vector3D curr_face_v3 = curr_half_edge->next()->next()->vertex()->position;
          
          //compute cross product
          Vector3D cross_prod = cross(curr_face_v1, curr_face_v2);
          double area = cross_prod.norm()/2;
          Vector3D face_norm = curr_half_edge->face()->normal();
          
          vertex_normal += face_norm / area;
      }
      vertex_normal.normalize();
      return vertex_normal;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      
      //return if either faces of the edge is on the boundary
    if (e0->isBoundary()) {
              return e0;
    } else {
        
        //get all half edges
        
        
            
        //all half edges on the left triangle to the edge
        HalfedgeIter h_bc = e0->halfedge();
        HalfedgeIter h_ca = h_bc->next();
        HalfedgeIter h_ab = h_ca->next();
        
        //all half edges on the right triangle to the edge
        HalfedgeIter h_cb = h_bc->twin();
        HalfedgeIter h_bd = h_cb->next();
        HalfedgeIter h_dc = h_bd->next();
        
        //all other half edges needed to be modified on the other side
        HalfedgeIter h_ac = h_ca->twin();
        HalfedgeIter h_ba = h_ab->twin();
        HalfedgeIter h_db = h_bd->twin();
        HalfedgeIter h_cd = h_dc->twin();
        
        //get the four vertices in the two triangles
        VertexIter b = h_bc->vertex();
        VertexIter c = h_cb->vertex();
        VertexIter a = h_ab->vertex();
        VertexIter d = h_dc->vertex();
        
        //get the edges
        EdgeIter e_ca = h_ca->edge();
        EdgeIter e_ab = h_ab->edge();
        EdgeIter e_bd = h_bd->edge();
        EdgeIter e_dc = h_dc->edge();
        
        EdgeIter e_bc = e0;
        
        FaceIter f_abc = h_bc->face(); //f_adb after flip
        FaceIter f_bcd = h_cb->face(); //f_acd after flip
        
        // void setNeighbors( HalfedgeIter next,
        // HalfedgeIter twin,
        // VertexIter vertex,
        // EdgeIter edge,
        // FaceIter face )
        
        h_bc->setNeighbors(h_ca, h_cb, d, e_bc, f_abc); //h_da
        h_ca->setNeighbors(h_ab, h_ba, a, e_ab, f_abc);
        h_ab->setNeighbors(h_bc, h_db, b, e_bd, f_abc);
        
        h_cb->setNeighbors(h_bd, h_bc, a, e_bc, f_bcd);
        h_bd->setNeighbors(h_dc, h_cd, d, e_dc, f_bcd);
        h_dc->setNeighbors(h_cb, h_ac, c, e_ca, f_bcd);
        
        h_ac->setNeighbors(h_ac->next(), h_dc, a, e_ca, h_ac->face());
        h_ba->setNeighbors(h_ba->next(), h_ca, b, e_ab, h_ba->face());
        h_db->setNeighbors(h_db->next(), h_ab, d, e_bd, h_db->face());
        h_cd->setNeighbors(h_cd->next(), h_bd, c, e_dc, h_cd->face());
        
        b->halfedge() = h_ab;
        c->halfedge() = h_dc;
        a->halfedge() = h_cb;
        d->halfedge() = h_bc;
        
        e_bc->halfedge() = h_bc;
        e_ca->halfedge() = h_dc;
        e_ab->halfedge() = h_ca;
        e_bd->halfedge() = h_ab;
        e_dc->halfedge() = h_bd;
        
        f_abc->halfedge() = h_bc;
        f_bcd->halfedge() = h_cb;
        
        return e_bc;
        
    }
    
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary()) {
        return e0->halfedge()->vertex();
      } else {
          
          //all half edges on the left triangle to the edge
          HalfedgeIter h_bc = e0->halfedge();
          HalfedgeIter h_ca = h_bc->next();
          HalfedgeIter h_ab = h_ca->next();
          
          //all half edges on the right triangle to the edge
          HalfedgeIter h_cb = h_bc->twin();
          HalfedgeIter h_bd = h_cb->next();
          HalfedgeIter h_dc = h_bd->next();
          
          //all other half edges needed to be modified on the other side
          HalfedgeIter h_ac = h_ca->twin();
          HalfedgeIter h_ba = h_ab->twin();
          HalfedgeIter h_db = h_bd->twin();
          HalfedgeIter h_cd = h_dc->twin();
          
          //create six new half edges
          HalfedgeIter h0 = newHalfedge();
          HalfedgeIter h1 = newHalfedge();
          HalfedgeIter h2 = newHalfedge();
          HalfedgeIter h3 = newHalfedge();
          HalfedgeIter h4 = newHalfedge();
          HalfedgeIter h5 = newHalfedge();
          
          //get the four vertices in the two triangles
          VertexIter b = h_bc->vertex();
          VertexIter c = h_cb->vertex();
          VertexIter a = h_ab->vertex();
          VertexIter d = h_dc->vertex();
          VertexIter new_v = newVertex();
          
          //get the five original edges
          EdgeIter e_bc = e0;
          EdgeIter e_ca = h_ca->edge();
          EdgeIter e_ab = h_ab->edge();
          EdgeIter e_bd = h_bd->edge();
          EdgeIter e_dc = h_dc->edge();
          
          //create three new edges
          
          EdgeIter e1 = newEdge();
          EdgeIter e2 = newEdge();
          EdgeIter e3 = newEdge();
          
          
          
          //get_faces
          FaceIter f_abc = h_bc->face(); //f_adb after flip
          FaceIter f_bcd = h_cb->face(); //f_acd after flip
          //create two new faces
          FaceIter f0 = newFace();
          FaceIter f1 = newFace();
          
          new_v->position = (b->position + c->position) / 2.0;
          
          //reassignment
          h_bc->setNeighbors(h_ca, h_cb, new_v, e_bc, f_abc);
          h_ca->setNeighbors(h_ab, h_ac, c, e_ca, f_abc);
          h_ab->setNeighbors(h_bc, h1, a, e1, f_abc);
          
          h_cb->setNeighbors(h_bd, h_bc, c, e_bc, f_bcd);
          h_bd->setNeighbors(h_dc, h5, new_v, e3, f_bcd);
          h_dc->setNeighbors(h_cb, h_cd, d, e_dc, f_bcd);
          
          h_ac->setNeighbors(h_ac->next(), h_ca, a, e_ca, h_ac->face());
          h_ba->setNeighbors(h_ba->next(), h2, b, e_ab, h_ba->face());
          h_db->setNeighbors(h_db->next(), h4, d, e_bd, h_db->face());
          h_cd->setNeighbors(h_cd->next(), h_dc, c, e_dc, h_cd->face());
          
          h0->setNeighbors(h1, h3, b, e2, f0);
          h1->setNeighbors(h2, h_ab, new_v, e1, f0);
          h2->setNeighbors(h0, h_ba, a, e_ab, f0);
          
          h3->setNeighbors(h4, h0, new_v, e2, f1);
          h4->setNeighbors(h5, h_db, b, e_bd, f1);
          h5->setNeighbors(h3, h_bd, d, e3, f1);
          
          //reassign vertices
          new_v->halfedge() = h_bc;
          b->halfedge() = h0;
          c->halfedge() = h_ca;
          a->halfedge() = h2;
          d->halfedge() = h_dc;
          
          //reassign edges
          e_bc->halfedge() = h_bc;
          e_ca->halfedge() = h_ca;
          e_ab->halfedge() = h2;
          e_bd->halfedge() = h4;
          e_dc->halfedge() = h_dc;
                
          e1->halfedge() = h_ab;
          e2->halfedge() = h0;
          e3->halfedge() = h_bd;
          
          //faces
          f_abc->halfedge() = h_bc;
          f_bcd->halfedge() = h_cb;
          f0->halfedge() = h0;
          f1->halfedge() = h3;
          
          e1->isNew = true;
          e3->isNew = true;
          new_v->isNew = true;
          
          
          
          
          return new_v;
      }
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
      
      std::vector<VertexIter> old_vertices = vector<VertexIter>();
      
      //iterate over all vertices on the mesh
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          //update the old vertices' position
          
          Vector3D original_neighbor_position_sum = Vector3D();
          HalfedgeCIter curr_halfedge = v->halfedge();
          HalfedgeCIter curr_halfedge_twin;
          Size num_faces = v->degree();
          
          //store the old vertex
          old_vertices.push_back(v);
          
          for (int i = 0; i < num_faces; i++) {
              curr_halfedge_twin = curr_halfedge->twin();
              Vector3D neighbor_position = curr_halfedge_twin->vertex()->position;
              original_neighbor_position_sum += neighbor_position;
              curr_halfedge = curr_halfedge_twin->next();
          }
          
          double u;
          if (num_faces == 3) {
              u = (double) 3/16;
          } else {
              u = (double) 3 / (8*num_faces);
          }
          
          Vector3D original_position = v->position;
          
          v->newPosition = (1.0 - num_faces * u) * original_position + u * original_neighbor_position_sum;
          v->isNew = false;
          
      }
      

    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
          HalfedgeIter curr_halfedge = edge->halfedge();
          
          VertexIter A = curr_halfedge->vertex();
          VertexIter B = curr_halfedge->twin()->vertex();
          VertexIter C = curr_halfedge->next()->next()->vertex();
          VertexIter D = curr_halfedge->twin()->next()->next()->vertex();
          
          Vector3D A_pos = A->position;
          Vector3D B_pos = B->position;
          Vector3D C_pos = C->position;
          Vector3D D_pos = D->position;
          
          Vector3D updated_position = ((double) 3.0/8.0) * (A_pos + B_pos) + ((double) 1.0/8.0) * (C_pos + D_pos);
          edge->newPosition = updated_position;
          edge->isNew = false;
      }
      
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
      std::vector<EdgeIter> old_edges = vector<EdgeIter>();
      for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
          if (edge->isBoundary()) {
              continue;
          } else {
              old_edges.push_back(edge);
          }
      }
      Size num_old_edges = old_edges.size();
      for (int i = 0; i < num_old_edges; i++) {
          EdgeIter curr_edge = old_edges[i];
          
          //check the two vertices
          VertexIter v1 = curr_edge->halfedge()->vertex();
          VertexIter v2 = curr_edge->halfedge()->twin()->vertex();
          if (v1->isNew == false || v2->isNew == false) {
              VertexIter splited = mesh.splitEdge(curr_edge);
              splited->newPosition = curr_edge->newPosition;
          }
      }
      
    
    // 4. Flip any new edge that connects an old and new vertex.
      for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
          if (!(edge->isNew)) {
              continue;
          } else {
              //check the two vertices
              VertexIter v1 = edge->halfedge()->vertex();
              VertexIter v2 = edge->halfedge()->twin()->vertex();
              
              if ((v1->isNew == false && v2->isNew == true) || (v1->isNew == true && v2->isNew == false)) {
                  mesh.flipEdge(edge);
              }
          }
      }

    // 5. Copy the new vertex positions into final Vertex::position.
      for (VertexIter vertex = mesh.verticesBegin(); vertex != mesh.verticesEnd(); vertex++) {
          vertex->position = vertex->newPosition;
      }
      
      

  }
}
