//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Ricardo Lopez Lopez (a.k.a Ric92)
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef MICO_DNN_UTILS_CONVEXPOLYHEDRON_H_
#define MICO_DNN_UTILS_CONVEXPOLYHEDRON_H_

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include <mico/dnn/utils/Facet.h>
namespace mico {
class ConvexPolyhedron
{
public:
  ConvexPolyhedron();

  std::unordered_map<std::string, std::shared_ptr<Facet>> getFacets();
  void setFacets(std::unordered_map<std::string, std::shared_ptr<Facet>> _facets);

  std::vector<Eigen::Vector3f> getVertices();
  void setVertices(std::vector<Eigen::Vector3f> _vertices);

  float getVolume();
  void setVolume(float _volume);

  void clipConvexPolyhedron(std::shared_ptr<ConvexPolyhedron> _convexPolyhedron, std::vector<Eigen::Vector3f> &_intersectionPoints);

  float computeVolumeFromPoints(std::vector<Eigen::Vector3f> _points);

private:
  void clipSegmentFacets(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedronFacets,
                         std::pair<Eigen::Vector3f, Eigen::Vector3f> _segment, std::vector<Eigen::Vector3f> &_output);

  bool isInsidePolyhedron(std::unordered_map<std::string, std::shared_ptr<Facet>> _polyhedron, Eigen::Vector3f _point);

  float distanceToPlane(Eigen::Vector4f _plane, Eigen::Vector3f _point);

  // Facets
  std::unordered_map<std::string, std::shared_ptr<Facet>> mFacets;

  //Vertex
  std::vector<Eigen::Vector3f> mVertices;

  //Volume
  float mVolume;
  
};
} // namespace mico

#include <mico/dnn/utils/ConvexPolyhedron.inl>

#endif //MICO_DNN_UTILS_CONVEXPOLYHEDRON_H_