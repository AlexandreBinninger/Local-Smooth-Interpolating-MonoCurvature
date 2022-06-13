// This file is part of libhedra, a library for polyhedral mesh processing
//
// Copyright (C) 2018 Amir Vaxman <avaxman@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef HEDRA_LINEAR_CC_SUBDIVISION_H
#define HEDRA_LINEAR_CC_SUBDIVISION_H
#include <igl/igl_inline.h>
#include <hedra/vertex_valences.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <cstdio>

namespace hedra
{
  
  class LinearCCSubdivisionData:public OneRingSubdivisionData{
  public:
    Eigen::MatrixXd centers;
    Eigen::VectorXd radii;
    
    void setup(const Eigen::MatrixXd& _V, const Eigen::VectorXi& _D, const Eigen::MatrixXi& _F){
      V=_V; D=_D; F=_F;
      hedra::polygonal_edge_topology(D, F, EV, FE, EF, EFi, FEs, innerEdges);
      hedra::dcel(D, F, EV,EF,EFi,innerEdges,VH,EH,FH,HV,HE,HF,nextH, prevH,twinH);
      hedra::vertex_stars(EV,VH,EH,FH,HV,HE,HF,nextH,prevH,twinH,vertexValences,starVertices,starHalfedges,ringFaces,isBoundaryVertex);
    }
    //there is no special canonical forms for linear subdivision
    Eigen::MatrixXd original2Canonical(const int, const Eigen::MatrixXd& origPoints){return origPoints;}
    Eigen::MatrixXd canonical2Original(const int, const Eigen::MatrixXd& origPoints){return origPoints;}
    Eigen::MatrixXd threePointsExtrapolation(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b , const Eigen::MatrixXd& c){
      return c*2.0-b;
    }
    Eigen::MatrixXd boundaryEdgePoint(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, const Eigen::MatrixXd& c, const Eigen::MatrixXd& d){
      return (b+c)/2.0;
    }
    Eigen::MatrixXd boundaryVertexPoint(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, const Eigen::MatrixXd& p, const Eigen::MatrixXd& c, const Eigen::MatrixXd& d){
      return ((a+d).array()/8.0+p.array()*3.0/4.0);
    }
    
    Eigen::MatrixXd fourPointsInterpolation(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, const Eigen::MatrixXd& c, const Eigen::MatrixXd& d){
      return (b+c)/2.0;
    }
    Eigen::MatrixXd facePointBlend(const Eigen::MatrixXd& candidateFacePoints)
    {
      
      Eigen::MatrixXd fineFacePoints=Eigen::MatrixXd::Zero(F.rows(),3);
      for (int i=0;i<D.rows();i++){
        for (int j=0;j<D(i);j++)
          fineFacePoints.row(i)+=candidateFacePoints.block(i,3*j,1,3);
        
        fineFacePoints.row(i)/=(double)D(i);
      }
      return fineFacePoints;
    }
    
    /*Eigen::MatrixXd EdgePointBlend(const Eigen::MatrixXd& candidateEdgePoints)
     {
     //these points are anyhow equal and this is also the boundary formula from the vertex stars function, so no special treatment needed.
     return(candidateEdgePoints.block(0,0,candidateEdgePoints.rows(),3)+candidateEdgePoints.block(0,3,candidateEdgePoints.rows(),3))/2.0;
     }*/
    
    Eigen::RowVector3d innerVertexCanonicalBlend(const Eigen::RowVector3d& canonCenter,
                                                 const Eigen::MatrixXd& canonEdgePoints,
                                                 const Eigen::MatrixXd& canonFacePoints)
    {
      
      Eigen::RowVector3d F = canonFacePoints.colwise().mean();
      Eigen::RowVector3d E = canonEdgePoints.colwise().mean();
      return((F+E*4.0-F*2.0+(double)(canonEdgePoints.rows()-3)*canonCenter)/(double)canonEdgePoints.rows());
    }
    
    Eigen::MatrixXd canonicalEdgePoints(const int v0,
                                        const Eigen::RowVector3d& canonCenter,
                                       const Eigen::MatrixXd& canonStarVertices,
                                       const Eigen::MatrixXd& canonFacePoints)
    {
      Eigen::MatrixXd canonEdgePoints(vertexValences(v0),3);
      for (int j=isBoundaryVertex(v0);j<vertexValences(v0)-isBoundaryVertex(v0);j++)
        canonEdgePoints.row(j)=(canonCenter+canonStarVertices.row(j)+canonFacePoints.row(j)+canonFacePoints.row((j+vertexValences(v0)-1)%vertexValences(v0)))/4.0;
      
      return canonEdgePoints;
   
    }
    
    LinearCCSubdivisionData(){}
    ~LinearCCSubdivisionData(){}
    
  };
  
}


#endif


