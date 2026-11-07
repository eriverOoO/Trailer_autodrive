#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>
#include <iostream>
#include <limits>
#include <cmath>
// Inspection only; no change to the FBX. Bounds apply the exact XML visual pose.
int main(int argc,char**argv) {
 if(argc!=3)return 2;
 Assimp::Importer loader;
 auto scene=loader.ReadFile(argv[1],aiProcess_Triangulate|aiProcess_PreTransformVertices);
 if(!scene){std::cerr<<loader.GetErrorString();return 1;}
 bool body=std::string(argv[2])=="body";
 float lo[3]={1e9,1e9,1e9},hi[3]={-1e9,-1e9,-1e9};unsigned vertices=0,faces=0;
 for(unsigned i=0;i<scene->mNumMeshes;i++) {
  auto m=scene->mMeshes[i];vertices+=m->mNumVertices;faces+=m->mNumFaces;
  for(unsigned j=0;j<m->mNumVertices;j++) {
   auto p=m->mVertices[j];
   // body Rz(90)*Rx(90); wheel Rz(90), scale 0.01
   float v[3]={body?p.z*.01f:-p.y*.01f,p.x*.01f,body?p.y*.01f:p.z*.01f};
   for(int k=0;k<3;k++){lo[k]=std::min(lo[k],v[k]);hi[k]=std::max(hi[k],v[k]);}
  }
 }
 std::cout<<"{\"file\":\""<<argv[1]<<"\",\"vertices\":"<<vertices<<",\"triangles\":"<<faces<<",\"min\":["<<lo[0]<<","<<lo[1]<<","<<lo[2]<<"],\"max\":["<<hi[0]<<","<<hi[1]<<","<<hi[2]<<"],\"textures\":[";
 bool first=true;
 for(unsigned i=0;i<scene->mNumMaterials;i++) {
  aiString name;if(scene->mMaterials[i]->GetTexture(aiTextureType_DIFFUSE,0,&name)==AI_SUCCESS) {
   if(!first)std::cout<<",";first=false;
   std::string s=name.C_Str();for(auto&c:s)if(c=='\\')c='/';
   std::cout<<"\""<<s<<"\"";
  }
 }
 std::cout<<"]}\n";
}
