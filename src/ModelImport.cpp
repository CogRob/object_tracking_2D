/**
 * \file ModelImport.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2015-4-8
 *
 * \copyright
 *
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "object_tracking_2D/ModelImport.h"

#ifdef USE_ASSIMP

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <assimp/Importer.hpp>
#include <assimp/mesh.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <stdexcept>
#include <set>
#include <vector>
#include <assert.h>

#include <iostream>

// Comparison operator for vector set
struct compareVectors
{
	bool operator() (const aiVector3D& s, const aiVector3D& r)
	{
		if (s.Length() < r.Length())
		{
			return true;
		}
		else if (s.Length() == r.Length())
		{
			if (s.x + s.y + s.z < r.x + r.y + r.z)
			{
				return true;
			}
			else if (s.x + s.y + s.z == r.x + r.y + r.z)
			{
				if (s.x < r.x)
				{
					return true;
				}
				else if(s.x == r.x)
				{
					if (s.y < r.y)
					{
						return true;
					}
					else if(s.y == r.y)
					{
						return s.z < r.z;
					}
					else
					{
						return false;
					}
				}
				else
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}
		return false;

	}
};

typedef std::set<aiVector3D, compareVectors> VectorSet;

GLMmodel* loadObject(const std::string& filename)
{
	// Setup import options
	Assimp::Importer importer;
	const int postprocessingFlags =
			aiProcess_JoinIdenticalVertices |
			aiProcess_GenNormals |
			aiProcess_ImproveCacheLocality |
			aiProcess_Triangulate |
			aiProcess_OptimizeGraph |
			aiProcess_OptimizeMeshes |
			//aiProcess_FindDegenerates |
			aiProcess_FixInfacingNormals |
			aiProcess_SortByPType;

	// Read the source file
	const aiScene* pScene = importer.ReadFile(filename.c_str(),
											  aiProcess_ValidateDataStructure |
											  postprocessingFlags);

	if (NULL == pScene)
	{
		throw std::runtime_error(importer.GetErrorString());
		return NULL;
	}

	// Find a triangle mesh
	int triMeshIdx = 0;
	for (int i = 0; i < pScene->mNumMeshes; i++)
	{
		if (pScene->mMeshes[i]->HasFaces() && pScene->mMeshes[i]->mFaces[0].mNumIndices == 3)
		{
			triMeshIdx = i;
		}
	}

	aiMesh* pMesh = pScene->mMeshes[triMeshIdx];
	if (NULL == pMesh)
	{
		throw std::runtime_error("No mesh found in model file");
		return NULL;
	}

	GLMmodel* model = new GLMmodel();

	// Get the set of unique vertices
	VectorSet vertices;

	VectorSet::const_iterator insertResult;
	std::vector<VectorSet::iterator> indexToSetIterator;
	std::vector<int> indexMap;

	//NB: do this twice, iterators move around on insert
	for (int i = 0; i < pMesh->mNumVertices; i++)
	{
		vertices.insert(pMesh->mVertices[i]);
	}

	for (int i = 0; i < pMesh->mNumVertices; i++)
	{
		insertResult = vertices.find(pMesh->mVertices[i]);
		if (vertices.end() == insertResult) { throw std::runtime_error("Unmatched vertex in set."); }
		indexMap.push_back(std::distance(vertices.begin(), insertResult));
		indexToSetIterator.push_back(insertResult);
	}

	// Resize the vertex array
	model->numvertices = vertices.size();
	model->vertices = new GLfloat[3*model->numvertices + 1]; // These +1's are a legacy from the original import code...

	// Copy the vertex data into the model
	int count = 0;
	VectorSet::iterator firstVertex = vertices.begin(), lastVertex = vertices.end();
	for (VectorSet::iterator iter = firstVertex; iter != lastVertex; ++iter)
	{
		model->vertices[3*count + 0] = iter->x;
		model->vertices[3*count + 1] = iter->y;
		model->vertices[3*count + 2] = iter->z;
		++count;
	}

	assert(NULL != pMesh->mFaces);
	assert(pMesh->HasNormals());

	model->numtriangles = pMesh->mNumFaces;
	model->triangles = new GLMtriangle[model->numtriangles + 1];

	for (int i = 0; i < pMesh->mNumFaces; ++i)
	{
		if (3 != pMesh->mFaces[i].mNumIndices)
		{
			throw std::runtime_error("Incorrect number of vertices for face.");
		}

		GLMtriangle& tri = model->triangles[i];
		tri.findex = i;

		for (int j = 0; j < 3; ++j)
		{
			uint originalIdx = pMesh->mFaces[i].mIndices[j];
			int shiftedIdx = std::distance(vertices.begin(), indexToSetIterator[originalIdx]);

			tri.vindices[j] = shiftedIdx;
		}
	}

	glmFacetNormals(model);          // (re)calculate face normals
    glmVertexNormals(model, 90.0);   // (re)calculate vertex normals

	return model;
}

#endif // USE_ASSIMP
