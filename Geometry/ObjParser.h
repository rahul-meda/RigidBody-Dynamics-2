
#pragma once

#include <iostream>
#include "Mesh.h"

struct ObjFace
{
	int id;
	std::vector<int> vids;
};

// parse obj data into half-edge format
bool ParseObj(const std::string& file, HMesh& mesh);

bool ParseObj(const std::string& file, std::vector<HMesh>& meshes);

// check for bad/invalid file
bool Validate(const std::string& file);

void ParseLine(std::vector<ObjFace>& objFaces, HMesh& mesh, const std::string& line, bool& split, bool& first, int offset = 0);

void AddFace(HMesh& mesh, const std::vector<int>& vids);

// connect edges to their corresponding twin edges
void ConnectTwins(HMesh& mesh);

// sort edges into two halves based on direction
// second half has twins
void SortEdges(HMesh& mesh);