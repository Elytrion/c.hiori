#include "pch.h"
#include "fractureWorld.h"

using namespace chiori;

int cFractureWorld::MakeFracturable(int inActorIndex, cFractureMaterial inMaterial)
{
	// check if this actor is already fracturable
	for (int i = 0; i < f_fractors.size(); i++)
	{
		const cFracturable* fractor = f_fractors[i];
		if (fractor->actorIndex == inActorIndex)
			return -1; // invalid make 
	}
	// it doesnt exist, make new fractor
	cFracturable* n_fractor = f_fractors.Alloc();
	n_fractor->actorIndex = inActorIndex;
	n_fractor->f_material = inMaterial;
	return f_fractors.getIndex(n_fractor);
}

void cFractureWorld::SetFracturePattern(int inPatternIndex, int inFractorIndex)
{
	cassert(f_patterns.isValid(inPatternIndex));
	cassert(f_fractors.isValid(inFractorIndex));

	cFracturable* fractor = f_fractors[inFractorIndex];
	fractor->patternIndex = inPatternIndex;
}

int cFractureWorld::CreateNewFracturePattern( const cVoronoiDiagram& inDiagram, const cAABB& inBounds)
{
	if (CheckDupePattern(inDiagram))
		return -1; // duplicate
	cFracturePattern* n_pattern = f_patterns.Alloc();
	bool success =
		CreateFracturePattern(*n_pattern, inDiagram, inBounds);
	if (!success)
		return -1;
	return f_patterns.getIndex(n_pattern);
}

bool cFractureWorld::CreateFracturePattern(
	cFracturePattern& outPattern, const cVoronoiDiagram& inDiagram, const cAABB& inBounds, bool shift)
{
	if (inBounds.perimeter() <= 0)
	{
		outPattern.pattern = inDiagram;
		return true; // save the entire pattern
	}
	
	cVoronoiDiagram& vd = outPattern.pattern;
	cVec2 center = inBounds.getCenter();
	std::unordered_map<unsigned, unsigned> edgeVertMap;
	std::unordered_set<unsigned> keptEdges;
	std::unordered_set<cVec2, cVec2Hash> keptPoints;
	std::unordered_set<cVec2, cVec2Hash> finalKeptPoints;
	for (int i = 0; i < inDiagram.vertices.size(); ++i)
	{
		const cVVert& vt = inDiagram.vertices[i];
		if (!inBounds.contains(vt.site))
			continue; // ignore verts outside the bounds
		cVec2 localPos = vt.site - center; // Transform to local space
		cVVert newVt = vt;
		if (shift)
			newVt.site = localPos;
		vd.vertices.push_back(newVt);
		unsigned index = vd.vertices.size() - 1;
		
		for (unsigned edge : vt.edgeIndices) // add connected edges
		{
			keptEdges.insert(edge);
			edgeVertMap[edge] = index;
		}
	}

	for (const unsigned& i : keptEdges)
	{	
		cVEdge vedge = inDiagram.edges[i];
		cVVert& vt = vd.vertices[edgeVertMap[i]];

		int index = -1;
		for (int j = 0; j < vt.edgeIndices.size(); ++j)
		{
			if (vt.edgeIndices[j] == i)
			{
				index = j;
				break;
			}
		}
		cassert(index >= 0);
		if (shift)
		{
			vedge.origin -= center;
			if (!vedge.infinite)
				vedge.endDir -= center;
		}
		
		vd.edges.push_back(vedge);
		vt.edgeIndices[index] = vd.edges.size() - 1;
	}
	
	for (unsigned i = 0; i < inDiagram.v_points.size(); i++) {
		if (!inBounds.contains(inDiagram.v_points[i]))
			continue;
		keptPoints.insert(inDiagram.v_points[i]);
	}

	for (const auto& tri : inDiagram.triangles)
	{
		for (int i = 0; i < 3; ++i)
		{
			if (keptPoints.count(tri[i]))
			{
				finalKeptPoints.insert(tri[0]);
				finalKeptPoints.insert(tri[1]);
				finalKeptPoints.insert(tri[2]);
				break;
			}
		}
	}

	for (auto& p : finalKeptPoints)
	{		
		vd.v_points.push_back(p);
		if (shift)
			vd.v_points[vd.v_points.size() - 1] -= center;
	}
	
	vd.triangles = cVoronoiDiagram::triangulateDelaunator(vd.v_points);

	return true;
}

void cFractureWorld::f_step(float inFDT, int primaryIterations, int secondaryIterations, bool warmStart)
{
	step(inFDT, primaryIterations, secondaryIterations, warmStart);
}