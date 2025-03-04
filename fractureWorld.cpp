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

void cFractureWorld::MakeUnfracturable(int inFractorIndex)
{
	cassert(f_fractors.isValid(inFractorIndex));
	f_fractors.Free(f_fractors[inFractorIndex]);
}

void cFractureWorld::SetFracturePattern(int inPatternIndex, int inFractorIndex)
{
	cassert(f_patterns.isValid(inPatternIndex));
	cassert(f_fractors.isValid(inFractorIndex));

	cFracturable* fractor = f_fractors[inFractorIndex];
	fractor->patternIndex = inPatternIndex;
}

int cFractureWorld::CreateNewFracturePattern(const cVoronoiDiagram& inDiagram, const cAABB& inBounds)
{
	if (CheckDupePattern(inDiagram))
		return -1; // duplicate
	cFracturePattern* n_pattern = f_patterns.Alloc();
	bool success =
		CreateFracturePattern(*n_pattern, inDiagram, inBounds);
	if (!success)
		return -1; // failed clip
	return f_patterns.getIndex(n_pattern);
}

bool cFractureWorld::CreateFracturePattern(
	cFracturePattern& outPattern, const cVoronoiDiagram& inDiagram, const cAABB& inBounds, bool shift)
{
	if (inBounds.perimeter() <= 0)
	{
		outPattern.pattern = inDiagram;
		return true; // save the entire pattern if no bounds provided
	}

	// we expand the bounds to encapsulate all points that could end up in the provided
	// bounds if the center of those bounds was shifted to its edges
	cAABB exBounds = inBounds;
	exBounds.min = inBounds.min - (inBounds.getExtents());
	exBounds.max = inBounds.max + (inBounds.getExtents());

	cVoronoiDiagram& vd = outPattern.pattern;
	cVec2 center = exBounds.getCenter();
	std::unordered_map<unsigned, unsigned> edgeVertMap;
	std::unordered_set<unsigned> keptEdges;
	std::unordered_set<cVec2, cVec2Hash> keptPoints;
	std::unordered_set<cVec2, cVec2Hash> finalKeptPoints;
	for (int i = 0; i < inDiagram.vertices.size(); ++i)
	{
		const cVVert& vt = inDiagram.vertices[i];
		if (!exBounds.contains(vt.site))
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

		if (!exBounds.contains(vedge.origin) && !vedge.infinite)
		{
			std::swap(vedge.origin, vedge.endDir);
			cVec2 nDir = vedge.endDir - vedge.origin;
			vedge.endDir = nDir;
			vedge.infinite = true;
		}
		else if (!exBounds.contains(vedge.endDir) && !vedge.infinite)
		{
			cVec2 nDir = vedge.endDir - vedge.origin;
			vedge.endDir = nDir;
			vedge.infinite = true;
		}

		vd.edges.push_back(vedge);
		vt.edgeIndices[index] = vd.edges.size() - 1;
	}

	for (unsigned i = 0; i < inDiagram.v_points.size(); i++) {
		if (!exBounds.contains(inDiagram.v_points[i]))
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

	// fractor broadphase collision check, ignore all fractors unable to fracture this frame
	std::unordered_map<int, std::vector<int>> fractorContactsMap;
	for (int i = 0; i < f_fractors.size(); ++i)
	{
		cFracturable* fractor = f_fractors[i];
		cassert(p_actors.isValid(fractor->actorIndex));
		cActor* actor = p_actors[fractor->actorIndex];
		if (actor->contactCount < 1)
			continue; // no collision here, we continue

		// loop through all the contacts this fractor is in
		int contactKey = actor->contactList;
		while (contactKey != NULL_INDEX)
		{
			cContact* contact = p_contacts[(contactKey >> 1)];
			const cManifold& manifold = contact->manifold;
			if (manifold.pointCount > 0)
			{
				fractorContactsMap[i].push_back((contactKey >> 1));
			}
			contactKey = contact->edges[contactKey & 1].nextKey;
		}
	}

	// fractor narrowphase fracture check, check if a fracture is possible given the force of collision;
	for (const auto& [fractorID, contactIDs] : fractorContactsMap)
	{
		cassert(f_fractors.isValid(fractorID));
		cFracturable* fractor = f_fractors[fractorID];
		int aid = fractor->actorIndex;
		cassert(p_actors.isValid(aid));
		cActor* actor = p_actors[aid];
		cFractureMaterial& mat = fractor->f_material;

		const cAABB& actorAABB = GetActorAABB(aid);
		cVec2 extents = actorAABB.getExtents();
		float boundingRadius = extents.magnitude(); // AABB diagonal
		float estimatedThickness = c_min(extents.x, extents.y); // Use smallest dimension
		float estimatedMinArea = PI * pow(0.1f * c_max(extents.x, extents.y), 2); // Prevents zero area

		for (auto& cid : contactIDs)
		{
			const cContact* contact = p_contacts[cid];
			bool flip = (contact->edges[0].bodyIndex != aid);
			const cManifold& manifold = contact->manifold;
			float impactArea = -1.0f; // we need to determine fracture threshold
			cVec2 normalForce = cVec2::zero;
			cVec2 mp1impulse, mp2impulse;
			if (manifold.pointCount == 1)
			{
				const cManifoldPoint& mpt1 = manifold.points[0];
				impactArea = c_max(estimatedMinArea, PI * boundingRadius * boundingRadius);
				normalForce = mp1impulse = (mpt1.normalImpulse / inFDT) * manifold.normal;
			}
			else
			{
				cassert(manifold.pointCount == 2);
				const cManifoldPoint& mpt1 = manifold.points[0];
				const cManifoldPoint& mpt2 = manifold.points[1];
				float contactSpan = -1.0f;
				if (flip)
					contactSpan = distance(mpt1.localAnchorB, mpt2.localAnchorB);
				else
					contactSpan = distance(mpt1.localAnchorA, mpt2.localAnchorA);

				impactArea = contactSpan * estimatedThickness;
				mp1impulse = (mpt1.normalImpulse / inFDT) * manifold.normal;
				mp2impulse = (mpt2.normalImpulse / inFDT) * manifold.normal;
				normalForce = mp1impulse + mp2impulse;
			}
			float appliedStress = normalForce.magnitude() / impactArea;
			float fractureStress = (mat.elasticity / (1.0f + mat.brittleness)) * (1.0f / mat.toughness);
			// Adjust for anisotropy
			cVec2 impactDirection = normalForce.normalized();
			float angle = acos(dot(impactDirection, mat.anisotropy.normalized()));
			float anisotropyMultiplier = 1.0f + mat.anisotropyFactor * cos(angle);
			fractureStress *= anisotropyMultiplier;
			fractureStress *= mat.k;

			// Fracture condition check
			if (appliedStress >= fractureStress)
			{
				// we need to get the collision points in a sum for this fractor,
				// skewed via the normal forces
				float mp1mag = mp1impulse.magnitude();
				float mp2mag = mp2impulse.magnitude();
				if (manifold.pointCount == 1)
				{
					fractorPointsMap[fractorID] = (flip) ? manifold.points[0].localAnchorB : manifold.points[0].localAnchorA;
				}
				else
				{
					cassert(manifold.pointCount == 2);
					if (mp1mag < mp2mag)
						fractorPointsMap[fractorID] = (flip) ? manifold.points[1].localAnchorB : manifold.points[1].localAnchorA;
					else
						fractorPointsMap[fractorID] = (flip) ? manifold.points[0].localAnchorB : manifold.points[0].localAnchorA;
				}
			}
		}
	}

	for (const auto& [fractorID, fracturePoint] : fractorPointsMap)
	{
		cassert(f_fractors.isValid(fractorID));
		cFracturable* fractor = f_fractors[fractorID];
		int aid = fractor->actorIndex;
		cassert(p_actors.isValid(aid));
		cActor* actor = p_actors[aid];
		cFractureMaterial& mat = fractor->f_material;

		const cAABB& actorAABB = GetActorAABB(aid);
		const cVec2 extents = actorAABB.getExtents();

		cVoronoiDiagram overlayPattern;
		if (fractor->patternIndex < 0)
		{
			// no pattern create one!
		}
		else
		{
			// use fracture pattern provided
			//  translate fracture pattern to found collision point by scaling up bounds to be the bounds of the actor shape(s)
		}
		
		//  If required, create fracture pattern from material properties, else use fracture pattern provided!
		//  translate fracture pattern to found collision point by scaling up bounds to be the bounds of the actor shape(s)
		//  create fragments by overlaying pattern onto actual shape polygon
		//  Reset the new COMs and masses to the individual fragments, including original fractor (resized to the largest piece)			//  Add additional fragments as fractors/actors (depending on if allow deep fractures) into system
		//  Apply initial velocity and angular velocity using a dividng formula and dampening based on material properties to each fragment
	}


	// loop through all fracturing fractors, as above
	//  loop through each collision manifold
	//   merge all the collision points to find 1 primary collision position, skewing the average based on the points with the most impact force
	//  end loop
	//  If required, create fracture pattern from material properties, else use fracture pattern provided!
	//  translate fracture pattern to found collision point by scaling up bounds to be the bounds of the actor shape(s)
	//  create fragments by overlaying pattern onto actual shape polygon
	//  Reset the new COMs and masses to the individual fragments, including original fractor (resized to the largest piece)
	//  Add additional fragments as fractors/actors (depending on if allow deep fractures) into system
	//  Apply initial velocity and angular velocity using a dividng formula and dampening based on material properties to each fragment
	// repeat loop until we checked all fracturing fractors
}