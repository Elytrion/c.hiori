#pragma once
#include "chioriMath.h"

namespace chiori
{
	// forward declaration
	class cPhysicsWorld;
	struct cContact;

	struct SolverContext
	{
		float dt, inv_dt;
		float h, inv_h;
		int iterations;
		int extraIterations;
		bool warmStart;
	};

	struct ContactConstraintPoint
	{
		vec2 rA0, rB0; // initial anchors in world space
		vec2 localAnchorA, localAnchorB; // local anchors
		vec2 localFrictionA, localFrictionB; // local friction anchors
		float tangentSeparation;
		float separation;
		float adjustedSeparation;
		float normalImpulse;
		float tangentImpulse;
		float normalMass;
		float tangentMass;
		float massCoefficient;
		float biasCoefficient;
		float impulseCoefficient;
		bool frictionValid;
	};

	struct ContactConstraint
	{
		cContact* contact;
		int indexA;
		int indexB;
		ContactConstraintPoint points[2];
		vec2 normal;
		float friction;
		int pointCount;
	};

	void PGSSoftSolver(cPhysicsWorld* world, SolverContext* context);

	void IntegrateVelocities(cPhysicsWorld* world, float h);
	void IntegratePositions(cPhysicsWorld* world, float h);
	void SolvePositions(cPhysicsWorld* world);
	void PrepareSoftContacts(cPhysicsWorld* world, SolverContext* context, ContactConstraint* constraints, int constraintCount, float h, float hertz);
	void WarmStartContacts(cPhysicsWorld* world, ContactConstraint* constraints, int constraintCount);
	void StoreContactImpluses(ContactConstraint* constraints, int constraintCount);
}