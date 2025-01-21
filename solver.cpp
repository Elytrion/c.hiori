#include "pch.h"
#include "solver.h"
#include "contact.h"
#include "physicsWorld.h"
#include "chioriPool.h"
#include "cActor.h"
#include "cShape.h"

namespace chiori
{
	#define MaxBaumgarteVelocity 4.0f

	static void PGSSoftContactSolver(cPhysicsWorld* world, ContactConstraint* constraints, int constraintCount, float inv_h, bool useBias)
	{
		auto& actors = world->p_actors;

		for (int i = 0; i < constraintCount; ++i)
		{
			ContactConstraint* constraint = constraints + i;

			cActor* bodyA = actors[constraint->indexA];
			cActor* bodyB = actors[constraint->indexB];

			float mA = bodyA->invMass;
			float iA = bodyA->invInertia;
			float mB = bodyB->invMass;
			float iB = bodyB->invInertia;
			int pointCount = constraint->pointCount;

			vec2 vA = bodyA->linearVelocity;
			float wA = bodyA->angularVelocity;
			vec2 vB = bodyB->linearVelocity;
			float wB = bodyB->angularVelocity;

			vec2 normal = constraint->normal;
			vec2 tangent = { normal.y, -normal.x };
			float friction = constraint->friction;

			for (int j = 0; j < pointCount; ++j)
			{
				ContactConstraintPoint* cp = constraint->points + j;

				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if (cp->separation > 0.0f)
				{
					// Speculative
					bias = cp->separation * inv_h;
				}
				else if (useBias)
				{
					bias = c_max(cp->biasCoefficient * cp->separation, -MaxBaumgarteVelocity);
					massScale = cp->massCoefficient;
					impulseScale = cp->impulseCoefficient;
				}

				// static anchors
				vec2 rA = cp->rA0;
				vec2 rB = cp->rB0;

				// Relative velocity at contact
				vec2 vrB = vB + cross(wB, rB);
				vec2 vrA = vA + cross(wA, rA);
				float vn = (vrB - vrA).dot(normal);

				// Compute normal impulse
				float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;

				// Clamp the accumulated impulse
				float newImpulse = c_max(cp->normalImpulse + impulse, 0.0f);
				impulse = newImpulse - cp->normalImpulse;
				cp->normalImpulse = newImpulse;

				// Apply contact impulse
				vec2 P = (impulse * normal);
				vA = vA - (mA * P);
				wA -= iA * cross(rA, P);

				vB = vB + (mB * P);
				wB += iB * cross(rB, P);
			}

			for (int j = 0; j < pointCount; ++j)
			{
				ContactConstraintPoint* cp = constraint->points + j;

				// static anchors
				vec2 rA = cp->rA0;
				vec2 rB = cp->rB0;

				// Relative velocity at contact
				vec2 vrB = vB + cross(wB, rB);
				vec2 vrA = vA + cross(wA, rA);
				vec2 dv = (vrB - vrA);

				// Compute tangent force
				float vt = dv.dot(tangent);
				float lambda = cp->tangentMass * (-vt);

				// Clamp the accumulated force
				float maxFriction = friction * cp->normalImpulse;
				float newImpulse = c_clamp(cp->tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - cp->tangentImpulse;
				cp->tangentImpulse = newImpulse;

				// Apply contact impulse
				vec2 P = (lambda * tangent);

				vA = vA - (mA * P);
				wA -= iA * cross(rA, P);

				vB = vB + (mB * P);
				wB += iB * cross(rB, P);
			}

			bodyA->linearVelocity = vA;
			bodyA->angularVelocity = wA;
			bodyB->linearVelocity = vB;
			bodyB->angularVelocity = wB;
		}
	}

	void PGSSoftSolver(cPhysicsWorld* world, SolverContext* context)
	{
		auto& contacts = world->p_contacts;
		int contactCapacity = static_cast<int>(contacts.capacity());

		ContactConstraint* constraints = static_cast<ContactConstraint*>(world->allocator->allocate(sizeof(ContactConstraint) * contactCapacity));
		int constraintCount = 0;

		for (int i = 0; i < contactCapacity; ++i)
		{
			if (!contacts.isValid(i))
				continue;
			
			cContact* contact = contacts[i];
			if (contact->manifold.pointCount == 0)
				continue;

			new (constraints + constraintCount) ContactConstraint(); //placement new construct to not cause errors
			constraints[constraintCount].contact = contact;
			constraints[constraintCount].contact->manifold.constraintIndex = constraintCount;
			constraintCount += 1;
		}

		int velocityIterations = context->iterations;
		int positionIterations = context->extraIterations;
		float h = context->dt;
		float inv_h = context->inv_dt;

		float contactHertz = c_min(30.0f, 0.333f * inv_h);
		// Loops: body 3, constraint 2 + vel iter + pos iter

		IntegrateVelocities(world, h);

		PrepareSoftContacts(world, context, constraints, constraintCount, h, contactHertz);

		if (context->warmStart)
		{
			WarmStartContacts(world, constraints, constraintCount);
		}

		// constraint loop * velocityIterations
		bool useBias = true;
		for (int iter = 0; iter < velocityIterations; ++iter)
		{
			PGSSoftContactSolver(world, constraints, constraintCount, inv_h, useBias);
		}

		// Update positions from velocity
		// body loop
		IntegratePositions(world, h);

		// Relax
		// constraint loop * positionIterations
		useBias = false;
		for (int iter = 0; iter < positionIterations; ++iter)
		{
			PGSSoftContactSolver(world, constraints, constraintCount, inv_h, useBias);
		}

		// Update positions from velocity
		// body loop
		SolvePositions(world);

		// constraint loop
		StoreContactImpluses(constraints, constraintCount);

		// free the constraints
		world->allocator->deallocate(constraints, sizeof(ContactConstraint) * contactCapacity);
	}

	void IntegrateVelocities(cPhysicsWorld* world, float h)
	{
		auto& actors = world->p_actors;
		int actorCapacity = static_cast<int>(actors.capacity());
		vec2 gravity = world->gravity;

		int i = actorCapacity - 1;
		for (; i >= 0; --i)
		{
			if (!actors.isValid(i))
				continue;

			cActor* actor = actors[i];
			if (!actor->type == cActorType::DYNAMIC)
				continue;

			float invMass = actor->invMass;
			float invI = actor->invInertia;

			vec2 v = actor->linearVelocity;
			float w = actor->angularVelocity;

			vec2 f = actor->forces;
			if (actor->_flags.isSet(cActor::USE_GRAVITY))
				f += actor->mass * actor->gravityScale * gravity;
			vec2 dv = (h * invMass) * f;
			v += dv;

			w += h * actor->torques * invI;

			// Damper to prevent infinite oscillation
			v *= 1.0f / (1.0f + h * actor->linearDamping);
			w *= 1.0f / (1.0f + h * actor->angularDamping);

			actor->linearVelocity = v;
			actor->angularVelocity = w;
		}
	}

	void IntegratePositions(cPhysicsWorld* world, float h)
	{
		auto& actors = world->p_actors;
		int actorCapacity = static_cast<int>(actors.capacity());
		vec2 gravity = world->gravity;

		int i = actorCapacity - 1;
		for (; i >= 0; --i)
		{
			if (!actors.isValid(i))
				continue;

			cActor* actor = actors[i];
			if (actor->type == cActorType::STATIC)
				continue;

			actor->deltaPosition = actor->deltaPosition + h * actor->linearVelocity;
			actor->rot = actor->rot.intergrated(h * actor->angularVelocity);
		}
	}

	void SolvePositions(cPhysicsWorld* world)
	{
		auto& actors = world->p_actors;
		int actorCapacity = static_cast<int>(actors.capacity());
		vec2 gravity = world->gravity;

		int i = actorCapacity - 1;
		for (; i >= 0; --i)
		{
			if (!actors.isValid(i))
				continue;

			cActor* actor = actors[i];
			if (actor->type == cActorType::STATIC)
				continue;

			actor->position += actor->deltaPosition;
			actor->deltaPosition = vec2::zero;
		}
	}
	
	void PrepareSoftContacts(cPhysicsWorld* world, SolverContext* context, ContactConstraint* constraints, int constraintCount, float h, float hertz)
	{
		auto& actors = world->p_actors;
		bool warmStart = context->warmStart;

		for (int i = 0; i < constraintCount; ++i)
		{
			ContactConstraint* constraint = constraints + i;
			
			cContact* contact = constraint->contact;
			const cManifold& manifold = contact->manifold;
			int pointCount = manifold.pointCount;
			cassert(0 < pointCount && pointCount <= 2);
			int indexA = contact->edges[0].bodyIndex;
			int indexB = contact->edges[1].bodyIndex;

			constraint->indexA = indexA;
			constraint->indexB = indexB;
			constraint->normal = manifold.normal;
			constraint->friction = contact->friction;
			constraint->pointCount = pointCount;

			cActor* actorA = actors[indexA];
			cActor* actorB = actors[indexB];

			float mA = actorA->invMass; float iA = actorA->invInertia;
			float mB = actorB->invMass; float iB = actorB->invInertia;

			// Stiffer for dynamic vs static
			float contactHertz = (mA == 0.0f || mB == 0.0f) ? 2.0f * hertz : hertz;

			cRot qA = actorA->rot;
			cRot qB = actorB->rot;

			vec2 normal = constraint->normal;
			vec2 tangent = { normal.y, -normal.x };

			for (int j = 0; j < pointCount; ++j)
			{
				const cManifoldPoint* mp = manifold.points + j;
				ContactConstraintPoint* cp = constraint->points + j;

				if (warmStart)
				{
					cp->normalImpulse = mp->normalImpulse;
					cp->tangentImpulse = mp->tangentImpulse;
				}
				else
				{
					cp->normalImpulse = 0.0f;
					cp->tangentImpulse = 0.0f;
				}

				cp->localAnchorA = mp->localAnchorA - actorA->localCenter;
				cp->localAnchorB = mp->localAnchorB - actorB->localCenter;

				vec2 rA = cp->localAnchorA.rotated(qA);
				vec2 rB = cp->localAnchorB.rotated(qB);
				cp->rA0 = rA;
				cp->rB0 = rB;

				cp->separation = mp->separation;
				cp->adjustedSeparation = mp->separation - (rB - rA).dot(normal);

				float rnA = rA.cross(normal);
				float rnB = rB.cross(normal);
				float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
				cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

				float rtA = rA.cross(tangent);
				float rtB = rB.cross(tangent);
				float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
				cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

				// soft contact
				// should use the substep not the full time step
				const float zeta = 10.0f;
				float omega = 2.0f * PI * contactHertz;
				float c = h * omega * (2.0f * zeta + h * omega);
				cp->biasCoefficient = omega / (2.0f * zeta + h * omega);
				cp->impulseCoefficient = 1.0f / (1.0f + c);
				cp->massCoefficient = c * cp->impulseCoefficient;
			}
		}
	}
	
	void WarmStartContacts(cPhysicsWorld* world, ContactConstraint* constraints, int constraintCount)
	{
		auto& actors = world->p_actors;

		for (int i = 0; i < constraintCount; ++i)
		{
			ContactConstraint* constraint = constraints + i;

			int pointCount = constraint->pointCount;
			cassert(0 < pointCount && pointCount <= 2);

			cActor* actorA = actors[constraint->indexA];
			cActor* actorB = actors[constraint->indexB];

			float mA = actorA->invMass;
			float iA = actorA->invInertia;
			float mB = actorB->invMass;
			float iB = actorB->invInertia;

			vec2 vA = actorA->linearVelocity;
			float wA = actorA->angularVelocity;
			vec2 vB = actorB->linearVelocity;
			float wB = actorB->angularVelocity;

			cRot qA = actorA->rot;
			cRot qB = actorB->rot;

			vec2 normal = constraint->normal;
			vec2 tangent = { normal.y, -normal.x };

			for (int j = 0; j < pointCount; ++j)
			{
				ContactConstraintPoint* cp = constraint->points + j;

				vec2 rA = cp->localAnchorA.rotated(qA);
				vec2 rB = cp->localAnchorB.rotated(qB);

				vec2 P = (cp->normalImpulse * normal) + (cp->tangentImpulse * tangent);
				wA -= iA * rA.cross(P);
				vA = vA + (-mA * P);
				wB += iB * rB.cross(P);
				vB = vB + (mB * P);
			}

			actorA->linearVelocity = vA;
			actorA->angularVelocity = wA;
			actorB->linearVelocity = vB;
			actorB->angularVelocity = wB;
		}
	}

	void StoreContactImpluses(ContactConstraint* constraints, int constraintCount)
	{
		for (int i = 0; i < constraintCount; ++i)
		{
			ContactConstraint* constraint = constraints + i;
			cContact* contact = constraint->contact;
			cManifold* manifold = &contact->manifold;

			for (int j = 0; j < constraint->pointCount; ++j)
			{
				manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
				manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
			}
		}
	}


	static void PGSBaumgarteContactSolver(cPhysicsWorld* world, ContactConstraint* constraints, int constraintCount, float inv_h)
	{
		auto& actors = world->p_actors;

		for (int i = 0; i < constraintCount; ++i)
		{
			ContactConstraint* constraint = constraints + i;
			
			cActor* bodyA = actors[constraint->indexA];
			cActor* bodyB = actors[constraint->indexB];

			float mA = bodyA->invMass;
			float iA = bodyA->invInertia;
			float mB = bodyB->invMass;
			float iB = bodyB->invInertia;
			int pointCount = constraint->pointCount;

			vec2 vA = bodyA->linearVelocity;
			float wA = bodyA->angularVelocity;
			vec2 vB = bodyB->linearVelocity;
			float wB = bodyB->angularVelocity;

			vec2 normal = constraint->normal;
			vec2 tangent = { normal.y, -normal.x };
			float friction = constraint->friction;

			for (int j = 0; j < pointCount; ++j)
			{
				ContactConstraintPoint* cp = constraint->points + j;

				float bias = 0.0f;
				if (cp->separation > 0.0f)
				{
					// Speculative
					bias = cp->separation * inv_h;
				}
				else
				{
					bias = c_max(0.2f * inv_h * c_min(0.0f, cp->separation + 0.005f), -MaxBaumgarteVelocity);
				}

				// static anchors
				vec2 rA = cp->rA0;
				vec2 rB = cp->rB0;

				// Relative velocity at contact
				vec2 vrB = vB + cross(wB, rB);
				vec2 vrA = vA + cross(wA, rA);
				float vn = (vrB - vrA).dot(normal);

				// Compute normal impulse
				float impulse = -cp->normalMass * (vn + bias);

				// Clamp the accumulated impulse
				float newImpulse = c_max(cp->normalImpulse + impulse, 0.0f);
				impulse = newImpulse - cp->normalImpulse;
				cp->normalImpulse = newImpulse;

				// Apply contact impulse
				vec2 P = (impulse * normal);
				vA = (vA - mA * P);
				wA -= iA * cross(rA, P);

				vB = (vB + mB * P);
				wB += iB * cross(rB, P);
			}

			for (int j = 0; j < pointCount; ++j)
			{
				ContactConstraintPoint* cp = constraint->points + j;

				// static anchors
				vec2 rA = cp->rA0;
				vec2 rB = cp->rB0;

				// Relative velocity at contact
				vec2 vrB = (vB + cross(wB, rB));
				vec2 vrA = (vA + cross(wA, rA));
				vec2 dv = (vrB - vrA);

				// Compute tangent force
				float vt = dot(dv, tangent);
				float lambda = cp->tangentMass * (-vt);

				// Clamp the accumulated force
				float maxFriction = friction * cp->normalImpulse;
				float newImpulse = c_clamp(cp->tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - cp->tangentImpulse;
				cp->tangentImpulse = newImpulse;

				// Apply contact impulse
				vec2 P = (lambda * tangent);

				vA = (vA - mA * P);
				wA -= iA * cross(rA, P);

				vB = (vB + mB * P);
				wB += iB * cross(rB, P);
			}

			bodyA->linearVelocity = vA;
			bodyA->angularVelocity = wA;
			bodyB->linearVelocity = vB;
			bodyB->angularVelocity = wB;
		}
	}

	static void PrepareContacts(cPhysicsWorld* world, ContactConstraint* constraints, int constraintCount, bool warmStart)
	{
		auto& actors = world->p_actors;

		for (int i = 0; i < constraintCount; ++i)
		{
			ContactConstraint* constraint = constraints + i;

			cContact* contact = constraint->contact;
			const cManifold* manifold = &contact->manifold;
			int pointCount = manifold->pointCount;
			cassert(0 < pointCount && pointCount <= 2);
			int indexA = contact->edges[0].bodyIndex;
			int indexB = contact->edges[1].bodyIndex;

			constraint->indexA = indexA;
			constraint->indexB = indexB;
			constraint->normal = manifold->normal;
			constraint->friction = contact->friction;
			constraint->pointCount = pointCount;

			cActor* bodyA = actors[constraint->indexA];
			cActor* bodyB = actors[constraint->indexB];

			float mA = bodyA->invMass;
			float iA = bodyA->invInertia;
			float mB = bodyB->invMass;
			float iB = bodyB->invInertia;

			cRot qA = bodyA->rot;
			cRot qB = bodyB->rot;

			vec2 normal = constraint->normal;
			vec2 tangent = { normal.y, -normal.x };

			for (int j = 0; j < pointCount; ++j)
			{
				const cManifoldPoint* mp = manifold->points + j;
				ContactConstraintPoint* cp = constraint->points + j;

				if (warmStart)
				{
					cp->normalImpulse = mp->normalImpulse;
					cp->tangentImpulse = mp->tangentImpulse;
				}
				else
				{
					cp->normalImpulse = 0.0f;
					cp->tangentImpulse = 0.0f;
				}

				cp->localAnchorA = (mp->localAnchorA - bodyA->localCenter);
				cp->localAnchorB = (mp->localAnchorB - bodyB->localCenter);

				vec2 rA = (cp->localAnchorA.rotated(qA));
				vec2 rB = (qB, cp->localAnchorB.rotated(qB));
				cp->rA0 = rA;
				cp->rB0 = rB;

				cp->separation = mp->separation;
				cp->adjustedSeparation = mp->separation - dot((rB - rA), normal);

				cp->biasCoefficient = mp->separation > 0.0f ? 1.0f : 0.0f;

				float rtA = cross(rA, tangent);
				float rtB = cross(rB, tangent);
				float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
				cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

				float rnA = cross(rA, normal);
				float rnB = cross(rB, normal);
				float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
				cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
			}
		}
	}

	// This is the solver in box2d_lite
	void PGSSolver(cPhysicsWorld* world, SolverContext* context)
	{
		auto& contacts = world->p_contacts;
		int contactCapacity = static_cast<int>(contacts.capacity());
		
		ContactConstraint* constraints = static_cast<ContactConstraint*>(world->allocator->allocate(sizeof(ContactConstraint) * contactCapacity));
		int constraintCount = 0;

		for (int i = 0; i < contactCapacity; ++i)
		{
			if (!world->p_contacts.isValid(i))
				continue;
			
			cContact* contact = contacts[i];

			if (contact->manifold.pointCount == 0)
			{
				continue;
			}

			constraints[constraintCount].contact = contact;
			constraints[constraintCount].contact->manifold.constraintIndex = constraintCount;
			constraintCount += 1;
		}

		int iterations = context->iterations;
		float h = context->dt;
		float inv_h = context->inv_dt;

		// Loops: body 2, constraint 2 + iterations

		// body loop
		IntegrateVelocities(world, h);

		// constraint loop
		PrepareContacts(world, constraints, constraintCount, context->warmStart);

		if (context->warmStart)
		{
			WarmStartContacts(world, constraints, constraintCount);
		}
		
		for (int iter = 0; iter < iterations; ++iter)
		{
			PGSBaumgarteContactSolver(world, constraints, constraintCount, inv_h);
		}

		// body loop
		// Update positions from velocity
		IntegratePositions(world, h);
		SolvePositions(world);

		// constraint loop
		StoreContactImpluses(constraints, constraintCount);
		
		// free the constraints
		world->allocator->deallocate(constraints, sizeof(ContactConstraint) * contactCapacity);
	}
}