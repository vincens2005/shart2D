#include "types.h"
#include "raylib.h"




typedef struct {
	int numPoints;
	Vector2 *pointArray;
	Vector2 *globalPointArray;
} polygonCollisionShape;

typedef struct {
	float staticFriction;
	float dynamicFriction;
	float mass;
	float inertia;
	float invMass;
	float invInertia;
	Vector2 position;
	Vector2 velocity;
	float angularVelocity;
	float rotation; // thank god it's 2d
	float gravityStrength;
	bool isStaticBody;
	polygonCollisionShape *collisionShape;
} physicsObject;

typedef struct {
	Vector2 normal;
	Vector2 contact1;
	Vector2 contact2;
	int numContacts;
	float penetrationDepth;
	bool isCollided;
	// physicsObject *object1;
	// physicsObject *object2;
} collisionResult;


float get_inertia(physicsObject *object) {
	if (object->isStaticBody) {
		return 0;
	}

	Vector2 *points = object->collisionShape->pointArray;
	float inertia = 0;

	for (int i = 0; i < object->collisionShape->numPoints; i++) {
		Vector2 point1 = points[i];
		Vector2 point2 = points[(i + 1) % object->collisionShape->numPoints];

		float term1 = vec2Cross(point1, point1) + vec2Cross(point2, point2);
		float term2 = vec2Cross(point1, point2);

		inertia += term1 + term2;
	};

		return fabsf(inertia) / 100.0f;
	};
