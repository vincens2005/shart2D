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
