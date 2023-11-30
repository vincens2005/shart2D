#include "types.h"
#include "raylib.h"

typedef struct {
	int numPoints;
	Vector2 *pointArray;
	Vector2 *globalPointArray;
} polygonCollisionShape;

typedef struct {
	float mass;
	Vector2 position;
	Vector2 velocity;
	float angularVelocity;
	float rotation; // thank god it's 2d
	float gravityStrength;
	float inertia;
	bool isStaticBody;
	polygonCollisionShape *collisionShape;
} physicsObject;

typedef struct {
  Vector2 normal;
  Vector2 point;
  float penetrationDepth;
  bool isCollided;
  // physicsObject *object1;
  // physicsObject *object2;
} collisionResult;
