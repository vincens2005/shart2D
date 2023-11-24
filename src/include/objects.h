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
	float rotation; // thank god it's 2d
	float gravityStrength;
	bool isStaticBody;
	polygonCollisionShape *collisionShape;
} physicsObject;
