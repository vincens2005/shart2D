#include "types.h"
#include "raylib.h"

typedef struct {
	int numPoints;
	Vector2 *pointArray;
	Vector2 *globalPointArray;
} polygonCollisionShape;

typedef struct {
	int weight;
	Vector2 position;
	Vector2 velocity;
	float rotation; // thank god it's 2d
	float gravityStrength;
	polygonCollisionShape *collisionShape;
} physicsObject;
