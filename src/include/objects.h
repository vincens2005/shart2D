#include "types.h"

typedef struct {
	int weight;
	Vector2 position;
	Vector2 velocity;
} physicsObject;

typedef struct {
	physicsObject base;
	int radius;
} physicsBall;
