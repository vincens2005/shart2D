#include "types.h"
#include "raylib.h"

typedef struct {
	int weight;
	Vector2 position;
	Vector2 velocity;
	float rotation; // thank god it's 2d
} physicsObject;

typedef struct {
	physicsObject base;
	int radius;
} physicsBall;
