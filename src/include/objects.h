#include "types.h"
#include "raylib.h"


typedef struct{
	Vector2 dimensions;
} epickStaticRectangle;

typedef struct {
	int weight;
	Vector2 position;
	Vector2 velocity;
	float rotation; // thank god it's 2d
} physicsObject;

typedef struct {
	physicsObject base;
	int radius;
} epickBall;

typedef struct {
  physicsObject base;
  Vector2 dimensions;
} epickBox;

