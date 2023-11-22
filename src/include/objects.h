#include "types.h"
#include "raylib.h"


typedef struct{
	Vector2 dimensions;
} staticRectangle;

typedef struct {
	int weight;
	Vector2 position;
	Vector2 velocity;
	float rotation; // thank god it's 2d
} physicsObject;

typedef struct {
	physicsObject base;
	int radius;
} rigidBall;

typedef struct {
  physicsObject base;
  Vector2 dimensions;
} rigidBox;

