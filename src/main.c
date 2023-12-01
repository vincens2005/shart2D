#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "include/raylib.h"
#include "include/objects.h"
#include "include/vectormath.h"
#include "include/collision.h"


#define MAX_OBJECTS 10

physicsObject objectArray[MAX_OBJECTS];
int objectCount = 0;

float gravity = 1.0;
void applyPolygonTransform(physicsObject *object) {
	polygonCollisionShape *poly = object->collisionShape;

	for (int i = 0; i < poly->numPoints; i++) {
		// Apply rotation (radians)
		float rotatedX = poly->pointArray[i].x * cosf(object->rotation) - poly->pointArray[i].y * sinf(object->rotation);
		float rotatedY = poly->pointArray[i].x * sinf(object->rotation) + poly->pointArray[i].y * cosf(object->rotation);

		// Apply translation
		poly->globalPointArray[i] = vec2Add(object->position, (Vector2){rotatedX, rotatedY});
	}
}

void handleCollision(physicsObject *object1, physicsObject *object2, collisionResult result) {
	Vector2 axis = result.normal;

	if (vec2Dot(object1->position, axis) < vec2Dot(object2->position, axis)) {
		axis = vec2Negate(axis);
	}

	Vector2 velocity1 = object1->velocity;
	Vector2 velocity2 = object2->velocity;
	float mass1 = object1->mass;
	float mass2 = object2->mass;

	Vector2 velocity = vec2Sub(velocity1, velocity2);
	float velocityProjection = vec2Dot(velocity, axis);

	float elasticity = 0.5f; // Adjust this value as needed
	float impulse = (-(1.0f + elasticity) * velocityProjection) / (1.0f / mass1 + 1.0f / mass2);
	Vector2 impulseVector = vec2Scale(axis, impulse);

	Vector2 penetration = vec2Scale(axis, result.penetrationDepth / (1.0f / mass1 + 1.0f / mass2));

	if (!object1->isStaticBody) {
		object1->velocity = vec2Add(object1->velocity, vec2Scale(impulseVector, 1.0f / mass1));
		object1->position = vec2Add(object1->position, vec2Scale(penetration, 1.0f / mass1));
	}
	if (!object2->isStaticBody) {
		object2->velocity = vec2Sub(object2->velocity, vec2Scale(impulseVector, 1.0f / mass2));
		object2->position = vec2Sub(object2->position, vec2Scale(penetration, 1.0f / mass2));
	}
	applyPolygonTransform(object1);
	applyPolygonTransform(object2);
}


void handleVelocity(physicsObject *object) {
	if (!(object->isStaticBody)) {
		object->velocity.y += gravity;
		object->rotation += object->angularVelocity * 0.001;
		object->angularVelocity *= 0.1;
	}
	if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
		if (CheckCollisionPointPoly(GetMousePosition(), object->collisionShape->globalPointArray, object->collisionShape->numPoints)) {
			Vector2 delta = GetMouseDelta();
			object->velocity = delta;
		}
	}
	// substeps????????
	object->position = vec2Add(object->position, object->velocity);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < objectCount; j++) {
			physicsObject *object2 = &objectArray[j];
			if (object != object2) {
					collisionResult result = polygonIntersect(object->collisionShape->numPoints, object->collisionShape->globalPointArray, object2->collisionShape->numPoints, object2->collisionShape->globalPointArray);
					if (result.isCollided) {
						handleCollision(object, object2, result);
					}
			}
		}
	}
}

void drawPhysicsPolygon(polygonCollisionShape *poly, Color color) {
	DrawTriangleFan(poly->globalPointArray, poly->numPoints, color);
}

void createPhysicsRect(Vector2 center, Vector2 dimensions, float rotation, bool isStaticBody, float mass, float gravityStrength) {
	if (objectCount >= MAX_OBJECTS) {
		return;
	}
	// Create a physics shape based on dimensions (using malloc to avoid a dangling pointer)
	polygonCollisionShape *rectShape =  (polygonCollisionShape *)malloc(sizeof(polygonCollisionShape));
	rectShape->numPoints = 4;
	rectShape->pointArray = (Vector2 *)malloc(rectShape->numPoints * sizeof(Vector2));
	rectShape->globalPointArray = (Vector2 *)malloc(rectShape->numPoints * sizeof(Vector2));
	rectShape->pointArray[0] = (Vector2){dimensions.x * -0.5f, dimensions.y * -0.5f}; // top left
	rectShape->pointArray[1] = (Vector2){dimensions.x * -0.5f, dimensions.y * 0.5f}; // bottom left
	rectShape->pointArray[2] = (Vector2){dimensions.x * 0.5f, dimensions.y * 0.5f}; // bottom right
	rectShape->pointArray[3] = (Vector2){dimensions.x * 0.5f, dimensions.y * -0.5f}; // top right

	// create the physicsObject and assign collision shape
	physicsObject object;
	object.mass = mass;
	object.position = center;
	object.velocity = (Vector2){0, 0};
	object.rotation = rotation;
	object.gravityStrength = gravityStrength;
	object.isStaticBody = isStaticBody;
	object.collisionShape = rectShape;
	object.inertia = object.mass;
	object.angularVelocity = 0;

	if (object.isStaticBody) {
		object.inertia = 0.0f;
	}
	// apply transforms _before_ adding to the array
	applyPolygonTransform(&object);
	objectArray[objectCount++] = object;
}

void initializeShapes() {
	createPhysicsRect((Vector2){300, 100}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){0, 10}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){150, 10}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){400, 10}, (Vector2){200, 200}, 0.0f, false, 5.0f, 1.0f);
	createPhysicsRect((Vector2){0, 500}, (Vector2){1920, 50}, 0.0f, true, 1.0f, 1.0f);
}

void physicsTick(){
	// random colors to choose from
	Color colors[12] = {BLUE,RED,ORANGE,PURPLE,GREEN,LIME,VIOLET,DARKBLUE,SKYBLUE,MAROON,BROWN,BEIGE};
	// TODO: handle substeps
	for (int i = 0; i < objectCount; i++){
		physicsObject *object = &objectArray[i];
		handleVelocity(object);
		// handle drawing
		applyPolygonTransform(object);
		drawPhysicsPolygon(object->collisionShape, colors[i % 12] /*inputting colors*/);
	}
}

void cleanupShapes() {
	for (int i = 0; i < objectCount; i++) {
		free(objectArray[i].collisionShape->pointArray);
		free(objectArray[i].collisionShape->globalPointArray);
		free(objectArray[i].collisionShape);
	}
}

int main() {
	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	InitWindow(640, 480, "hyper realistic 2D physics engine");
	SetTargetFPS(60); // 60 fps
	initializeShapes();
	while (!WindowShouldClose()) {
		// draw the ball
		BeginDrawing();
		ClearBackground(BLACK);
		// drawing shapes or something idfk
		physicsTick();
		// offset = (Vector2){GetScreenWidth()/2.0f,GetScreenHeight()/2.0f};
		DrawFPS(10, 10); // show current fps on screen

		EndDrawing(); // drawing done!
	}
	cleanupShapes();

	CloseWindow();
	return 0;
}
