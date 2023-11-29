#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "include/raylib.h"
#include "include/objects.h"
#include "include/collision.h"

#define MAX_OBJECTS 10

physicsObject objectArray[MAX_OBJECTS];
int objectCount = 0;

float gravity = 1.0;

Vector2 offset = {0,0};

Vector2 addOffset(Vector2 position){
	return (Vector2){position.x + offset.x, position.y + offset.y};
}

void handleVelocity(physicsObject *object) {
	if (object->isStaticBody) {
		return;
	}
	object->velocity.y += gravity;
	object->position = (Vector2){object->position.x + object->velocity.x, object->position.y + object->velocity.y};
}

void drawPhysicsPolygon(polygonCollisionShape *poly, Color color) {
	DrawTriangleFan(poly->globalPointArray, poly->numPoints, color);
}

void applyPolygonTransform(physicsObject *object) {
	polygonCollisionShape *poly = object->collisionShape;

	for (int i = 0; i < poly->numPoints; i++) {
		// Apply rotation (radians)
		float rotatedX = poly->pointArray[i].x * cosf(object->rotation) - poly->pointArray[i].y * sinf(object->rotation);
		float rotatedY = poly->pointArray[i].x * sinf(object->rotation) + poly->pointArray[i].y * cosf(object->rotation);

		// Apply translation
		poly->globalPointArray[i] = (Vector2){rotatedX + object->position.x, rotatedY + object->position.y};
	}
}

// void calculatePolyNormals(polygonCollisionShape *poly) {

// }

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

	// apply transforms _before_ adding to the array
	applyPolygonTransform(&object);
	objectArray[objectCount++] = object;
}

void initializeShapes() {
	createPhysicsRect((Vector2){100, 100}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){960, 1000}, (Vector2){1920, 50}, 0.0f, true, 1.0f, 1.0f);
}


void physicsTick(){
	// random colors to choose from
	Color colors[12] = {BLUE,RED,ORANGE,PURPLE,GREEN,LIME,VIOLET,DARKBLUE,SKYBLUE,MAROON,BROWN,BEIGE};

	for (int i = 0; i < objectCount; i++){
		physicsObject *object = &objectArray[i];
		handleVelocity(object);
		// handle drawing
		applyPolygonTransform(object);
		drawPhysicsPolygon(object->collisionShape, colors[i % 12] /*inputting colors*/);
		for (int i = 0; i < objectCount; i++) {
			physicsObject *object2 = &objectArray[i];
			if (object != object2) {
				arePolygonsIntersecting(object->collisionShape->numPoints, object->collisionShape->globalPointArray,object2->collisionShape->numPoints, object2->collisionShape->globalPointArray);
			}
		}
	}
}

void cleanupShapes() {
	for (int i = 0; i < objectCount; i++) {
		free(objectArray[i].collisionShape->pointArray);
		free(objectArray[i].collisionShape->globalPointArray);
		free(objectArray[i].collisionShape);
	}
}

// Vector2 projectVector2(Vector2 a, Vector2 b) {

// }

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
