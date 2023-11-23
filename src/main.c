#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "include/raylib.h"
#include "include/objects.h"

#define MAX_OBJECTS 10

physicsObject objectArray[MAX_OBJECTS];
int objectCount = 0;

float gravity = 1.0;

Vector2 offset = {0,0};

Vector2 addOffset(Vector2 position){
	return (Vector2){position.x + offset.x, position.y + offset.y};
}

void handleVelocity(physicsObject *object) {
	object->velocity.y += gravity;
	object->position = (Vector2){object->position.x + object->velocity.x, object->position.y + object->velocity.y};
}

void drawPhysicsPolygon(polygonCollisionShape *poly, Color color) {
	for (int i = 0; i < poly->numPoints; i++) {
		printf("point %d applied translation: (%f, %f) \n",i , poly->globalPointArray[i].x, poly->globalPointArray[i].y);
	}
	printf("woaw %d \n",poly->numPoints);
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
		// printf("point %d applied translation: (%f, %f) \n",i , poly->globalPointArray[i].x, poly->globalPointArray[i].y);
	}
}

void createRect(Vector2 center, Vector2 dimensions) {
	if (objectCount >= MAX_OBJECTS) {
		return;
	}
	// Create a square collision shape (adjust points as needed)
	polygonCollisionShape *squareShape =  (polygonCollisionShape *)malloc(sizeof(polygonCollisionShape));
	squareShape->numPoints = 4;
	squareShape->pointArray = (Vector2 *)malloc(squareShape->numPoints * sizeof(Vector2));
	squareShape->globalPointArray = (Vector2 *)malloc(squareShape->numPoints * sizeof(Vector2));
	squareShape->pointArray[0] = (Vector2){-dimensions.x, -dimensions.y};
	squareShape->pointArray[1] = (Vector2){dimensions.x, -dimensions.y};
	squareShape->pointArray[2] = (Vector2){dimensions.x, dimensions.y};
	squareShape->pointArray[3] = (Vector2){-dimensions.x, dimensions.y};

	// Create a square physics object
	physicsObject squareObject;
	squareObject.weight = 1;
	squareObject.position = center;
	squareObject.velocity = (Vector2){0, 0};
	squareObject.rotation = 0.0f;
	squareObject.gravityStrength = 1.0f;

	squareObject.collisionShape = squareShape;

	// Add the square object to the array
	objectArray[objectCount++] = squareObject;
	applyPolygonTransform(&squareObject);
}

void initializeShapes() {
	createRect((Vector2){0, 0}, (Vector2){50, 50});
}


void drawShapes(){
	// random colors to choose from
	Color colors[12] = {BLUE,RED,ORANGE,PURPLE,GREEN,LIME,VIOLET,DARKBLUE,SKYBLUE,MAROON,BROWN,BEIGE};
	//boxes
	// printf("object count: %d\n", objectCount);
	for (int i = 0; i < objectCount; i++){
		// TODO: seperate physics from drawing
		physicsObject *object = &objectArray[i];

		handleVelocity(object);
		applyPolygonTransform(object);
		drawPhysicsPolygon(object->collisionShape, colors[i % 12] /*inputting colors*/);
	}
	// Vector2 triangle[4] = {(Vector2){0,0}, (Vector2){0,30},(Vector2){30,30}, (Vector2){30,0}};
	// DrawTriangleFan(&triangle, 4, colors[0]);
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
		drawShapes();
		// offset = (Vector2){GetScreenWidth()/2.0f,GetScreenHeight()/2.0f};
		DrawFPS(10, 10); // show current fps on screen

		EndDrawing(); // drawing done!
	}
	// cleanupShapes();

	CloseWindow();
	return 0;
}
