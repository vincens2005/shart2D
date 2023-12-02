#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "math.h"
#include "include/raylib.h"
#include "include/vectormath.h"
#include "include/objects.h"
#include "include/collision.h"


#define MAX_OBJECTS 10
// amount of physics iterations per frame
#define SUBSTEP_AMOUNT 8
// factor to multiply position changes by
#define SUBSTEP_FACTOR 0.125f

physicsObject objectArray[MAX_OBJECTS];
int objectCount = 0;
float gravity = 0.6f;

// TODO: unclutter main.c :D

void applyPolygonTransform(physicsObject *object) {
	polygonCollisionShape *poly = object->collisionShape;

	poly->min.x = INFINITY;
	poly->min.y = INFINITY;
	poly->max.x = -INFINITY;
	poly->max.y = -INFINITY;

	for (int i = 0; i < poly->numPoints; i++) {
		// Apply rotation (radians)
		float rotatedX = poly->pointArray[i].x * cosf(object->rotation) - poly->pointArray[i].y * sinf(object->rotation);
		float rotatedY = poly->pointArray[i].x * sinf(object->rotation) + poly->pointArray[i].y * cosf(object->rotation);

		// Apply translation
		poly->globalPointArray[i] = vec2Add(object->position, (Vector2){rotatedX, rotatedY});
		// set AABB
		if (poly->globalPointArray[i].x < poly->min.x) {
			poly->min.x = poly->globalPointArray[i].x;
		}
		if (poly->globalPointArray[i].y < poly->min.y) {
			poly->min.y = poly->globalPointArray[i].y;
		}
		if (poly->globalPointArray[i].x > poly->max.x) {
			poly->max.x = poly->globalPointArray[i].x;
		}
		if (poly->globalPointArray[i].y > poly->max.y) {
			poly->max.y = poly->globalPointArray[i].y;
		}
	}

	DrawRectangleLines(poly->min.x - 2, poly->min.y - 2, poly->max.x - poly->min.x + 4, poly->max.y - poly->min.y + 4, RED);
}

void separateBodies(physicsObject *object1, physicsObject *object2, Vector2 penetration) {
	// we already know that object1 is no longer a static body
	if (object2->isStaticBody) {
		object1->position = vec2Add(object1->position, penetration);
		applyPolygonTransform(object1);
	}
	else {
		object1->position = vec2Add(object1->position, vec2Scale(penetration, 0.5f));
		object2->position = vec2Add(object2->position, vec2Scale(penetration, -0.5f));
		applyPolygonTransform(object1);
		applyPolygonTransform(object2);
	}

}

void resolveVelocity(collisionResult result) {

	Vector2 relativeVelocity = vec2Sub(result.object1->velocity, result.object2->velocity);
	float velocityProjection = vec2Dot(relativeVelocity, result.normal);

	if (velocityProjection > 0.0f) {
		return;
	}

	float elasticity = 0.3f; // Adjust this value as needed
	float impulse = (-(1.0f + elasticity) * velocityProjection) / (result.object1->invMass + result.object2->invMass);

	Vector2 impulseVector = vec2Scale(result.normal, impulse);

	result.object1->velocity = vec2Add(result.object1->velocity, vec2Scale(impulseVector, result.object1->invMass));
	result.object2->velocity = vec2Sub(result.object2->velocity, vec2Scale(impulseVector, result.object2->invMass));

}

void handleCollision(physicsObject *object1) {
	for (int i = 0; i < objectCount; i++) {
		physicsObject *object2 = &objectArray[i];
		if (object1 != object2) {
			// if (!(AABBIntersect(object1->collisionShape, object2->collisionShape))) {
			// 	printf("fuck that shi");
			// 	return;
			// }

			collisionResult result = polygonIntersect(object1, object2);
			if (result.isCollided) {
				DrawCircle(result.contact1.x,result.contact1.y, 10, BLUE);
				if (result.numContacts > 1) {
					DrawCircle(result.contact2.x,result.contact2.y, 10, RED);
				}
				Vector2 penetration = vec2Scale(result.normal, result.penetrationDepth);
				separateBodies(result.object1, result.object2, penetration);
				resolveVelocity(result);
			}
		}
	}
}


void handleVelocity(physicsObject *object) {
	if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
		if (CheckCollisionPointPoly(GetMousePosition(), object->collisionShape->globalPointArray, object->collisionShape->numPoints)) {
			Vector2 delta = GetMouseDelta();
			object->velocity = delta;
		}
	}
	// apply the position and multiply the velocity by the factor to keep it scaled properly
	object->position = vec2Add(object->position, vec2Scale(object->velocity, SUBSTEP_FACTOR));
	if (object->isStaticBody) {
		object->velocity = (Vector2){0,0};
		return;
	}
	object->velocity.y += (gravity * SUBSTEP_FACTOR);
	object->rotation += (object->angularVelocity * SUBSTEP_FACTOR);
	handleCollision(object);
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
	object.inertia = get_inertia(&object);
	if (object.isStaticBody) {
		object.invMass = 0.0f;
	} else {
		object.invMass = 1.0f / object.mass;
	}
	object.invInertia = 1.0f / object.inertia;
	object.angularVelocity = 0.0f;
	object.staticFriction = 0.8f;
	object.dynamicFriction = 0.5f;

	// apply transforms _before_ adding to the array
	applyPolygonTransform(&object);
	objectArray[objectCount++] = object;
}

void initializeShapes() {
	createPhysicsRect((Vector2){300, 100}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){0, 10}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){150, 10}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){400, 10}, (Vector2){200, 200}, 0.4f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){0, 500}, (Vector2){1920, 50}, 0.0f, true, 1.0f, 1.0f);
}

void physicsTick() {
	for (int j = 0; j < objectCount; j++) {
		physicsObject *object = &objectArray[j];
		handleVelocity(object);
		applyPolygonTransform(object);
	}
}

void drawShapes() {
	// random colors to choose from
	Color colors[12] = {BLUE,RED,ORANGE,PURPLE,GREEN,LIME,VIOLET,DARKBLUE,SKYBLUE,MAROON,BROWN,BEIGE};
	for (int i = 0; i < objectCount; i++) {
		physicsObject *object = &objectArray[i];
		drawPhysicsPolygon(object->collisionShape, colors[i % 12] /*inputting colors*/);
	}
}

void tick(){
	for (int i = 0; i < SUBSTEP_AMOUNT; i++) {
		physicsTick();
	}
	drawShapes();
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
		BeginDrawing();
		ClearBackground(BLACK);

		tick();
		DrawFPS(10, 10); // show current fps on screen

		EndDrawing(); // drawing done!
	}
	cleanupShapes();

	CloseWindow();
	return 0;
}
