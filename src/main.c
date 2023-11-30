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

void handleCollision(physicsObject *object1, physicsObject *object2, collisionResult result) {
	  // Check if either object is static


    if (object1->isStaticBody) {
        // Both objects are static, no need to resolve collision
        return;
    }

    // Calculate the relative velocity of the objects
    Vector2 relativeVelocity = (Vector2){
        object2->velocity.x - object1->velocity.x,
        object2->velocity.y - object1->velocity.y
    };

    // Calculate the relative normal velocity
    float relativeNormalVelocity = relativeVelocity.x * result.normal.x + relativeVelocity.y * result.normal.y;

    // If objects are moving away from each other, no need to resolve collision
    if (relativeNormalVelocity > 0) {
        return;
    }

    // Calculate the impulse along the normal
    float impulse = -(1 + 0.3) * relativeNormalVelocity /
                    (1 / object1->mass + 1 / object2->mass);

    // Apply impulse to update velocities
    if (!object1->isStaticBody) {
        object1->velocity.x -= impulse * result.normal.x / object1->mass;
        object1->velocity.y -= impulse * result.normal.y / object1->mass;

        // If the objects have angular velocity, apply angular impulse
        object1->angularVelocity -= impulse * result.normal.x * object2->mass;
    }

    if (!object2->isStaticBody) {
        object2->velocity.x += impulse * result.normal.x / object2->mass;
        object2->velocity.y += impulse * result.normal.y / object2->mass;

        // If the objects have angular velocity, apply angular impulse
        object2->angularVelocity += impulse * result.normal.x * object2->mass;
    }

    // Apply correction to the positions of the colliding objects
    float correctionMagnitude = fmaxf(result.penetrationDepth - 0.1f, 0.0f);
    Vector2 correction = (Vector2){
        result.normal.x * correctionMagnitude,
        result.normal.y * correctionMagnitude
    };

    if (!object1->isStaticBody) {
        object1->position.x += correction.x * (object2->isStaticBody ? 1.0f : 0.5f);
        object1->position.y += correction.y * (object2->isStaticBody ? 1.0f : 0.5f);
    }

    if (!object2->isStaticBody) {
        object2->position.x -= correction.x * (object1->isStaticBody ? 1.0f : 0.5f);
        object2->position.y -= correction.y * (object1->isStaticBody ? 1.0f : 0.5f);
    }
}

void handleVelocity(physicsObject *object) {
	if (object->isStaticBody) {
		return;
	}
	object->velocity.y += gravity;
	for (int i = 0; i < objectCount; i++) {
		physicsObject *object2 = &objectArray[i];
		if (object != object2) {
				collisionResult result = polygonIntersect(object->collisionShape->numPoints, object->collisionShape->globalPointArray,object2->collisionShape->numPoints, object2->collisionShape->globalPointArray);
				if (result.isCollided) {
					printf("askldjflasj KILL ME \n");
					printf("resulting max normal (%f, %f) \n", result.normal.x, result.normal.y);
					handleCollision(object, object2, result);
				}
		}
	}
	object->position = (Vector2){object->position.x + object->velocity.x, object->position.y + object->velocity.y};
	object->rotation += object->angularVelocity;
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
