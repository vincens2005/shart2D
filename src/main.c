#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "math.h"
#include "include/raylib.h"
#include "types.h"
#include "include/vectormath.h"
#include "include/objects.h"
#include "include/collision.h"


#define MAX_OBJECTS 20
// amount of physics iterations per frame
#define SUBSTEP_AMOUNT 20
// factor to multiply position changes by
#define SUBSTEP_FACTOR 0.05f

#define POSITION_SLOP 0.01f

int selectedObject = -1;

physicsObject objectArray[MAX_OBJECTS];
int objectCount = 0;
float gravity = 0.6f;

// TODO: unclutter main.c :D

void applyPolygonTransform(physicsObject *object) {
	polygonCollisionShape *poly = object->collisionShape;
	AABB *box = &object->box;

	Vector2 min = (Vector2){INFINITY, INFINITY};
	Vector2 max = (Vector2){-INFINITY, -INFINITY};

	for (int i = 0; i < poly->numPoints; i++) {
		// Apply rotation (radians)
		float rotatedX = poly->pointArray[i].x * cosf(object->rotation) -
										poly->pointArray[i].y * sinf(object->rotation);
		float rotatedY = poly->pointArray[i].x * sinf(object->rotation) +
										poly->pointArray[i].y * cosf(object->rotation);

		// Apply translation
		poly->globalPointArray[i] = vec2Add(
																	object->position,
																	(Vector2){rotatedX, rotatedY}
															);
		// set AABB
		if (poly->globalPointArray[i].x < min.x) {
			min.x = poly->globalPointArray[i].x;
		}
		if (poly->globalPointArray[i].y < min.y) {
			min.y = poly->globalPointArray[i].y;
		}
		if (poly->globalPointArray[i].x > max.x) {
			max.x = poly->globalPointArray[i].x;
		}
		if (poly->globalPointArray[i].y > max.y) {
			max.y = poly->globalPointArray[i].y;
		}
	}
	box->min = min;
	box->max = max;
	DrawRectangleLines(min.x - 2, min.y - 2, max.x - min.x + 4, max.y - min.y + 4, RED);
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

void resolveVelocity(collisionResult *result) {
	physicsObject *object1 = result->object1;
	physicsObject *object2 = result->object2;

	// cache velocities before collision
	Vector2 velocity1 = object1->velocity;
	Vector2 velocity2 = object2->velocity;
	float angularVelocity1 = object1->angularVelocity;
	float angularVelocity2 = object2->angularVelocity;

	Vector2 normal = result->normal;

	float elasticity = 0.5f; // TODO: put this inside of the physicsObject

	int numContacts  = result->numContacts;
	Vector2 contactArray[2] = {result->contact1, result->contact2};
	float impulseArray[2] = {0.0f, 0.0f};

	float staticFriction = (object1->staticFriction + object2->staticFriction) * 0.5f;
	float dynamicFriction = (object1->dynamicFriction + object2->dynamicFriction) * 0.5f;

	for (int i = 0; i < numContacts; i++) {
		Vector2 r1 = vec2Sub(contactArray[i], object1->position);
		Vector2 r2 = vec2Sub(contactArray[i], object2->position);

		Vector2 r1Perp = vec2Perp(r1);
		Vector2 r2Perp = vec2Perp(r2);

		Vector2 angularLinearVelocity1 = vec2Scale(r1Perp, angularVelocity1);
		Vector2 angularLinearVelocity2 = vec2Scale(r2Perp, angularVelocity2);

		Vector2 relativeVelocity = vec2Sub(
				vec2Add(velocity1, angularLinearVelocity1),
				vec2Add(velocity2, angularLinearVelocity2)
		);

		float velocityProjection = vec2Dot(relativeVelocity, normal);

		if (velocityProjection > 0.0f) {
			continue;
		}
		float r1PerpDotNormal = vec2Dot(r1Perp, normal);
		float r2PerpDotNormal = vec2Dot(r2Perp, normal);
		float impulse =
		(-(1.0f + elasticity) * velocityProjection) /
		(
				(object1->invMass + object2->invMass) +
				((r2PerpDotNormal * r2PerpDotNormal) * object2->invInertia) +
				((r1PerpDotNormal * r1PerpDotNormal) * object1->invInertia)
		);
		impulse /= (float)numContacts;
		impulseArray[i] = impulse;
		// applying velocity
		Vector2 impulseVector = vec2Scale(normal, impulse);
		object1->velocity = vec2Add(object1->velocity, vec2Scale(impulseVector, object1->invMass));
		object1->angularVelocity += vec2Cross(r1, impulseVector) * object1->invInertia;
		object2->velocity = vec2Sub(object2->velocity, vec2Scale(impulseVector, object2->invMass));
		object2->angularVelocity -= vec2Cross(r2, impulseVector) * object2->invInertia;
	}

	velocity1 = object1->velocity;
	velocity2 = object2->velocity;
	angularVelocity1 = object1->angularVelocity;
	angularVelocity2 = object2->angularVelocity;
	for (int i = 0; i < numContacts; i++) {

		Vector2 r1 = vec2Sub(contactArray[i], object1->position);
		Vector2 r2 = vec2Sub(contactArray[i], object2->position);

		Vector2 r1Perp = vec2Perp(r1);
		Vector2 r2Perp = vec2Perp(r1);

		Vector2 angularLinearVelocity1 = vec2Scale(r1Perp, angularVelocity1);
		Vector2 angularLinearVelocity2 = vec2Scale(r2Perp, angularVelocity2);

		Vector2 relativeVelocity = vec2Sub(vec2Add(velocity1, angularLinearVelocity1),
																			vec2Add(velocity2, angularLinearVelocity2));

		Vector2 tangent = vec2Sub(relativeVelocity, vec2Scale(normal, vec2Dot(relativeVelocity, normal)));

		// if the vector is nearly zero, stop this iteration
		if (vec2IsZeroApprox(tangent)) {
			continue;
		} else {
			tangent = vec2Normalize(tangent);
		}

		float r1PerpDotTangent = vec2Dot(r1Perp, tangent);
		float r2PerpDotTangent = vec2Dot(r2Perp, tangent);

		float frictionImpulse = -vec2Dot(relativeVelocity, tangent) /
									((
										(object1->invMass + object2->invMass) +
										((r1PerpDotTangent * r1PerpDotTangent) * object1->invInertia) +
										((r2PerpDotTangent * r2PerpDotTangent) * object2->invInertia)
									) * (float)numContacts);

		Vector2 frictionImpulseVector;

		if (fabsf(frictionImpulse) <= impulseArray[i] * staticFriction) {
			frictionImpulseVector = vec2Scale(tangent, frictionImpulse);
		} else {
			frictionImpulseVector = vec2Scale(tangent, -impulseArray[i] * dynamicFriction);
		}
		object1->velocity = vec2Add(object1->velocity, vec2Scale(frictionImpulseVector, object1->invMass));
		object1->angularVelocity += vec2Cross(r1, frictionImpulseVector) * object1->invInertia;
		object2->velocity = vec2Sub(object2->velocity, vec2Scale(frictionImpulseVector, object2->invMass));
		object2->angularVelocity -= vec2Cross(r2, frictionImpulseVector) * object2->invInertia;
	}
}

void handleCollision(physicsObject *object1) {
	for (int i = 0; i < objectCount; i++) {
		physicsObject *object2 = &objectArray[i];
		if (object1 != object2) {
			if (!(AABBIntersect(&object1->box, &object2->box))) {
				continue;
			}

			collisionResult result = polygonIntersect(object1, object2);
			if (result.isCollided) {
				Vector2 penetration = vec2Scale(result.normal, result.penetrationDepth);
				separateBodies(result.object1, result.object2, penetration);
				resolveVelocity(&result);
			}
		}
	}
}


void handleVelocity(physicsObject *object) {
	// apply the position and multiply the velocity by the factor to keep it scaled properly
	object->position = vec2Add(object->position, vec2Scale(object->velocity, SUBSTEP_FACTOR));
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
	object.collisionShape = rectShape;

	object.gravityStrength = gravityStrength;
	object.staticFriction = 0.6f;
	object.dynamicFriction = 0.3f;

	object.position = center;
	object.rotation = rotation;
	object.velocity = (Vector2){0, 0};
	object.angularVelocity = 0.0f;
	object.isStaticBody = isStaticBody;

	if (object.isStaticBody) {
		object.inertia = 0.0f;
		object.mass = 0.0f;
		object.invMass = 0.0f;
		object.invInertia = 0.0f;
	} else {
		object.inertia = getPolygonInertia(rectShape);
		object.mass = mass;
		object.invMass = 1.0f / object.mass;
		object.invInertia = 1.0f / object.inertia;
	}

	// apply transforms _before_ adding to the array
	applyPolygonTransform(&object);
	objectArray[objectCount++] = object;
}

void initializeShapes() {
	createPhysicsRect((Vector2){300, 100}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){0, 10}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){150, 10}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){500, 10}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){500, 100}, (Vector2){50, 50}, 0.0f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){400, 10}, (Vector2){200, 200}, 0.4f, false, 1.0f, 1.0f);
	createPhysicsRect((Vector2){0, 500}, (Vector2){1920, 50}, 0.0f, true, 5.0f, 1.0f);
}

void physicsTick() {
	for (int j = 0; j < objectCount; j++) {
		physicsObject *object = &objectArray[j];
		if (object->isStaticBody) {
			continue;
		}
		handleVelocity(object);
		applyPolygonTransform(object);
	}
}

void drawShapes() {
	// random colors to choose from
	Color colors[12] = {BLUE,RED,ORANGE,PURPLE,GREEN,LIME,VIOLET,DARKBLUE,SKYBLUE,MAROON,BROWN,BEIGE};
	for (int i = 0; i < objectCount; i++) {
		physicsObject *object = &objectArray[i];
		if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
			if (AABBIntersectPoint(&object->box, GetMousePosition()) && selectedObject == -1) {
				selectedObject = i;
			}
		} else {
			selectedObject = -1;
		}
		if (selectedObject == i) {
			Vector2 delta = GetMouseDelta();
			if (object->isStaticBody) {
				object->position = vec2Add(object->position, delta);
				applyPolygonTransform(object);
			} else {
				object->velocity = delta;
			}
		}
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
	InitWindow(640, 480, "shart2D");
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
