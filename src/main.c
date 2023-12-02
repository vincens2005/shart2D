#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "math.h"
#include "include/raylib.h"
#include "include/objects.h"
#include "include/vectormath.h"
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

	for (int i = 0; i < poly->numPoints; i++) {
		// Apply rotation (radians)
		float rotatedX = poly->pointArray[i].x * cosf(object->rotation) - poly->pointArray[i].y * sinf(object->rotation);
		float rotatedY = poly->pointArray[i].x * sinf(object->rotation) + poly->pointArray[i].y * cosf(object->rotation);

		// Apply translation
		poly->globalPointArray[i] = vec2Add(object->position, (Vector2){rotatedX, rotatedY});
	}
}

void separateBodies(physicsObject *object1, physicsObject *object2, Vector2 penetration) {
	if (object1->isStaticBody) {
		object2->position = vec2Add(object2->position, vec2Negate(penetration));
		applyPolygonTransform(object2);
	}
	else if (object2->isStaticBody) {
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

void resolveCollisionWithRotation(physicsObject* object1, physicsObject* object2, collisionResult result) {
	Vector2 normal = result.normal;
	int numContacts = result.numContacts;

	float elasticity = 0.2f; // Adjust this value as needed

	float staticFriction = (object1->staticFriction + object2->staticFriction) * 0.5f;
	float dynamicFriction = (object1->dynamicFriction + object2->dynamicFriction) * 0.5f;

	Vector2 contactArray[2] = {result.contact1, result.contact2};
	Vector2 impulseArray[2] = {(Vector2){0,0}, (Vector2){0,0}};
	Vector2 frictionImpulseArray[2] = {(Vector2){0,0}, (Vector2){0,0}};
	float jArray[2] = {0.0f,0.0f};
	Vector2 raArray[2] = {(Vector2){0,0}, (Vector2){0,0}};
	Vector2 rbArray[2] = {(Vector2){0,0}, (Vector2){0,0}};

	// loop through each contact
	for (int i = 0; i < numContacts; i++) {

		Vector2 ra = vec2Sub(contactArray[i], object1->position);
		Vector2 rb = vec2Sub(contactArray[i], object2->position);

		raArray[i] = ra;
		rbArray[i] = rb;

		Vector2 raPerp = vec2Perp(ra);
		Vector2 rbPerp = vec2Perp(rb);

		Vector2 angularLinearVelocity1 = vec2Scale(raPerp, object1->angularVelocity);
		Vector2 angularLinearVelocity2 = vec2Scale(rbPerp, object2->angularVelocity);

		Vector2 relativeVelocity = vec2Sub(vec2Add(object2->velocity, angularLinearVelocity2),
																			vec2Add(object1->velocity, angularLinearVelocity1));

		float contactVelocityLength = vec2Dot(relativeVelocity, normal);

		if (contactVelocityLength > 0.0f) {
			continue;
		}

		float raPerpDotNormal = vec2Dot(raPerp, normal);
		float rbPerpDotNormal = vec2Dot(rbPerp, normal);

		float denom = (object1->invMass + object2->invMass) +
									(raPerpDotNormal * raPerpDotNormal) * object1->invInertia +
									(rbPerpDotNormal * rbPerpDotNormal) * object2->invInertia;

		float j = -(1.0f + elasticity) * contactVelocityLength;
		j /= denom;
		j /= (float)numContacts;

		jArray[i] = j;
		Vector2 impulse = vec2Scale(normal, j);
		impulseArray[i] = impulse;
	}

	for (int i = 0; i < numContacts; i++) {
		Vector2 impulse = impulseArray[i];
		Vector2 ra = raArray[i];
		Vector2 rb = rbArray[i];
		if (!(object1->isStaticBody)) {
			object1->velocity = vec2Add(object1->velocity, vec2Scale(impulse, object1->invMass));
			object1->angularVelocity += vec2Cross(ra, impulse) * -object1->invInertia;
		}
		if (!(object2->isStaticBody)) {
			object2->velocity = vec2Add(object2->velocity, vec2Scale(impulse, -object2->invMass));
			object2->angularVelocity += vec2Cross(rb, impulse) * object2->invInertia;
		}
	}
	for (int i = 0; i < numContacts; i++) {

		Vector2 ra = vec2Sub(contactArray[i], object1->position);
		Vector2 rb = vec2Sub(contactArray[i], object2->position);

		raArray[i] = ra;
		rbArray[i] = rb;

		Vector2 raPerp = vec2Perp(ra);
		Vector2 rbPerp = vec2Perp(rb);

		Vector2 angularLinearVelocity1 = vec2Scale(raPerp, object1->angularVelocity);
		Vector2 angularLinearVelocity2 = vec2Scale(rbPerp, object2->angularVelocity);

		Vector2 relativeVelocity = vec2Sub(vec2Add(object2->velocity, angularLinearVelocity2),
																			vec2Add(object1->velocity, angularLinearVelocity1));

		Vector2 tangent = vec2Sub(relativeVelocity, vec2Scale(normal, vec2Dot(relativeVelocity, normal)));

		// if the vector is nearly zero, stop this iteration
		if (fmaxf(vec2LengthSquared(tangent) - 0.3f, 0.0f) == 0) {
			continue;
		} else {
			tangent = vec2Normalize(tangent);
		}

		float raPerpDotTangent = vec2Dot(raPerp, tangent);
		float rbPerpDotTangent = vec2Dot(rbPerp, tangent);

		float denom = (object1->invMass + object2->invMass) +
									(raPerpDotTangent * raPerpDotTangent) * object1->invInertia +
									(rbPerpDotTangent * rbPerpDotTangent) * object2->invInertia;

		float jTangent = -vec2Dot(relativeVelocity, tangent);
		jTangent /= denom;
		jTangent /= (float)numContacts;

		Vector2 frictionImpulse;

		if (fabsf(jTangent) <= jArray[i] * staticFriction) {
			frictionImpulse = vec2Scale(tangent, jTangent);
		} else {
			frictionImpulse = vec2Scale(tangent, -jArray[i] * dynamicFriction);
		}

		frictionImpulseArray[i] = frictionImpulse;
	}

	for (int i = 0; i < numContacts; i++) {
		Vector2 frictionImpulse = frictionImpulseArray[i];
		Vector2 ra = raArray[i];
		Vector2 rb = rbArray[i];
		if (!(object1->isStaticBody)) {
			object1->velocity = vec2Add(object1->velocity, vec2Scale(frictionImpulse, object1->invMass));
			object1->angularVelocity += vec2Cross(ra, frictionImpulse) * -object1->invInertia;
		}
		if (!(object2->isStaticBody)) {
			object2->velocity = vec2Add(object2->velocity, vec2Scale(frictionImpulse, -object2->invMass));
			object2->angularVelocity += vec2Cross(rb, frictionImpulse) * object2->invInertia;
		}
	}
}


void resolveVelocity(physicsObject *object1, physicsObject *object2, collisionResult result) {

	Vector2 relativeVelocity = vec2Sub(object1->velocity, object2->velocity);
	float velocityProjection = vec2Dot(relativeVelocity, result.normal);

	float elasticity = 0.2f; // Adjust this value as needed
	float impulse = (-(1.0f + elasticity) * velocityProjection) / (object1->invMass + object2->invMass);

	Vector2 impulseVector = vec2Scale(result.normal, impulse);
	if (!object1->isStaticBody) {
		object1->velocity = vec2Add(object1->velocity, vec2Scale(impulseVector, object1->invMass));
	}
	if (!object2->isStaticBody) {
		object2->velocity = vec2Sub(object2->velocity, vec2Scale(impulseVector, object2->invMass));
	}
}

void handleCollision(physicsObject *object1, physicsObject *object2, collisionResult result) {
	// correct the collision normal
	if (vec2Dot(object1->position, result.normal) < vec2Dot(object2->position, result.normal)) {
		result.normal = vec2Negate(result.normal);
	}

	Vector2 penetration = vec2Scale(result.normal, result.penetrationDepth);
	separateBodies(object1, object2, penetration);
	resolveCollisionWithRotation(object1, object2, result);
}


void handleVelocity(physicsObject *object) {
	if (!(object->isStaticBody)) {
		object->velocity.y += (gravity * SUBSTEP_FACTOR);
		object->rotation += (object->angularVelocity * SUBSTEP_FACTOR) * 0.01;
	}

	if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
		if (CheckCollisionPointPoly(GetMousePosition(), object->collisionShape->globalPointArray, object->collisionShape->numPoints)) {
			Vector2 delta = GetMouseDelta();
			object->velocity = delta;
		}
	}
	// apply the position and multiply the velocity by the factor to keep it scaled properly
	object->position = vec2Add(object->position, vec2Scale(object->velocity, SUBSTEP_FACTOR));

	// iterate through every object and handle collisions
	for (int i = 0; i < objectCount; i++) {
		physicsObject *object2 = &objectArray[i];
		if (object != object2) {
			collisionResult result = polygonIntersect(object->collisionShape->numPoints, object->collisionShape->globalPointArray, object2->collisionShape->numPoints, object2->collisionShape->globalPointArray);
			if (result.isCollided) {
				polygonsContactPoints(
					object->collisionShape->globalPointArray,
					object->collisionShape->numPoints,
					object2->collisionShape->globalPointArray,
					object2->collisionShape->numPoints,
					&result.contact1,
					&result.contact2,
					&result.numContacts
				);
				DrawCircle(result.contact1.x,result.contact1.y, 10, BLUE);
				if (result.numContacts > 1) {
					DrawCircle(result.contact2.x,result.contact2.y, 10, RED);
				}
				handleCollision(object, object2, result);
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
	object.invMass = 1.0f / object.mass;
	object.position = center;
	object.velocity = (Vector2){0, 0};
	object.rotation = rotation;
	object.gravityStrength = gravityStrength;
	object.isStaticBody = isStaticBody;
	object.collisionShape = rectShape;
	object.inertia = 1.0f;
	object.invInertia = 1.0f / object.inertia;
	object.angularVelocity = 0.0f;
	object.staticFriction = 0.6f;
	object.dynamicFriction = 0.4f;

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
