float getOverlap(Vector2 axis, int numPoints1, Vector2* points1, int numPoints2, Vector2* points2) {
  float min1 = INFINITY;
  float max1 = -INFINITY;
  // Find the extreme points on each axis for the first polygon
  for (int i = 0; i < numPoints1; i++) {
    float dotProduct = vec2Dot(points1[i], axis);
    if (dotProduct < min1) min1 = dotProduct;
    if (dotProduct > max1) max1 = dotProduct;
  }

  float min2 = INFINITY;
  float max2 = -INFINITY;
  // Do the same thing again for the second polygon
  for (int i = 0; i < numPoints2; i++) {

    float dotProduct = vec2Dot(points2[i], axis);
    if (dotProduct < min2) min2 = dotProduct;
    if (dotProduct > max2) max2 = dotProduct;
  }

  // Check for collision using the extreme points
  float overlap1 = max1 - min2;
  float overlap2 = max2 - min1;

  // Check for overlap and return the smaller overlap value
  if (overlap1 < 0 || overlap2 < 0) {
    // No overlap
    return 0.0f;
  } else {
    return fminf(overlap1, overlap2);
  }
}

void pointSegmentDistance(Vector2 p, Vector2 point1, Vector2 point2, float *distSq, Vector2 *cp) {
    float l2 = vec2DistSquared(point1, point2);
    if (l2 == 0.0f) {
        *distSq = vec2DistSquared(p, point1);
        *cp = point1;
    } else {
        float t = fmaxf(0.0f, fminf(1.0f, ((p.x - point1.x) * (point2.x - point1.x) + (p.y - point1.y) * (point2.y - point1.y)) / l2));
        Vector2 projection = {point1.x + t * (point2.x - point1.x), point1.y + t * (point2.y - point1.y)};
        *distSq = vec2DistSquared(p, projection);
        *cp = projection;
    }
}

void findPolygonContactPoints(
    Vector2 *points1, int numPoints1,
    Vector2 *points2, int numPoints2,
    Vector2 *contact1, Vector2 *contact2, int *contactCount) {

    *contact1 = (Vector2){0,0};
    *contact2 = (Vector2){0,0};
    *contactCount = 0;

    float minDistSq = INFINITY;

    for (int i = 0; i < numPoints1; i++) {
        Vector2 p = points1[i];

        for (int j = 0; j < numPoints2; j++) {
            Vector2 point1 = points2[j];
            Vector2 point2 = points2[(j + 1) % numPoints2];

            float distSq;
            Vector2 cp;
            pointSegmentDistance(p, point1, point2, &distSq, &cp);

            if (fabsf(distSq - minDistSq) < FLT_EPSILON) {
                if (!(cp.x == contact1->x && cp.y == contact1->y)) {
                    *contact2 = cp;
                    *contactCount = 2;
                }
            } else if (distSq < minDistSq) {
                minDistSq = distSq;
                *contactCount = 1;
                *contact1 = cp;
            }
        }
    }

    for (int i = 0; i < numPoints2; i++) {
        Vector2 p = points2[i];

        for (int j = 0; j < numPoints1; j++) {
            Vector2 point1 = points1[j];
            Vector2 point2 = points1[(j + 1) % numPoints1];

            float distSq;
            Vector2 cp;
            pointSegmentDistance(p, point1, point2, &distSq, &cp);

            if (fabsf(distSq - minDistSq) < FLT_EPSILON) {
                if (!(cp.x == contact1->x && cp.y == contact1->y)) {
                    *contact2 = cp;
                    *contactCount = 2;
                }
            } else if (distSq < minDistSq) {
                minDistSq = distSq;
                *contactCount = 1;
                *contact1 = cp;
            }
        }
    }
}

collisionResult polygonIntersect(physicsObject *object1, physicsObject *object2) {

  int numPoints1 = object1->collisionShape->numPoints;
  int numPoints2 = object2->collisionShape->numPoints;
  Vector2 *points1 = object1->collisionShape->globalPointArray;
  Vector2 *points2 = object2->collisionShape->globalPointArray;

  collisionResult result;
  result.normal = (Vector2){0,0};
  result.object1 = object1;
  result.object2 = object2;
  result.isCollided = false;
  result.penetrationDepth = 0.0f;

  float minOverlap = INFINITY;
  // iterate over every single edge on the first shape and check for a seperating axis
  for (int i = 0; i < numPoints1;  i++) {
    int nextIndex = (i + 1) % numPoints1;
    Vector2 edge = (Vector2){points1[i].x - points1[nextIndex].x, points1[i].y - points1[nextIndex].y};
    Vector2 axis = vec2Normalize(vec2Perp(edge));
    float overlap = getOverlap(axis, numPoints1, points1, numPoints2, points2);
    if (overlap == 0.0f) {
      return result;
    } else if (overlap < minOverlap){
        result.normal = axis;
        result.penetrationDepth = overlap;
        minOverlap = overlap;
    }
  }
  // iterate over the second shape
  for (int i = 0; i < numPoints2;  i++) {
    int nextIndex = (i + 1) % numPoints2;
    Vector2 edge = (Vector2){points2[i].x - points2[nextIndex].x, points2[i].y - points2[nextIndex].y};
    Vector2 axis = vec2Normalize(vec2Perp(edge));
    float overlap = getOverlap(axis, numPoints1, points1, numPoints2, points2);
    if (overlap == 0.0f) {
      return result;
    } else if (overlap < minOverlap){
        result.normal = axis;
        result.penetrationDepth = overlap;
        minOverlap = overlap;
    }

  }
  result.isCollided = true;
  findPolygonContactPoints(
		object1->collisionShape->globalPointArray,
		object1->collisionShape->numPoints,
		object2->collisionShape->globalPointArray,
		object2->collisionShape->numPoints,
		&result.contact1,
		&result.contact2,
		&result.numContacts
	);
  if (vec2Dot(object1->position, result.normal) < vec2Dot(object2->position, result.normal)) {
		result.normal = vec2Negate(result.normal);
	}
  // if there are no seperating axes, the shapes have indeed collided
  return result;
}

bool AABBIntersect(polygonCollisionShape *shape1, polygonCollisionShape *shape2) {
    return shape1->min.x < shape2->max.x &&
            shape1->max.x > shape2->min.x &&
            shape1->min.y < shape2->max.y &&
            shape1->max.y > shape2->min.y;
}

