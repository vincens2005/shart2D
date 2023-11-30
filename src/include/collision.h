
float vec2Length(Vector2 v) {
  return sqrt((v.x * v.x) + (v.y * v.y));
}

Vector2 vec2Normalize(Vector2 v) {
  float length = vec2Length(v);
  return (Vector2){v.x / length, v.y / length};
}

float getOverlap(Vector2 axis, int numPoints1, Vector2* points1, int numPoints2, Vector2* points2) {
  float min1 = INFINITY;
  float max1 = -INFINITY;
  // Find the extreme points on each axis for the first polygon
  for (int i = 0; i < numPoints1; i++) {
    float dotProduct = points1[i].x * axis.x + points1[i].y * axis.y;
    if (dotProduct < min1) min1 = dotProduct;
    if (dotProduct > max1) max1 = dotProduct;
  }

  float min2 = INFINITY;
  float max2 = -INFINITY;
  // Do the same thing again for the second polygon
  for (int i = 0; i < numPoints2; i++) {
    float dotProduct = points2[i].x * axis.x + points2[i].y * axis.y;
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

collisionResult polygonIntersect(int numPoints1, Vector2* points1, int numPoints2, Vector2* points2) {
  collisionResult result;
  result.normal = (Vector2){0,0};
  result.isCollided = false;
  result.penetrationDepth = 0.0f;

  float minOverlap = INFINITY;
  // iterate over every single edge on the first shape and check for a seperating axis
  for (int i = 0; i < numPoints1;  i++) {
    int nextIndex = (i + 1) % numPoints1;
    Vector2 edge = (Vector2){points1[i].x - points1[nextIndex].x, points1[i].y - points1[nextIndex].y};
    Vector2 axis = vec2Normalize((Vector2){-edge.y, edge.x});
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
    Vector2 axis = vec2Normalize((Vector2){-edge.y, edge.x});
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
  // if there are no seperating axes, the shapes have indeed collided
  return result;
}

