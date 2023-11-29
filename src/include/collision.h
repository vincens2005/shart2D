#include "types.h"
#include "raylib.h"
#include <stdio.h>
#include <stdbool.h>



bool isSeperatingAxis(Vector2 axis, int numPoints1, Vector2* points1, int numPoints2, Vector2* points2) {
  float min1 = INFINITY;
  float max1 = -INFINITY;
  // find the extreme points on each axis for the first polygon
  for (int i = 0; i < numPoints1; i++) {
    float dotProduct = points1[i].x * axis.x + points1[i].y * axis.y;
    if (dotProduct < min1) min1 = dotProduct;
    if (dotProduct > max1) max1 = dotProduct;
  }

  float min2 = INFINITY;
  float max2 = -INFINITY;
  // do the same thing again for the second polygon
  for (int i = 0; i < numPoints2; i++) {
    float dotProduct = points2[i].x * axis.x + points2[i].y * axis.y;
    if (dotProduct < min2) min2 = dotProduct;
    if (dotProduct > max2) max2 = dotProduct;
  }

  // check for collision using the extreme points
  return !(max1 >= min2 && max2 >= min1);
}

bool arePolygonsIntersecting(int numPoints1, Vector2* points1, int numPoints2, Vector2* points2) {
  // iterate over every single edge on the first shape and check for a seperating axis
  for (int i = 0; i < numPoints1;  i++) {
    int nextIndex = (i + 1) % numPoints1;
    Vector2 edge = (Vector2){points1[i].x - points1[nextIndex].x, points1[i].y - points1[nextIndex].y};
    Vector2 axis = (Vector2){-edge.y, edge.x};
    if (isSeperatingAxis(axis, numPoints1, points1, numPoints2, points2)) {
      return false;
    }
  }
  // iterate over the second shape
  for (int i = 0; i < numPoints2;  i++) {
    int nextIndex = (i + 1) % numPoints2;
    Vector2 edge = (Vector2){points2[i].x - points2[nextIndex].x, points2[i].y - points2[nextIndex].y};
    Vector2 axis = (Vector2){-edge.y, edge.x};
    if (isSeperatingAxis(axis, numPoints1, points1, numPoints2, points2)) {
      return false;
    }
  }
  // if there are no seperating axes, the shapes have indeed collided
  float divideByZero = 10 / 0;
  return true;
}

