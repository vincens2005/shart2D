float vec2Cross(Vector2 v1, Vector2 v2) {
  return (v1.x * v2.y) - (v1.y * v2.x);
}

Vector2 vec2Negate(Vector2 v) {
  return (Vector2){-v.x, -v.y};
}

Vector2 vec2Perp(Vector2 v) {
  return (Vector2){-v.y, v.x};
}

Vector2 vec2Add(Vector2 v1, Vector2 v2) {
  return (Vector2){v1.x + v2.x, v1.y + v2.y};
}

Vector2 vec2Sub(Vector2 v1, Vector2 v2) {
  return (Vector2){v1.x - v2.x, v1.y - v2.y};
}

Vector2 vec2Scale(Vector2 v, float scalar) {
  return (Vector2){v.x * scalar, v.y * scalar};
}

float vec2Dot(Vector2 v1, Vector2 v2) {
  return (v1.x * v2.x) + (v1.y * v2.y);
}

float vec2LengthSquared(Vector2 v) {
  return (v.x * v.x) + (v.y * v.y);
}

float vec2Length(Vector2 v) {
  return sqrt(vec2LengthSquared(v));
}

float vec2DistSquared(Vector2 v1, Vector2 v2) {
  Vector2 vdiff = vec2Sub(v1, v2);
  return vec2LengthSquared(vdiff);
}

float vec2Dist(Vector2 v1, Vector2 v2) {
  return sqrt(vec2DistSquared(v1, v2));
}

Vector2 vec2Normalize(Vector2 v) {
  float length = vec2Length(v);
  return (Vector2){v.x / length, v.y / length};
}
