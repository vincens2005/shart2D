#include <stdio.h>
#include "include/raylib.h"
#include "include/objects.h"

int main() {
	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	InitWindow(640, 480, "physics moment!!!");
	SetTargetFPS(60); // 60 fps

	physicsBall b = {
		.base.weight = 1,
		.radius = 20,
		.base.position = {0, 0}
	};

	while (!WindowShouldClose()) {
		// todo: physics calculations

		// draw the ball
		BeginDrawing();

		ClearBackground(BLACK);

		Vector2 offset = {GetScreenWidth()/2.0f, GetScreenHeight()/2.0f};

		DrawCircleV((Vector2){b.base.position.x + offset.x, b.base.position.y + offset.y}, (float)b.radius, RED);

		DrawFPS(10, 10); // show current fps on screen

		EndDrawing(); // drawing done!
	}

	CloseWindow();
	return 0;
}
