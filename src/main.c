#include <stdio.h>
#include "include/raylib.h"
#include "include/objects.h"

int main() {

	InitWindow(640, 480, "physics moment!!!");
	SetTargetFPS(60); // 60 fps

	physicsBall b = {
	.base.weight = 1,
	.radius = 20,
	.base.position = {GetScreenWidth()/2.0f, GetScreenHeight()/2.0f}
	};

	while (!WindowShouldClose()) {
		// todo: physics calculations

		// draw the ball
		BeginDrawing();

		ClearBackground(BLACK);

		DrawCircleV(b.base.position, (float)b.radius, RED);

		DrawFPS(10, 10); // show current fps on screen

		EndDrawing(); // drawing done!
	}

	CloseWindow();
	return 0;
}
