#include <stdio.h>
#include <stdlib.h>
#include "include/raylib.h"
#include "include/objects.h"

rigidBox boxArray[] = {
	{
		.base.weight = 1,
		.dimensions = {40,40},
		.base.position = {40,40}
	},
	// {
	// 	.base.weight = 1,
	// 	.dimensions = {40,60},
	// 	.base.position = {40,-40}
	// },
	// {
	// 	.base.weight = 1,
	// 	.dimensions = {100,60},
	// 	.base.position = {-40,10}
	// }
};

float gravity = 10;

Vector2 offset = {0,0};

Vector2 addOffset(Vector2 position){
	return (Vector2){position.x + offset.x, position.y + offset.y};
}

void drawShapes(){
	// defining list of colors for epick
	Color colors[12] = {BLUE,RED,ORANGE,PURPLE,GREEN,LIME,VIOLET,DARKBLUE,SKYBLUE,MAROON,BROWN,BEIGE};
	//boxes
	for (int i = 0; i < sizeof(boxArray)/sizeof(rigidBox); i++){
		rigidBox box = boxArray[i];
		box.base.velocity.y = box.base.velocity.y + gravity;
		box.base.position = (Vector2){box.base.position.x + box.base.velocity.x, box.base.position.y + box.base.velocity.y};
		printf("box y position: %f \n",box.base.position.y);
		DrawRectangleV(addOffset(box.base.position),box.dimensions,colors[i % 12] /*inputting colors*/);
	}
}

int main() {
	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	InitWindow(640, 480, "hyper realistic physics engine developed by epick game company llc");
	SetTargetFPS(60); // 60 fps
	while (!WindowShouldClose()) {
		// todo: physics calculations
		// draw the ball
		BeginDrawing();
		ClearBackground(BLACK);
		// drawing shapes or something idfk
		drawShapes();
		offset = (Vector2){GetScreenWidth()/2.0f,GetScreenHeight()/2.0f};
		DrawFPS(10, 10); // show current fps on screen

		EndDrawing(); // drawing done!
	}
	CloseWindow();
	return 0;
}
