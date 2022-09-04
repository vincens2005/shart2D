#include <stdio.h>
#include <stdlib.h>
#include "include/raylib.h"
#include "include/objects.h"

epickBox boxArray[] = {
	{
		.base.weight = 1,
		.dimensions = {40,40},
		.base.position = {40,40}
	},
	{
		.base.weight = 1,
		.dimensions = {40,60},
		.base.position = {40,-40}
	},
	{
		.base.weight = 1,
		.dimensions = {100,60},
		.base.position = {-40,10}
	}
};

Vector2 offset = {0,0};

Vector2 addOffset(Vector2 position){
	return (Vector2){position.x + offset.x, position.y + offset.y};
}

void drawShapes(){
	// defining list of colors for epick
	Color colors[] = {BLUE,RED,ORANGE,PURPLE,GREEN,LIME,VIOLET,DARKBLUE,SKYBLUE,MAROON,BROWN,BEIGE};
	//boxes
	for (int i = 0; i < sizeof(boxArray)/sizeof(epickBox); i++){
		epickBox box = boxArray[i];
		DrawRectangleV(addOffset(box.base.position),box.dimensions,colors[i % (sizeof(colors)/sizeof(Color))] /*inputting colors*/);
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
