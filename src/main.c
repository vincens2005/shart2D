#include <stdio.h>
#include "include/objects.h"

int main() {
	physicsBall b = {.base.weight = 1, .radius = 20};

	printf("i have a ball. It weighs %d grams. Its radius is %d pixels.\r\n", b.base.weight, b.radius);
	printf("that's it!!!\r\n");
	return 0;
}
