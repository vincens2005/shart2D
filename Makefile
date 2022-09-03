default:
	gcc src/main.c -o build/physics -Isrc/include

clean:
	rm -rf build
