_default:
        @just --unstable --list --list-heading $'Available commands:\n' --list-prefix $' - '


raylib:
        echo building raylib...
        cd raylib/src/ && \
        make PLATFORM=PLATFORM_DESKTOP

        cp raylib/src/raylib.h src/include/



build: raylib
        echo building physics engine...

        if [ ! -d "build" ]; then \
                mkdir build; \
        fi

        gcc -Wall -Lraylib/src -L/opt/vc/lib -Isrc/include src/main.c -o build/physics -lraylib -lm

        echo done!

clean:
        rm -rf build
        cd raylib/src/ && \
        make clean

run: raylib build
        ./build/physics

#.PHONY: all clean raylib
