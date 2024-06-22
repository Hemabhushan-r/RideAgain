default: 
	gcc.exe tetris_main.c -o tetris.exe -O2 -Wall -Wno-missing-braces -L ./lib/ -lraylib -s -static -lopengl32 -lgdi32 -lwinmm