default:
	gcc.exe mainscreen.c -o mainscreen.exe -O2 -Wall -Wno-missing-braces -L ./lib/ -lraylib -s -static -lopengl32 -lgdi32 -lwinmm