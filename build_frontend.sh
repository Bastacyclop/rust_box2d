g++ -c src/box2d_frontend.cpp -o $DEPS_DIR/box2d_frontend.o -std=c++11 -Isrc -fPIC -Wall
ar rcs $DEPS_DIR/libbox2d_frontend.a $DEPS_DIR/box2d_frontend.o
