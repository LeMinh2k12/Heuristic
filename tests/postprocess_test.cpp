// Exercise walking-block optimisation independently of the heuristic search.
#define main solver_main
#include "../main.cpp"
#undef main

static void check(const char* expected) {
    FastIO::op = 0;
    outputOperations();
    const string actual(FastIO::out, FastIO::op);
    if (actual != expected) {
        cerr << "Unexpected operations:\n" << actual << "Expected:\n" << expected;
        exit(1);
    }
}

int main() {
    N = 128;
    M = 3;
    gx[0] = 0; gy[0] = 0;
    gx[1] = 1; gy[1] = 1;
    gx[2] = 0; gy[2] = 1;
    gx[3] = 0; gy[3] = 4;
    operations = {{0,'D'}, {0,'R'}, {0,'L'}, {0,'U'}, {0,'R'},
                  {(uint16_t)cell(0,2),'J'}, {0,'R'}};
    check("D\nR\nU\nJ 0 2\nR\n");

    // Taking a different shortest path visits goal 2 before a teleport.
    // The subsequent walking block can then skip that waypoint.
    gx[2] = 1; gy[2] = 3;
    gx[3] = 4; gy[3] = 4;
    operations = {{0,'D'}, {0,'R'}, {0,'D'}, {0,'D'}, {0,'R'}, {0,'R'},
                  {(uint16_t)cell(3,4),'J'},
                  {0,'U'}, {0,'U'}, {0,'L'}, {0,'L'},
                  {0,'D'}, {0,'D'}, {0,'D'}, {0,'R'}};
    check("D\nR\nR\nR\nD\nD\nJ 3 4\nD\nL\n");
    return 0;
}
