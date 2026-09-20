#define _CRT_SECURE_NO_WARNINGS
// #include "demos/city_demo.hpp"
#include "demos/animation_demo.hpp"

// ---- which GPU this runs on ----
//
// A laptop has two, and the one it hands an OpenGL context to by default is
// the integrated one. That is not a small difference for a renderer like this
// one, which is bound by the driver's per-draw cost far more than by anything
// the GPU does: Intel's OpenGL driver charges something like twenty
// microseconds a draw where NVIDIA's charges one or two, so a few hundred
// draws a frame is the whole frame budget on one and a rounding error on the
// other.
//
// Both vendors watch for these two exported symbols in the executable and use
// them to pick the high-performance adapter. They have to be exported from the
// .exe itself, which is why they live here and not in a header.
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 1;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}

int main() {
    // test_city_demo();
    test_animation_demo();
    return 0;
}
