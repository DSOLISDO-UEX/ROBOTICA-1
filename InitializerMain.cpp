#include <iostream>
#include <random>

struct Range {
    float init = 0.0f;
    float end = 0.0f;
    float step = 1.0f;
};

int main() {
    auto range = Range{
        .init = 1.0f,
        .end = 2.0f,
        .step = 0.1f
    };

    return 0;
}
