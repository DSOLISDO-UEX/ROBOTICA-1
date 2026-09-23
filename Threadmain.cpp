#include <chrono>
#include <iostream>
#include <utility>
#include <vector>
#include <functional>
#include <execution>
#include <algorithm>

using Clock = std::chrono::steady_clock;

void by_value(std::vector<char> v) {
    std::cout << "size = " << v.size() << '\n';
}

void by_ref(std::vector<char> &v) {
    std::cout << "size = " << v.size() << '\n';
}

void by_const_ref(const std::vector<char> &v) {
    std::cout << "size = " << v.size() << '\n';
}

void by_rvalue_ref(std::vector<char> &&v) {
    std::cout << "size = " << v.size() << '\n';
}

template<typename F>
void measure(const char *label, F &&f) {
    const auto start = Clock::now();
    std::forward<F>(f)();
    const auto end = Clock::now();

    const auto us =
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << label << ": " << us.count() << " ms\n";
}

int main() {
    constexpr std::size_t MiB = 1024ULL * 1024ULL;
    constexpr std::size_t size = 100; // Start small; increase carefully.

    std::vector<char> v(size * MiB, 'x');

    measure("lvalue reference", [&] { by_ref(v); });
    measure("const reference", [&] { by_const_ref(v); });

    measure("copy by value", [&] { by_value(v); });

    measure("move into by-value", [&] {
        std::vector<char> copy = v;
        by_value(std::move(copy));
    });

    measure("sort paralel",
            [&]() {
                std::sort(std::execution::par, v.begin(), v.end(), [&](char a, char b) -> bool {
                    return a < b;
                });
            }
    );

    measure("sort",
            [&]() {
                std::sort(v.begin(), v.end(), [&](char a, char b) -> bool {
                    return a < b;
                });
            }
    );
}
