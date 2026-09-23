#include <algorithm>
#include <iostream>
#include <numeric>
#include <ostream>
#include <string>
#include <vector>
#include <cstdint>
#include <map>
#include <ranges>

int ascii_sum(const std::string &text) {
    return std::accumulate(text.begin(), text.end(), 0,
                           [](int total, unsigned char c) {
                               return total + c;
                           });
}

struct Object {
    int id;
    std::uint64_t timestamp;
};

template<typename... Args>
void print(Args... args) {
    ((std::cout << args << ' '), ...);
    std::cout << '\n';
}

int main() {
    print(1, "hello", 3.14, 'c', 1313, 21312321, 123123, 12);
    print(42);
    print();

    std::vector<std::string> input{"cat", "robot", "C++3"};
    std::vector<int> output(input.size());

    std::transform(input.begin(), input.end(), output.begin(),
                   ascii_sum);

    auto found = std::find_if(input.begin(), input.end(), [](const auto &elem) -> bool {
        return elem.size() == 3;
    });

    std::cout << *found << std::endl;

    for (auto &i: input) {
        std::cout << i << std::endl;
    }

    for (auto &i: output) {
        std::cout << i << std::endl;
    }

    std::map<int, Object> semantic_map;

    semantic_map.emplace(0, Object{0, 45});
    semantic_map.emplace(1, Object{1, 3});
    semantic_map.emplace(2, Object{2, 12});

    auto removable =
            semantic_map
            | std::views::values
            | std::views::filter([](const Object &o) {
                return o.timestamp > 5;
            });

    std::vector<int> ids;
    for (const auto &object: removable) {
        ids.push_back(object.id);
    }

    for (int id: ids) {
        semantic_map.erase(id);
    }

    std::cout << "Remaining objects: "
            << semantic_map.size() << '\n';
}
