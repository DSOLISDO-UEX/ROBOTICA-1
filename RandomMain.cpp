#include <iostream>
#include <random>

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<int> dice(1, 6);
    std::uniform_real_distribution<double> percentage(0.0, 100.0);
    std::normal_distribution<double> height(170.0, 10.0);

    std::cout << "Dice rolls: ";
    for (int i = 0; i < 5; ++i) {
        std::cout << dice(gen) << ' ';
    }

    std::cout << "\nPercentage: " << percentage(gen) << '\n';
    std::cout << "Height sample: " << height(gen) << " cm\n";
}

