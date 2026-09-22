//
// Created by usuario on 22/9/26.
//
#include <vector>
#include <iostream>
#include <chrono>
#include <map>
#include <random>

int receptoraVectoresValor(std::vector<int> vector) {
    return vector.size();
}

int receptoraVectoresReferencia( std::vector<int> &vector) {
    auto local_var = std::move(vector);

    return vector.size();
}

int receptoraVectoresMoveSemantic(std::vector<int> &&vector) {
    auto local_var = vector;
    return vector.size();
}

int main() {
    std::vector<int> vector10Gb(2684354560);
    {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        receptoraVectoresValor(vector10Gb);
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Función receptora Vector Valor 10Gb: " << ms  << std::endl;
    }
    {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        receptoraVectoresReferencia(vector10Gb);
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Función receptora Vector Referencia 10Gb: " << ms  << std::endl;
    }
    {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        receptoraVectoresMoveSemantic(std::move(vector10Gb));
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Función receptora Vector Move Semantics 10Gb: " << ms  << std::endl;
    }
    /*{
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::sort(vector10Gb.begin(), vector10Gb.end());
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Sort del Vector de 10Gb: " << ms / (10e9)  << std::endl;
    }
    {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::min_element(vector10Gb.begin(), vector10Gb.end());
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Sort del Vector de 10Gb: " << ms / (10e9)  << std::endl;
    }

    {
        std::cout << "Imprimit2o los 5 primeros elementos del vector: " << std::endl;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::for_each(vector10Gb.begin(), vector10Gb.begin()+5, [](int &elem) { std::cout << elem << ", "; });
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Coger los 5 primeros elementos Vector de 10Gb: " << ms / (10e9)  << std::endl;
    }
    {
        std::cout << "Hacit2o shuffle los 5 primeros elementos del vector: " << std::endl;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::shuffle(vector10Gb.begin(), vector10Gb.end(), std::default_random_engine(std::random_device{}()));
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        std::cout << "Shuffle del Vector de 10Gb: " << ms / (10e9)  << std::endl;
    }*/

    return 0;
}
