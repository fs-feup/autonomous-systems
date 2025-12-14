#include <OsqpEigen/OsqpEigen.h>
#include <iostream>

int test() {
    std::cout << "OSQP-Eigen está instalado corretamente!" << std::endl;
    
    // Criar um solver vazio só para testar
    OsqpEigen::Solver solver;
    
    
    return 0;
}