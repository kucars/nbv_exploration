#include <iostream>
#include <class_ViewGenerator.hpp>

// ============
// Base class
// ============
ViewGenerator_Base::ViewGenerator_Base(){}
ViewGenerator_Base::~ViewGenerator_Base(){}
void ViewGenerator_Base::generate()
{
    std::cout << "[ERROR] Call to ViewGenerator_Base::generate(). Impliment function in derived class" << std::endl;
}


// ==============
// Frontier class
// ==============
ViewGenerator_Frontier::ViewGenerator_Frontier(){}
ViewGenerator_Frontier::~ViewGenerator_Frontier(){}

void ViewGenerator_Frontier::generate(){
    std::cout << "Frontier::generate()" << std::endl;
}
