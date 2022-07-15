#include <iostream>
#include "kinova_ar_teleop/hololens_utility.h"

HololensUtility::HololensUtility() : test_string{"Hello World"}
{

}

HololensUtility::~HololensUtility()
{
    
}

void HololensUtility::GetTestString()
{
    std::cout << test_string << std::endl;
}