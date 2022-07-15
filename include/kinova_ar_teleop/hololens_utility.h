#ifndef _HOLOLENS_UTILITY_H_
#define _HOLOLENS_UTILITY_H_

/* 
* Utility used to control HoloLens 2 with Kinova Gen3 Dual Arm Setup
*
* Author: Frank Regal
* Date: 2022-07-15
*/

#include <string>

class HololensUtility
{

private:
    std::string test_string;

public:
    HololensUtility();
    ~HololensUtility();

    void GetTestString();

};

#endif //_HOLOLENS_UTILITY_H_


