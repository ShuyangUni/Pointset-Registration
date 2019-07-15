#ifndef H_BASE_FUN_H
#define H_BASE_FUN_H
#include "BasicInfo.h"

class BaseFun
{
public:
    virtual void CalcWeight(const vector<float> &vecResidual, vector<float> &vecWeight, double &Scale) = 0;
};

#endif