#ifndef H_WELSCH_FUN_H
#define H_WELSCH_FUN_H
#include "BaseFun.h"

class WelschFun : public BaseFun
{
public:
    void CalcWeight(const vector<float> &vecResidual, vector<float> &vecWeight, double &Scale)
    {
        float c = 2.9846;
        vecWeight.clear();
        for (int i = 0; i < vecResidual.size(); ++i)
        {
            float tmpValue = vecResidual.at(i) / Scale / c;
            float tmpWeight = exp(-pow(tmpValue, 2));
            vecWeight.push_back(tmpWeight);
        }
    }
};

#endif