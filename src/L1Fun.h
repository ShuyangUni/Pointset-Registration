#ifndef H_L1_FUN_H
#define H_L1_FUN_H
#include "BaseFun.h"

class L1Fun : public BaseFun
{
public:
    void CalcWeight(const vector<float> &vecResidual, vector<float> &vecWeight, double &Scale)
    {
        vecWeight.clear();
        for (int i = 0; i < vecResidual.size(); ++i)
        {
            double tmpWeight = 1/(fabs(vecResidual.at(i))+0.00001);
            vecWeight.push_back(tmpWeight);
        }
    }
};

#endif