#ifndef H_IRLS_ICP_PARAM_H
#define H_IRLS_ICP_PARAM_H

#include "BasicInfo.h"
#include "Param.h"

struct Param
{
    // 0: Welsch, 1: L1
    bool m_bLog;

    int m_IDMEstimator;
    
    double m_InitScale = 1.0;
    double m_DescendRate = 0.95;
    double m_MinScale = 0.1;
    double m_IterInitErr = 1e4;
    double m_IterQuitErr = 1e-4;
    int m_MaxIterTime = 30;
    
    Param()
    {
        m_IDMEstimator = 0;
        m_bLog = true;
    }

    Param(int IDMEstimator,bool bLog)
    {
        m_IDMEstimator = IDMEstimator;
        m_bLog = bLog;
    }

    void Load(const char *filename)
    {
        stringstream buffer;
        string line;
        string paramName;

        ifstream fin(filename); 
        if (!fin.good()) 
        { 
            cerr<<"No Parameters File!"<<endl;
        }

        while (fin.good())
        {
            getline(fin,line);
            if(line[0] == '#')
            {
                continue;
            }
            
            buffer.clear();
            buffer << line;
            buffer >> paramName;

            if(paramName.compare("m_bLog") == 0)
            {
                bool tmp;
                buffer >> tmp;
                m_bLog = tmp;
                continue;
            }

            if(paramName.compare("m_IDMEstimator") == 0)
            {
                int tmp;
                buffer >> tmp;
                m_IDMEstimator = tmp;
                continue;
            }

            if(paramName.compare("m_InitScale") == 0)
            {
                double tmp;
                buffer >> tmp;
                m_InitScale = tmp;
                continue;
            }

            if(paramName.compare("m_DescendRate") == 0)
            {
                double tmp;
                buffer >> tmp;
                m_DescendRate = tmp;
                continue;
            }

            if(paramName.compare("m_MinScale") == 0)
            {
                double tmp;
                buffer >> tmp;
                m_MinScale = tmp;
                continue;
            }

            if(paramName.compare("m_IterInitErr") == 0)
            {
                double tmp;
                buffer >> tmp;
                m_IterInitErr = tmp;
                continue;
            }

            if(paramName.compare("m_IterQuitErr") == 0)
            {
                double tmp;
                buffer >> tmp;
                m_IterQuitErr = tmp;
                continue;
            }

            if(paramName.compare("m_MaxIterTime") == 0)
            {
                int tmp;
                buffer >> tmp;
                m_MaxIterTime = tmp;
                continue;
            }

            cerr<<"Parameter File Wrong!"<<endl;
        }

    }

    void Show()
    {
        cout<<"-----------------Param------------------"<<endl;
        switch (m_IDMEstimator)
        {
        case 0:
            cout << "Weight Function: Welsch M-estimator" << endl;
            break;
        case 1:
            cout << "Weight Function: L1 Norm" << endl;
            break;
        }

        cout<<"Scale Init: "<<m_InitScale<<endl;
        cout<<"Scale Min: "<<m_MinScale<<endl;
        cout<<"Descend Rate: "<<m_DescendRate<<endl;

        cout<<"Iteration Times Max: "<<m_MaxIterTime<<endl;
        cout<<"Iteration Error Init: "<<m_IterInitErr<<endl;
        cout<<"Iteration Error Init: "<<m_IterQuitErr<<endl;
        
        cout<<"----------------------------------------"<<endl;
    }
};


#endif
