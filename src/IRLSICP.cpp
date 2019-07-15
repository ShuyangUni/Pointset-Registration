#include "IRLSICP.h"

/**
 *  Constructor
 */

IRLSICP::IRLSICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ref,
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_mov,
                 const Eigen::Affine3f &transformation,
                 const Param &param)
{
    // PointCLoud
    m_RefData.resize(0);
    for (int i = 0; i < cloud_ref->points.size(); ++i)
    {
        pcl::PointXYZ pt;
        pt.x = cloud_ref->points.at(i).x;
        pt.y = cloud_ref->points.at(i).y;
        pt.z = cloud_ref->points.at(i).z;
        m_RefData.push_back(pt);
    }

    m_MovData.resize(0);
    for (int i = 0; i < cloud_mov->points.size(); ++i)
    {
        pcl::PointXYZ pt;
        pt.x = cloud_mov->points.at(i).x;
        pt.y = cloud_mov->points.at(i).y;
        pt.z = cloud_mov->points.at(i).z;
        m_MovData.push_back(pt);
    }

    // Correspondance
    m_CorEst.setInputTarget(m_RefData.makeShared());

    // Init Transformation
    m_Transformation = transformation;

    pcl::transformPointCloud(m_MovData, m_AftData, m_Transformation);

    // Param
    m_Param = param;

    if(m_Param.m_bLog)
    {
        cout<<"RefData Size: "<<cloud_ref->points.size()
            <<" MovData Size: "<<cloud_mov->points.size()<<endl;
        m_Param.Show();
    }
    

    switch (m_Param.m_IDMEstimator)
    {
    case 0:
        m_WeightFun = new WelschFun();
        break;
    case 1:
        m_WeightFun = new L1Fun();
        break;
    }
}

IRLSICP::IRLSICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ref,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_mov,
                 const Eigen::Affine3f &transformation,
                 const Param &param)
{
    // PointCLoud
    m_RefData = *cloud_ref;
    m_MovData = *cloud_mov;

    // Correspondance
    m_CorEst.setInputTarget(m_RefData.makeShared());

    // Init Transformation
    m_Transformation = transformation;

    pcl::transformPointCloud(m_MovData, m_AftData, m_Transformation);

    // Param
    m_Param = param;

    if(m_Param.m_bLog)
    {
        cout<<"RefData Size: "<<cloud_ref->points.size()
            <<" MovData Size: "<<cloud_mov->points.size()<<endl;
        m_Param.Show();
    }

    switch (m_Param.m_IDMEstimator)
    {
    case 0:
        m_WeightFun = new WelschFun();
        break;
    case 1:
        m_WeightFun = new L1Fun();
        break;
    }
}

/**
 *  Public Function
 */
void IRLSICP::run()
{
    if(m_Param.m_bLog)
    {
        cout<<"Running..."<<endl;
    }
    
    m_Scale = m_Param.m_InitScale;
    m_DescendRate = m_Param.m_DescendRate;
    m_ScaleMin = m_Param.m_MinScale;
    m_IterErr = m_Param.m_IterInitErr;

    if (m_MovData.points.size() != 0 && m_RefData.points.size() != 0)
    {
        ICPIteration();
    }
    else
    {
        cerr << "Null Input!" << endl;
    }
}

void IRLSICP::getAftData(pcl::PointCloud<pcl::PointXYZ> &cloud_aft)
{
    cloud_aft = m_AftData;
}

void IRLSICP::getTransformation(Eigen::Affine3f &transformation)
{
    transformation = m_Transformation;
}

void IRLSICP::getRotation(Eigen::Matrix3f &rotation)
{
    rotation = m_Transformation.rotation();
}

void IRLSICP::getTranslation(Eigen::Vector3f &translation)
{
    translation << m_Transformation.translation()(0,0), 
                m_Transformation.translation()(0,1), 
                m_Transformation.translation()(0,2);
}

/**
 *  Private Function
 */

void IRLSICP::ICPIteration()
{
    int i;
    for (i = 0; i < m_Param.m_MaxIterTime; ++i)
    {
        vector<int> vecIdx;
        vector<float> vecResidual;

        FindClosestPt(vecIdx, vecResidual);
    
        vector<float> vecWeight;
        m_WeightFun->CalcWeight(vecResidual, vecWeight, m_Scale);
        
        pcl::PointCloud<pcl::PointXYZ> m_RefDataAligned;
        for (int j = 0; j < vecIdx.size(); ++j)
        {
            m_RefDataAligned.points.push_back(m_RefData.points.at(vecIdx.at(j)));
        }
        UpdateTransformation(m_RefDataAligned, vecWeight);
        
        if (m_IterErr < m_Param.m_IterQuitErr)
        {
            break;
        }
        UpdateScale();
    }

    if(m_Param.m_bLog)
    {
        cout<<"Iteration Times: "<<i<<endl;
        cout<<"Final Error: "<<m_IterErr<<endl;
        cout<<"Final Scale: "<<m_Scale<<endl;
    }
}

void IRLSICP::FindClosestPt(vector<int> &vecIdx, vector<float> &vecResidual)
{
    vecIdx.clear();
    vecResidual.clear();

    m_CorEst.setInputSource(m_AftData.makeShared());

    m_Correspondences.clear();
    m_CorEst.determineCorrespondences(m_Correspondences);

    for (int j = 0; j < m_AftData.points.size(); ++j)
    {
        vecIdx.push_back(m_Correspondences.at(j).index_match);
        vecResidual.push_back(m_Correspondences.at(j).distance);
    }
}

void IRLSICP::UpdateTransformation(const pcl::PointCloud<pcl::PointXYZ> &RefDataAligned, const vector<float> &vecWeight)
{
    double WeightSum;
    WeightSum = 0;

    for (int i = 0; i < vecWeight.size(); ++i)
    {
        WeightSum += vecWeight.at(i);
    }

    Eigen::Vector3f pMean;
    pMean << 0.0, 0.0, 0.0;
    for (int i = 0; i < m_AftData.points.size(); ++i)
    {
        Eigen::Vector3f pt;
        pt << m_AftData.points.at(i).x, m_AftData.points.at(i).y, m_AftData.points.at(i).z;
        pMean += vecWeight.at(i) * pt;
    }
    pMean = pMean / WeightSum;
    // cout<<pMean<<endl;

    Eigen::Vector3f yMean;
    yMean << 0.0, 0.0, 0.0;
    for (int i = 0; i < RefDataAligned.points.size(); ++i)
    {
        Eigen::Vector3f pt;
        pt << RefDataAligned.points.at(i).x, RefDataAligned.points.at(i).y, RefDataAligned.points.at(i).z;
        yMean += vecWeight.at(i) * pt;
    }
    yMean = yMean / WeightSum;
    // cout<<yMean<<endl;

    Eigen::Matrix3f C;
    C = -pMean * yMean.transpose();
    // cout<<C<<endl;

    for (int i = 0; i < vecWeight.size(); ++i)
    {
        Eigen::Vector3f p;
        p << m_AftData.points.at(i).x, m_AftData.points.at(i).y, m_AftData.points.at(i).z;

        Eigen::Vector3f y;
        y << RefDataAligned.points.at(i).x, RefDataAligned.points.at(i).y, RefDataAligned.points.at(i).z;

        C += (vecWeight.at(i) * p * y.transpose()) / WeightSum;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(C, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();

    Eigen::Matrix3f dR = V * U.transpose();
    if (dR.determinant() < 0.0)
    {
        Eigen::Matrix3f tmpM = Eigen::Matrix3f::Identity();
        tmpM(2, 2) = -1;
        dR = V * tmpM * U.transpose();
    }

    Eigen::Vector3f dT;
    dT = yMean - dR * pMean;

    // Update
    Eigen::Matrix3f R = m_Transformation.rotation();
    Eigen::Vector3f T = m_Transformation.translation();

    R = dR * R;
    T = dR * T + dT;

    Eigen::Quaternionf quaternion(R);
    Eigen::Translation3f translation(T(0), T(1), T(2));
    Eigen::Affine3f tmpTransformation = translation * quaternion.toRotationMatrix();
    m_Transformation = tmpTransformation;

    pcl::transformPointCloud(m_MovData, m_AftData, m_Transformation);

    m_IterErr = dT.norm();
}

void IRLSICP::UpdateScale()
{
    if (m_Scale > m_ScaleMin)
    {
        m_Scale = m_DescendRate * (m_Scale - m_ScaleMin) + m_ScaleMin;
    }
    else
    {
        m_Scale = m_ScaleMin;
    }
}