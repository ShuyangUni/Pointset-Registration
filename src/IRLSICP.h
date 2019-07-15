#ifndef H_IRLS_ICP_H
#define H_IRLS_ICP_H

#include "BasicInfo.h"
#include "Param.h"
#include "WelschFun.h"
#include "L1Fun.h"

class IRLSICP
{
public:
    IRLSICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ref,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_mov,
            const Eigen::Affine3f &transformation,
            const Param &param);
    IRLSICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ref,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_mov,
            const Eigen::Affine3f &transformation,
            const Param &param);
    void run(); 

    void getAftData(pcl::PointCloud<pcl::PointXYZ> &cloud_aft);
    void getTransformation(Eigen::Affine3f &transformation);
    void getRotation(Eigen::Matrix3f &rotation);
    void getTranslation(Eigen::Vector3f &translation);
private:
    /**
    *  Function
    */
   void ICPIteration();
   void FindClosestPt(vector<int> &vecIdx, vector<float> &vecResidual);

   void UpdateTransformation(const pcl::PointCloud<pcl::PointXYZ> &RefDataAligned, const vector<float> &vecWeight);

   void UpdateScale();

    /**
     *  Parameter
    */
    pcl::PointCloud<pcl::PointXYZ> m_RefData;
    pcl::PointCloud<pcl::PointXYZ> m_MovData;
    pcl::PointCloud<pcl::PointXYZ> m_AftData;

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> m_CorEst;
    pcl::Correspondences m_Correspondences;

    Eigen::Affine3f m_Transformation;

    Param m_Param;

    BaseFun* m_WeightFun;
    double m_Scale;
    double m_DescendRate;
    double m_ScaleMin;
    double m_IterErr;
};

#endif
