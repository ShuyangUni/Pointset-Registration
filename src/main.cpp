#include "IRLSICP.h"
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // Load reference Data
    string strFrameMov = "1561720805.261288000.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMov(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(strFrameMov, *cloudMov) == -1)
    {
        cout << "LoadPCDFile is wrong!" << endl;
        return -1;
    }

    // Load moving Data
    string strFrameRef = "1561720805.362049000.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRef(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(strFrameRef, *cloudRef) == -1)
    {
        cout << "LoadPCDFile is wrong!" << endl;
        return -1;
    }

    //Outliers Remove
    vector<int> vIndiceMov,vIndiceRef;
    pcl::removeNaNFromPointCloud(*cloudMov, *cloudMov, vIndiceMov);
    pcl::removeNaNFromPointCloud(*cloudRef, *cloudRef, vIndiceRef);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMovFilter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRefFilter(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < cloudMov->points.size(); ++i)
    {
        pcl::PointXYZ pt = cloudMov->points.at(i);
        if(pt.x>-2.3 && pt.x<0.6 && pt.y>-0.7 && pt.y<0.7)
        {
            continue;
        }
        cloudMovFilter->points.push_back(pt);
    }

    for (int i = 0; i < cloudRef->points.size(); ++i)
    {
        pcl::PointXYZ pt = cloudRef->points.at(i);
        if(pt.x>-2.3 && pt.x<0.6 && pt.y>-0.7 && pt.y<0.7)
        {
            continue;
        }
        cloudRefFilter->points.push_back(pt);
    }

        // DownSample
    pcl::VoxelGrid<pcl::PointXYZ> VoxelFilter;
    VoxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);

    VoxelFilter.setInputCloud(cloudRefFilter);
    VoxelFilter.filter(*cloudRefFilter);

    VoxelFilter.setInputCloud(cloudMovFilter);
    VoxelFilter.filter(*cloudMovFilter);

    // Init Param
    Param param;
    param.Load("param.ini");

    // IRLS ICP
    clock_t time1=clock();

    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    transformation.translation() << 0.0, 0.0, 0.0;

    
    IRLSICP myIRLSICP(cloudRefFilter,cloudMovFilter,transformation,param);
    myIRLSICP.run();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAft(new pcl::PointCloud<pcl::PointXYZ>);
    myIRLSICP.getAftData(*cloudAft);

    clock_t time2=clock();
    cout << "IRLS_ICP Runtime: " <<(double)(time2 - time1) / CLOCKS_PER_SEC << "s" << endl;
    
    // Display
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDisplayMov(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDisplayRef(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDisplayAft(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int i=0;i<cloudMovFilter->points.size();++i)
    {
        pcl::PointXYZRGB pt;
        pt.x = cloudMovFilter->points.at(i).x;
        pt.y = cloudMovFilter->points.at(i).y;
        pt.z = cloudMovFilter->points.at(i).z;
        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
        cloudDisplayMov->points.push_back(pt);
    }
        
    for(int i=0;i<cloudRefFilter->points.size();++i)
    {
        pcl::PointXYZRGB pt;
        pt.x = cloudRefFilter->points.at(i).x;
        pt.y = cloudRefFilter->points.at(i).y;
        pt.z = cloudRefFilter->points.at(i).z;
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;
        cloudDisplayRef->points.push_back(pt);
    }

    for(int i=0;i<cloudAft->points.size();++i)
    {
        pcl::PointXYZRGB pt;
        pt.x = cloudAft->points.at(i).x;
        pt.y = cloudAft->points.at(i).y;
        pt.z = cloudAft->points.at(i).z;
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        cloudDisplayAft->points.push_back(pt);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addCoordinateSystem(1.0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudDisplayRef, "RefData");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudDisplayMov, "MovData");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudDisplayAft, "AftData");
    viewer->initCameraParameters();

    viewer->spin();

    return 0;
}