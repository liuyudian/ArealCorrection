/***********************************************************************
 * OpenCV 2.3.1 测试例程
 * 于仕琪 提供
 ***********************************************************************/
#include <opencv2/opencv.hpp>
#include "rspfGeoModel.h"
#include "AffineModel.h"
#include "func.h"
 
using namespace std;
using namespace cv;

void test1()
{
	//const char* imagename = "airplane.jpg";
	//const char* outname = "airplane1.jpg";
	//string workfold = "D:\\Loong\\Programs\\CV\\testdata";
	string workfold = "E:\\Programs\\CV\\testdata";
	string imagename = workfold + "\\polygons.jpg";
	string outname = workfold + "\\polygons_de.jpg";
 
	//从文件中读入图像
	Mat img = imread(imagename.c_str());
	int iRows = img.rows;
	int iCols = img.cols;
	cv::Size sz(iRows, iCols);
	Mat newImg;
	Point2f src_center(img.cols/2.0F, img.rows/2.0F);
	Point2f src_point[4] = {Point2f(0.0F, 0.0F), Point2f(float(iRows), 0.0F), Point2f(0.0F, float(iCols)), Point2f(float(iRows), float(iCols))};
	Point2f dst_point[4] = {Point2f(0.0F, 0.0F), Point2f(0.7F * iRows, 0.2F * iCols), 
		Point2f(0.3F * iRows, 1.2F * iCols), Point2f(1.3F * iRows, 1.2F * iCols)};
	Mat affineTrans = getAffineTransform(src_point, dst_point);
	
	
	float m[6];
	Mat tmpMat(4, 2, CV_32F, m);
	affineTrans.convertTo(tmpMat, tmpMat.type());
	for(int i = 0;i < 2;i++)
	{
		m[i * 3 + 0] = tmpMat.at<float>(i, 0);
		m[i * 3 + 1] = tmpMat.at<float>(i, 1);
		m[i * 3 + 2] = tmpMat.at<float>(i, 2);
	}
	
	
	cout<<"affineTrans:\n"<<affineTrans<<endl;
	//float m[6] = {0.501036f, 0.865426f, -8.428640f, -0.865426f, 0.501036f, 31.380967f};
	//Mat affineTrans = Mat(2, 3, CV_32F, m);
	//cout<<M<<endl;


	/*
	float pos[12] = {0.0f, 0.0f, 1.0f,
	0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
	pos[4] = float(iCols);
	pos[6] = float(iRows);
	pos[9] = float(iRows);
	pos[10] = float(iCols);
	Mat matPos(4, 3, CV_32FC1, pos);

	Mat newPos = matPos * M.t();

	float element_x = newPos.at<float>(0, 0);
	float element_y = newPos.at<float>(0, 1);
	float minX = element_x;
	float maxX = element_x;
	float minY = element_y;
	float maxY = element_y;
	for(int i = 0;i < 3;i++)
	{
		float element_x = newPos.at<float>(i, 0);
		float element_y = newPos.at<float>(i, 1);
		if(element_x < minX)
		{
			minX = element_x;
		}
		if(element_x > minX)
		{
			maxX = element_x;
		}
		if(element_y < minY)
		{
			minY = element_y;
		}
		if(element_y > minY)
		{
			maxY = element_y;
		}
	}
*/
	float pos[8] = {0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	pos[3] = float(iRows);
	pos[4] = float(iCols);
	pos[6] = float(iCols);
	pos[7] = float(iRows);
	Mat matPos(4, 2, CV_32F, pos);
	//Mat newPos(4, 2, CV_32FC1);
	float newPos[8];
	for(int i = 0;i < 4;i++)
	{
		float element_x = matPos.at<float>(i, 0);
		float element_y = matPos.at<float>(i, 1);
		//float newX = m[3 * 1 + 1] * element_x + m[3 * 0 + 1] * element_y - m[3 * 0 + 2] * m[3 * 1 + 1] + m[3 * 0 + 1] * m[3 * 1 + 2];
		//newX = newX / (m[3 * 0 + 0] * m[3 * 1 + 1] - m[3 * 0 + 1] * m[3 * 1 + 0]);

		//float newY = m[3 * 1 + 0] * element_x + m[3 * 0 + 0] * element_y - m[3 * 0 + 2] * m[3 * 1 + 0] + m[3 * 0 + 0] * m[3 * 1 + 2];
		//newY = newY / (m[3 * 0 + 1] * m[3 * 1 + 0] - m[3 * 0 + 0] * m[3 * 1 + 1]);

		float newX = m[3 * 0 + 0] * element_x + m[3 * 0 + 1] * element_y + m[3 * 0 + 2];
		float newY = m[3 * 1 + 0] * element_x + m[3 * 1 + 1] * element_y + m[3 * 1 + 2];

		newPos[i * 2 + 0] = newX;
		newPos[i * 2 + 1] = newY;
	}
	//Mat m1 = matPos * M;
	//Mat m2 = (M.t() * M).inv();
	//cout<<"m1="<<endl<<m1<<endl;
	//cout<<"m2="<<endl<<m2<<endl;
	//Mat newPos = matPos * M * (M.t() * M).inv();
	//cout<<"newPos="<<endl<<newPos<<endl;
	//float element_x = newPos.at<float>(0, 0);
	//float element_y = newPos.at<float>(0, 1);
	float element_x = newPos[0];
	float element_y = newPos[1];
	float minX = element_x;
	float maxX = element_x;
	float minY = element_y;
	float maxY = element_y;
	for(int i = 1;i < 4;i++)
	{
		//float element_x = newPos.at<float>(i, 0);
		//float element_y = newPos.at<float>(i, 1);
		float element_x = newPos[i * 2 + 0];
		float element_y = newPos[i * 2 + 1];
		//float element_x = CV_MAT_ELEM(newPos,float,i + 1, 0);
		//float element_y = CV_MAT_ELEM(newPos,float,i + 1, 1);
		//float element_x = newPos.data[3 * i + 0];
		//float element_y = newPos.data[3 * i + 1];
		//float element_x = CV_MAT_ELEM(newPos, float, i + 1, 0);
		//float element_y = CV_MAT_ELEM(newPos, float, i + 1, 1);
		if(element_x < minX)
		{
			minX = element_x;
		}
		if(element_x > minX)
		{
			maxX = element_x;
		}
		if(element_y < minY)
		{
			minY = element_y;
		}
		if(element_y > minY)
		{
			maxY = element_y;
		}
	}

	int newCols = int(max(fabs(maxX), fabs(minX)) + 0.5);
	int newRows = int(max(fabs(maxY), fabs(minY)) + 0.5);
	//int newCols = int(maxX - minX + 0.5);
	//int newRows = int(maxY - minY + 0.5);
	cv::Size newSize(newCols, newRows);
	
	warpAffine(img, newImg, affineTrans , newSize, INTER_CUBIC);

 
	//如果读入图像失败
	if(img.empty())
	{
		fprintf(stderr, "Can not load image %s\n", imagename);
		return ;
	}
 
	//显示图像
	imshow("image", newImg);

	imwrite(outname.c_str(), newImg);
 
	//此函数等待按键，按键盘任意键就返回
	waitKey();
}


void geoCorrection()
{
	string workfold = "D:\\Loong\\Programs\\CV\\testdata";
	//string workfold = "E:\\Programs\\CV\\testdata";
	string warpFeatureFile = workfold + "\\warp_feature.txt";
	string baseFeatureFile = workfold + "\\base_feature.txt";
	string reportfile = workfold + "\\report.txt";

	string inAreaShp = workfold + "\\inArea.shp";
	string outAreaShp = workfold + "\\outArea.shp";	
	string inPointShp = workfold + "\\inPoint.shp";
	string outPointShp = workfold + "\\outPoint.shp";
	string imagename = workfold + "\\polygons_de.jpg";
	string resultname = workfold + "\\result.jpg";

	vector<rspfFeature> warpFeatureList, baseFeatureList;
	//ReadFeatures1(warpFeatureFile, warpFeatureList, rspfFeature::rspfPointType);
	//ReadFeatures1(baseFeatureFile, baseFeatureList, rspfFeature::rspfPointType);
	//ReadFeatures1(warpFeatureFile, warpFeatureList);
	//ReadFeatures1(baseFeatureFile, baseFeatureList);
	ReadFeatures1(warpFeatureFile, warpFeatureList, rspfFeature::rspfAreaType);
	ReadFeatures1(baseFeatureFile, baseFeatureList, rspfFeature::rspfAreaType);
	//根据序号连接对应的特征
	vector<rspfTieFeature> tieFeatureList;
	AppendTieFeatures(tieFeatureList, warpFeatureList, baseFeatureList);

	AffineModel *affineModel = new AffineModel();

	affineModel->optimizeFit(tieFeatureList);

	warpFeatureList.clear();
	baseFeatureList.clear();
	//ReadFeatures1(warpFeatureFile, warpFeatureList, rspfFeature::rspfAreaType);
	//ReadFeatures1(baseFeatureFile, baseFeatureList, rspfFeature::rspfAreaType);
	ReadFeatures1(warpFeatureFile, warpFeatureList, rspfFeature::rspfPointType);
	ReadFeatures1(baseFeatureFile, baseFeatureList, rspfFeature::rspfPointType);
	AppendTieFeatures(tieFeatureList, warpFeatureList, baseFeatureList);
	OutputReport(reportfile, affineModel, tieFeatureList);

	SaveFeaturetoShape(inAreaShp, tieFeatureList, rspfFeature::rspfAreaType);
	SaveFeaturetoShape(inPointShp, tieFeatureList, rspfFeature::rspfPointType);
	UpdateFeatures(affineModel, tieFeatureList);
	SaveFeaturetoShape(outAreaShp, tieFeatureList, rspfFeature::rspfAreaType);
	SaveFeaturetoShape(outPointShp, tieFeatureList, rspfFeature::rspfPointType);

	cv::Mat affineTrans = affineModel->getTransform();

	Mat img = imread(imagename);
	int iRows = img.rows;
	int iCols = img.cols;

	cv::Size newSize = getNewSize(iRows, iCols, affineTrans);

	Mat newImg;
	warpAffine(img, newImg, affineTrans , newSize, INTER_CUBIC);

	//显示图像
	imshow("image", newImg);

	imwrite(resultname, newImg);

	delete affineModel;
	
	//此函数等待按键，按键盘任意键就返回
	waitKey();
}
 
int main(int argc, char* argv[])
{
	//test1();
	geoCorrection();
 
	return 0;
}