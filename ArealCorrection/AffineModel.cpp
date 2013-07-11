#include "AffineModel.h"


#pragma warning(push)
#pragma warning(disable : 4482)


AffineModel::AffineModel()
{
	double m[6] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	m_parmlist = vector<double>(m, m + 6);
}

int AffineModel::getNumberOfAdjustableParameters()const
{
	return 6;
}
rspfPoint AffineModel::forward(const rspfPoint& warp_point)const
{
	double newX = m_parmlist[3 * 0 + 0] * warp_point.x + m_parmlist[3 * 0 + 1] * warp_point.y + m_parmlist[3 * 0 + 2];
	double newY = m_parmlist[3 * 1 + 0] * warp_point.x + m_parmlist[3 * 1 + 1] * warp_point.y + m_parmlist[3 * 1 + 2];
	return rspfPoint(newX, newY);
}
rspfPoint AffineModel::inverse(const rspfPoint& base_point)const
{
	double element_x = base_point.x;
	double element_y = base_point.y;
	double newX = m_parmlist[3 * 1 + 1] * element_x + m_parmlist[3 * 0 + 1] * element_y - m_parmlist[3 * 0 + 2] * m_parmlist[3 * 1 + 1] + m_parmlist[3 * 0 + 1] * m_parmlist[3 * 1 + 2];
	newX = newX / (m_parmlist[3 * 0 + 0] * m_parmlist[3 * 1 + 1] - m_parmlist[3 * 0 + 1] * m_parmlist[3 * 1 + 0]);

	double newY = m_parmlist[3 * 1 + 0] * element_x + m_parmlist[3 * 0 + 0] * element_y - m_parmlist[3 * 0 + 2] * m_parmlist[3 * 1 + 0] + m_parmlist[3 * 0 + 0] * m_parmlist[3 * 1 + 2];
	newY = newY / (m_parmlist[3 * 0 + 1] * m_parmlist[3 * 1 + 0] - m_parmlist[3 * 0 + 0] * m_parmlist[3 * 1 + 1]);
	return rspfPoint(newX, newY);
}
cv::Mat AffineModel::getTransform()const
{
	double m[6];
	for(int i = 0;i < 6;i++)
	{
		m[i] = m_parmlist[i];
	}
	cv::Mat TransMat(2, 3, CV_64F);
	for(int i = 0;i < 2;i++)
	{
		for(int j = 0;j < 3;j++)
		{
			TransMat.at<double>(i, j) = m_parmlist[i * 3 + j];
		}
	}
	return TransMat;
}
#pragma warning(pop)