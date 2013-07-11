#include <iostream>
#include <iomanip>
#include <sstream>
#include "rspfFeature.h"
#include "rspfConst.h"
#pragma warning(push)
#pragma warning(disable : 4482)



//////////////////////////////////////////////////////////////////////////
////////////////////////////rspfDLine////////////////////////////////

//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
///////////////////////////////rspfArea////////////////////////////////
//////////////////////////////////////////////////////////////////////////
rspfArea::rspfArea(const vector<rspfPoint>& Points)
{
	m_Points = Points;

	// 保证闭合
	if(m_Points.size() > 1)
	{
		if(m_Points[0] != m_Points.back())
		{
			m_Points.push_back(m_Points[0]);
		}
	}
}

rspfPoint rspfArea::DistanceFromPoint(rspfPoint pt) const
{
	double dis = 1048576.0;
	rspfPoint disVector;
	int nLeftIntersection = 0;
	int nRightIntersection = 0;

	for(unsigned int i = 0;i < m_Points.size() - 1;i++)
	{
		rspfPoint intersection;
		rspfPoint tmpDisVector = Distance2Segment(pt, m_Points[i], m_Points[i+1]);
		double tmpDis_2 = tmpDisVector.x * tmpDisVector.x + tmpDisVector.y * tmpDisVector.y;
		//double tmpDis = Distance2Straightline(pt, m_Points[i], m_Points[i+1], &intersection);
		int intersection_state = X_Intersection(pt, m_Points[i], m_Points[i+1], &intersection);

		if(small_quantity > tmpDis_2)
		{
			// 点在边界上
			return rspfPoint(0.0, 0.0);
		}
		else
		{
			if(dis > tmpDis_2)
			{
				dis = tmpDis_2;
				disVector = tmpDisVector;
			}
		}
		if(intersection_state != 0 && isPointOnSegment(intersection, m_Points[i], m_Points[i+1]))
		{
			// 非水平且交点在线段上
			if(intersection_state < 0) nLeftIntersection++;
			else nRightIntersection++;			
		}
	}
	if((nLeftIntersection % 2 == 1) && (nRightIntersection % 2 == 1))
	{
		return rspfPoint(0.0, 0.0);
	}
	else
	{
		return disVector;
	}
}

bool rspfArea::isPointOnSegment(rspfPoint pt, rspfPoint a, rspfPoint b)	const
{
	// 线段左闭右开
	if(pt.isNan())
	{
		// 点无效
		return false;
	}
	rspfPoint dpt1 = a - b;
	rspfPoint dpt2 = pt - a;
	if(fabs(dpt1.x * dpt2.y - dpt1.y * dpt2.x) > small_quantity)
	{
		// 不在直线上
		return false;
	}
	else
	{
		return (pt.x >= min(a.x, b.x) && pt.x < max(a.x, b.x) && pt.y >= min(a.y, b.y) && pt.y <= max(a.y, b.y));
	}
}
rspfPoint rspfArea::DistanceFromArea(const rspfArea &area, int *index/* = NULL*/) const
{
	// area中个顶点到该面特征距离的最大值
	double dis = 0.0;
	rspfPoint disVector;
	int indexTmp = 0;
	for(unsigned int i = 0;i < area.m_Points.size();i++)
	{
		rspfPoint tmpDisVector = DistanceFromPoint(area.m_Points[i]);
		double tmpDis_2 = tmpDisVector.x * tmpDisVector.x + tmpDisVector.y * tmpDisVector.y;
		if(dis < tmpDis_2)
		{
			dis = tmpDis_2;
			disVector = tmpDisVector;
			indexTmp = i;
		}
	}
	if(index) *index = indexTmp;
	return disVector;
}

// 
int rspfArea::X_Intersection(rspfPoint pt, rspfPoint a, rspfPoint b, rspfPoint* intersection/* = NULL*/)const
{
	rspfPoint dpt1 = a - b;
	rspfPoint dpt2 = pt - a;
	if(fabs(dpt1.y) < small_quantity)
	{
		//如果水平
		intersection->makeNan();
		return 0;
	}
	else
	{
		intersection->y = pt.y;
		intersection->x = dpt1.x * dpt2.y / dpt1.y + a.x;
		double dis = intersection->x - pt.x;
		if(dis > 0) return 1;	//右相交
		else if(dis < 0) return -1;	//左相交
		else return 0;	//不相交
	}
}

double rspfArea::Distance2Straightline(rspfPoint pt, rspfPoint a, rspfPoint b, rspfPoint* intersection/* = NULL*/)const
{
	double dis = rspf_nan;
	rspfPoint dpt1 = a - b;
	rspfPoint dpt2 = pt - a;
	if(fabs(dpt1.x) < small_quantity && fabs(dpt1.y) < small_quantity)
	{
		// 如果两点相同
		intersection->makeNan();
		dis = sqrt((pt.x - a.x) * (pt.x - a.x) + (pt.y - a.y) * (pt.y - a.y));
		return dis;
	}
	// X方向
	if(fabs(dpt1.y) < small_quantity)
	{
		//如果水平
		intersection->makeNan();
	}
	else
	{
		intersection->y = pt.y;
		intersection->x = dpt1.x * dpt2.y / dpt1.y + a.x;
		dis = intersection->x - pt.x;
	}
	if(fabs(dpt1.x) >= small_quantity)
	{
		//如果不垂直
		if(rspf_isnan(dis))
		{
			//如果水平
			dis = dpt1.y * dpt2.x / dpt1.x - dpt2.y;
		}
		else
		{
			if(dis >= 0.0)
				dis = min(fabs(dis), fabs(dpt1.y * dpt2.x / dpt1.x - dpt2.y));
			else
				dis = -min(fabs(dis), fabs(dpt1.y * dpt2.x / dpt1.x - dpt2.y));
		}
	}
	return dis;
}

rspfPoint rspfArea::getPointDistanceDeriv_X(rspfPoint dpt, double  hdelta/* =1e-11*/) const
{   
	double den = 0.5/hdelta;
	rspfPoint res;
	res.x = DistanceFromPoint(rspfPoint(dpt.x + hdelta, dpt.y)).x;
	res.x -= DistanceFromPoint(rspfPoint(dpt.x - hdelta, dpt.y)).x;
	res.y = DistanceFromPoint(rspfPoint(dpt.x, dpt.y + hdelta)).x;
	res.y -= DistanceFromPoint(rspfPoint(dpt.x, dpt.y - hdelta)).x;
	res = res * den;
	return res;
}

rspfPoint rspfArea::getPointDistanceDeriv_Y(rspfPoint dpt, double  hdelta/* =1e-11*/) const
{   
	double den = 0.5/hdelta;
	rspfPoint res;
	res.x = DistanceFromPoint(rspfPoint(dpt.x + hdelta, dpt.y)).y;
	res.x -= DistanceFromPoint(rspfPoint(dpt.x - hdelta, dpt.y)).y;
	res.y = DistanceFromPoint(rspfPoint(dpt.x, dpt.y + hdelta)).y;
	res.y -= DistanceFromPoint(rspfPoint(dpt.x, dpt.y - hdelta)).y;
	res = res * den;
	return res;
}

void rspfArea::getPointDistanceDeriv(rspfPoint dpt, rspfPoint* partial_x, rspfPoint* partial_y, double hdelta/* =1e-6*/) const
{
	double den = 0.5/hdelta;
	rspfPoint disDpt1 = DistanceFromPoint(rspfPoint(dpt.x + hdelta, dpt.y));
	rspfPoint disDpt2 = DistanceFromPoint(rspfPoint(dpt.x - hdelta, dpt.y));
	partial_x->x = disDpt1.x - disDpt2.x;
	partial_y->x = disDpt1.y - disDpt2.y;
	disDpt1 = DistanceFromPoint(rspfPoint(dpt.x, dpt.y + hdelta));
	disDpt2 = DistanceFromPoint(rspfPoint(dpt.x, dpt.y - hdelta));
	partial_x->y = disDpt1.x - disDpt2.x;
	partial_y->y = disDpt1.y - disDpt2.y;
	*partial_x = *partial_x * den;
	*partial_y = *partial_y * den;
}
double rspfArea::CrossMultiplication(rspfPoint pt, rspfPoint a, rspfPoint b)const
{
	return (a.x-pt.x)*(b.y-pt.y)-(b.x-pt.x)*(a.y-pt.y);
}

rspfPoint rspfArea::Distance2Segment(rspfPoint pt, rspfPoint a, rspfPoint b, rspfPoint* intersection/* = NULL*/) const
{
	rspfPoint disVector;
	rspfPoint intersection_Tmp;
	double len0_2 = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
	double len0 = sqrt(len0_2);
	if(len0 < small_quantity)
	{
		// 如果两点相同
		if(intersection) *intersection = a;
		disVector.x = a.x - pt.x;
		disVector.y = a.y - pt.y;
		return disVector;
	}
	double d1_2 = (a.x - pt.x) * (a.x - pt.x) + (a.y - pt.y) * (a.y - pt.y);
	double d1 = sqrt(d1_2);
	double d2_2 = (b.x - pt.x) * (b.x - pt.x) + (b.y - pt.y) * (b.y - pt.y);
	double d2 = sqrt(d2_2);
	//面积法求距离
	double d0 = fabs(CrossMultiplication(pt, a, b) / len0);
	double d0_2 = d0 * d0;
	double len1_2 = d1_2 - d0_2;
	double len1 = sqrt(len1_2);
	double len2_2 = d2_2 - d0_2;
	double len2 = sqrt(len2_2);

	if (len1_2 > len0_2 || len2_2 > len0_2)
	{
		//垂足在线段外,取端点
		if (len1_2 > len2_2)
		{
			if(intersection) *intersection = b;
			disVector.x = b.x - pt.x;
			disVector.y = b.y - pt.y;
			return disVector;
		}
		else
		{
			if(intersection) *intersection = a;
			disVector.x = a.x - pt.x;
			disVector.y = a.y - pt.y;
			return disVector;
		}
	}
	//垂足在线段上
	intersection_Tmp.x = a.x + (b.x - a.x) * len1 / len0;
	intersection_Tmp.y = a.y + (b.y - a.y) * len1 / len0;
	if(intersection)
	{
		*intersection = intersection_Tmp;
	}
	disVector.x = intersection_Tmp.x - pt.x;
	disVector.y = intersection_Tmp.y - pt.y;
	return disVector;
}
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
/////////////////////////////rspfFeature///////////////////////////////
//////////////////////////////////////////////////////////////////////////
rspfFeature::rspfFeature()
{
	m_featureType = rspfUnknown;
}
rspfFeature::rspfFeature(const rspfFeature::rspfFeatureType& type)
{
	m_featureType = type;
}
rspfFeature::rspfFeature(const rspfPoint& pt)
{
	m_featureType = rspfFeatureType::rspfPointType;
	m_Points.clear();
	strId = pt.strId;
	m_Points.push_back(pt);
}

rspfFeature::rspfFeature(const rspfLine& Line)
{
	m_featureType = rspfFeatureType::rspfLineType;
	m_Points.clear();

	m_Points = Line.m_Points;
	strId = Line.strId;
}

rspfFeature::rspfFeature(const rspfArea& Area)
{
	m_featureType = rspfFeatureType::rspfAreaType;
	m_Points = Area.m_Points;
}

rspfFeature::rspfFeature(const rspfFeature& src) :
strId(src.strId),
m_Points(src.m_Points),
m_featureType(src.m_featureType)
{
}
const rspfFeature& rspfFeature::operator = (const rspfFeature &aFeature)
{
	strId = aFeature.strId;
	m_featureType = aFeature.m_featureType;
	m_Points = aFeature.m_Points;
	return *this;
}
//////////////////////////////////////////////////////////////////////////

#pragma warning(pop)