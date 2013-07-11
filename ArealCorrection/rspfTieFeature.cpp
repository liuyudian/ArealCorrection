#include <iostream>
#include <iomanip>
#include "rspfTieFeature.h"
#pragma warning(push)
#pragma warning(disable : 4482)

rspfTieFeature::rspfTieFeature(const rspfFeature& warpFeature, const rspfFeature& baseFeature, const double& aScore, const string& astrId/*=""*/)
: score(aScore),
strId(astrId)
{
	m_warpFeature = warpFeature;
	m_baseFeature = baseFeature;
	updateTieFeatureType();
}

rspfTieFeature::rspfTieFeature(const rspfTieFeature& tieFeature) :
score(tieFeature.score),
strId(tieFeature.strId)
{
	if (this != &tieFeature)
	{
		m_warpFeature = tieFeature.m_warpFeature;
		m_baseFeature = tieFeature.m_baseFeature;
		updateTieFeatureType();
	}
}

void rspfTieFeature::updateTieFeatureType()
{
	if(rspfFeature::rspfPointType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfPointType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTiePointPoint;
	}
	else if(rspfFeature::rspfPointType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfLineType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTiePointLine;
	}
	else if(rspfFeature::rspfPointType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfAreaType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTiePointArea;
	}
	else if(rspfFeature::rspfLineType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfPointType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTieLinePoint;
	}
	else if(rspfFeature::rspfLineType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfLineType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTieLineLine;
	}
	else if(rspfFeature::rspfLineType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfAreaType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTieLineArea;
	}
	else if(rspfFeature::rspfAreaType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfPointType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTieAreaPoint;
	}
	else if(rspfFeature::rspfAreaType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfLineType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTieAreaLine;
	}
	else if(rspfFeature::rspfAreaType == m_warpFeature.m_featureType 
		&& rspfFeature::rspfAreaType == m_baseFeature.m_featureType)
	{
		m_tieFeatureType = rspfTieAreaArea;
	}
	else
	{
		m_tieFeatureType = rspfTieUnknown;
	}
}

void rspfTieFeature::setBaseFeature(const rspfFeature& baseFeature)
{
	m_baseFeature = baseFeature;
	updateTieFeatureType();
}
const rspfFeature& rspfTieFeature::getBaseFeature()const
{
	return m_baseFeature;
}
rspfFeature& rspfTieFeature::refBaseFeature()
{
	return m_baseFeature;
}
void rspfTieFeature::setWarpFeature(const rspfFeature& warpFeature)
{
	m_warpFeature = warpFeature;
	updateTieFeatureType();
}
const rspfFeature& rspfTieFeature::getWarpFeature()const
{
	return m_warpFeature;
}
rspfFeature& rspfTieFeature::refWarpFeature()
{
	return m_warpFeature;
}
#pragma warning(pop)