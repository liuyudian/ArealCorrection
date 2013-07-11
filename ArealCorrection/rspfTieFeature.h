#ifndef rspfTieFeature_HEADER
#define rspfTieFeature_HEADER
#include <iostream>
#include "rspfFeature.h"
#pragma warning(push)
#pragma warning(disable : 4482)
/**
 * storage class for tie point between ground and image
 * based on rspfGpt
 * + GML feature (OGC) serialization
 *
 * NOTES
 * accuracy is not stored here (need to derive object if need a per-point accuracy) 
 * GML storage is WGS84 only, it stores above ellipsoid height (m)
 *
 * TODO : 
 * -support other datum (easy) / ground projection (big change in RSPF)
 * -unify with rspfTDpt
 */
class rspfTieFeature
{
public:
	enum rspfTieFeatureType
	{
		rspfTiePointPoint,
		rspfTieLinePoint,
		rspfTieAreaPoint,
		rspfTiePointLine,
		rspfTieLineLine,
		rspfTieAreaLine,
		rspfTiePointArea,
		rspfTieLineArea,
		rspfTieAreaArea,
		rspfTieUnknown
	};
	rspfFeature m_warpFeature;
	rspfFeature m_baseFeature;
	std::string strId;
	double score;
	rspfTieFeature() : 
		  score(0.0),
	  strId("")
      {}
	rspfTieFeature(const rspfFeature& warpFeature, const rspfFeature& baseFeature, const double& aScore, const std::string& astrId="NULL"); 
         
	rspfTieFeature(const rspfTieFeature& tieFeature); 
   ~rspfTieFeature() {}
	inline const rspfTieFeature& operator=(const rspfTieFeature&);
	void					setWarpFeature(const rspfFeature& warpFeature);
	const rspfFeature& getWarpFeature()const;
	rspfFeature& refWarpFeature();
	void					setBaseFeature(const rspfFeature& baseFeature);
	const rspfFeature& getBaseFeature()const;
	rspfFeature& refBaseFeature();
	void            setId(const std::string& s) { strId=s; }
	void            setScore(const double& s) { score=s; }
	const double& getScore()const             { return score; }
	const std::string& getId()const             { return strId; }
	double& refScore()                  { return score; }
	void makeNan() 
	{
		m_warpFeature.makeNan();
		m_baseFeature.makeNan();
		score = rspf_nan;
	}
   
	bool hasNans()const
	{
		return (m_warpFeature.isNan() || m_baseFeature.isNan() || (rspf_isnan(score)));
	}
   
	bool isNan()const
	{
		return (m_warpFeature.isNan() && m_baseFeature.isNan() && (rspf_isnan(score)));
	}
	rspfTieFeatureType getTieFeatureType() const{return m_tieFeatureType;};
	void updateTieFeatureType();
protected:
	rspfTieFeatureType m_tieFeatureType;
};
inline const rspfTieFeature& rspfTieFeature::operator=(const rspfTieFeature& feature)
{
   if (this != &feature)
   {
	  m_warpFeature = feature.m_warpFeature;
	  m_baseFeature = feature.m_baseFeature;
      score = feature.score;
	  strId = feature.strId;
	  updateTieFeatureType();
   }
   return *this;
}
#pragma warning(pop)
#endif /* #ifndef rspfTieFeature_HEADER */
