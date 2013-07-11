#ifndef AffineModel_HEADER
#define AffineModel_HEADER

#include "rspfGeoModel.h"


#pragma warning(push)
#pragma warning(disable : 4482)

class AffineModel:public rspfGeoModel
{
public:
	AffineModel();
	virtual int getNumberOfAdjustableParameters()const;
	virtual rspfPoint forward(const rspfPoint& warp_point)const;
	virtual rspfPoint inverse(const rspfPoint& base_point)const;
	virtual cv::Mat getTransform()const;
};

#endif
#pragma warning(pop)