#ifndef rspfGeoModel_HEADER
#define rspfGeoModel_HEADER
#include <opencv2/opencv.hpp>
#include "rspfTieFeature.h"
/*!****************************************************************************
 *
 * CLASS:  rspfSensorModel
 *
 *****************************************************************************/
class rspfGeoModel
{
public:
   enum CovMatStatus
   {
      COV_INVALID = 0,
      COV_PARTIAL = 1,
      COV_FULL    = 2
   };
   enum DeriveMode
   {
      OBS_INIT =-99,
      EVALUATE =-98,
      P_WRT_X = -1,
      P_WRT_Y = -2,
      P_WRT_Z = -3
   };

   /*!
    * CONSTRUCTORS:
    */
   rspfGeoModel(){};
   rspfGeoModel(const rspfGeoModel& copy_this);
   virtual ~rspfGeoModel(){};

   /*!
    * Implementation of base-class pure virtual projection methods. These
    * methods may be overriden by derived classes if those have more efficient
    * schemes. The implementations here are iterative (relatively slow). Both
    * depend on calls to the pure virtual lineSampleHeightToWorld() method.
    */
   virtual void warpToBase(const rspfPoint& warp_point,
								   rspfPoint&       base_point) const;
   virtual void  baseToWarp(const rspfPoint& base_point,
                                   rspfPoint&       warp_point) const;
 
   /*!
    * METHOD: print()
    * Fulfills base-class pure virtual. Dumps contents of object to ostream.
    */
   virtual std::ostream& print(std::ostream& out) const;
   /*!
    * This is from the adjustable parameter interface.  It is
    * called when a paraemter adjustment is made.
    */
   virtual void adjustableParametersChanged()
      {
         updateModel();
      }
   /*!
    * VIRTUAL METHOD: updateModel()
    * Following a change to the adjustable parameter set, this virtual
    * is called to permit instances to compute derived quantities after
    * parameter change.
    */
   virtual void updateModel() {}
   /*!
    * METHODS:  saveState, loadState
    * Fulfills rspfObject base-class pure virtuals. Loads and saves geometry
    * KWL files.
    */
  // virtual bool saveState(rspfKeywordlist& kwl,
  //                        const char* prefix=0) const;
  // virtual bool loadState(const rspfKeywordlist& kwl,
  //                        const char* prefix=0);
   /*!
    * OPERATORS:
    */
   /*!
    * optimizableProjection implementation
    */
	virtual int degreesOfFreedom()const;
	virtual double optimizeFit(const vector< rspfTieFeature >& tieFeatureList);

	// 2010.1.14 loong
	// build a error equation according to a gcp
	// used for block adjustment
	void buildErrorEquation ( const rspfTieFeature& tieFeature, int nType, cv::Mat &A,
							cv::Mat &B, cv::Mat &L, double pstep_scale);
   // 2010.1.14 loong
   // compute the image derivatives regarding the ground coordinate of the tie point
   //rspfDpt getCoordinateForwardDeriv(int parmIdx , const rspfGpt& gpos, double hdelta=1e-11);

   /*!
    * METHOD: getForwardDeriv()
    * gives forward() partial derivative regarding parameter parmIdx (>=0)
    * default implementation is centered finite difference
    * -should be reimplemented with formal derivative in child class
    */
   virtual rspfPoint getForwardDeriv(int parmIdx, const rspfPoint& pt, double hdelta=1e-11);
   /*!
    * METHOD: getInverseDeriv()
    * gives inverse() partial derivative regarding parameter parmIdx (>=0)
    * default implementation is centered finite difference
    * -should be reimplemented with formal derivative in child class
    */
   virtual rspfPoint getInverseDeriv(int parmIdx, const rspfPoint& pt,double hdelta);


   static void funcErrorEquation(double *param, double *hx, int m, int n, void *adata);
   static void jacErrorEquation(double *param, double *j, int m, int n, void *adata);
   struct optimizeStruct 
   {
		vector< rspfTieFeature > tieFeatureList;
		rspfGeoModel* pThis;
   };

public:
	virtual int getNumberOfAdjustableParameters()const = NULL;
	virtual rspfPoint forward(const rspfPoint& warp_point)const = NULL;
	virtual rspfPoint inverse(const rspfPoint& base_point)const = NULL;
	virtual cv::Mat getTransform()const = NULL;
	double getAdjustableParameter(int idx);
	void getAdjustment(std::vector<double>& adj);
	void setAdjustableParameter(int idx, double value, bool notify);
	int getNumofObservations(const vector< rspfTieFeature >& tieFeatureList);
	/*
    * METHOD: getResidue()
    * returns ground or image residue
    */
   cv::Mat getResidue(const vector< rspfTieFeature >& tieFeatureList);

   void buildNormalEquation(const vector< rspfTieFeature >& tieLineList,
	   cv::Mat& A,
	   cv::Mat& residue,
	   cv::Mat& projResidue,
	   double pstep_scale);

   cv::Mat solveLeastSquares(cv::Mat& A,  cv::Mat& r)const;
   /*!
    * stable invert stolen from rspfRpcSolver
    */
   cv::Mat invert(const cv::Mat& m)const;

   double SumSquare(const cv::Mat& m)const;
   double NormInfinity(const cv::Mat& m)const;

protected:
	std::vector<double> m_parmlist;
};
#endif
