#ifndef rspfFeature_HEADER
#define rspfFeature_HEADER
#include <iosfwd>
#include <string>
#include <vector>
#include <math.h>
#include "rspfConst.h"
using namespace std;
#pragma warning(push)
#pragma warning(disable : 4482)

class rspfPoint;
class rspfLine;
class rspfArea;
class rspfFeature;

class rspfPoint
{
public:
	double x;
	double y;
	string strId;
public:
	/*!
	* Constructors
	*/
	rspfPoint(){strId = "NULL";};
	rspfPoint(double ax, double ay){x = ax;y = ay;strId = "NULL";};
	rspfPoint(const rspfPoint& pt)
	{
		x = pt.x;
		y = pt.y;
		strId = pt.strId;
	};

	bool operator==(const rspfPoint& pt) const
   { return ( (x == pt.x) && (y == pt.y) ); } 
   bool operator!=(const rspfPoint& pt) const
   { return ( (x != pt.x) || (y != pt.y) ); }
   /*!
    * METHOD: length()
    * Returns the RSS of the components.
    */
   double length() const { return sqrt(x*x + y*y); }
   
   rspfPoint operator+(const rspfPoint& p) const
      { return rspfPoint(x+p.x, y+p.y); }
   rspfPoint operator-(const rspfPoint& p) const
      { return rspfPoint(x-p.x, y-p.y); }
   const rspfPoint& operator+=(const rspfPoint& p)
      { x += p.x; y += p.y; return *this; }
   const rspfPoint& operator-=(const rspfPoint& p)
      { x -= p.x; y -= p.y; return *this; }
   rspfPoint operator*(const double& d) const
      { return rspfPoint(d*x, d*y); }
   rspfPoint operator/(const double& d) const
      { return rspfPoint(x/d, y/d); }
   std::ostream& print(std::ostream& os, int precision=15) const;

   void makeNan(){x = rspf_nan; y = rspf_nan;}

   bool hasNans()const
   {
	   return (rspf_isnan(x) || rspf_isnan(y));
   }
   bool isNan()const
   {
	   return (rspf_isnan(x) && rspf_isnan(y));
   }
   
   friend std::ostream& operator<<(std::ostream& os,
                                                  const rspfPoint& pt);
};

// Line class
class rspfLine
{
public:
	vector<rspfPoint> m_Points;
	string strId;
public:
	/*!
	* Constructors
	*/
	rspfLine(){};
	rspfLine(const vector<rspfPoint>& Points);
};

// Area class
class rspfArea
{
public:
	vector<rspfPoint> m_Points;
	string strId;
public:
	/*!
	* Constructors
	*/
	rspfArea(){};
	rspfArea(const vector<rspfPoint>& Points);
	rspfPoint DistanceFromPoint(rspfPoint pt)	const;
	double xDistanceFromPoint(rspfPoint pt, bool upper)	const;
	double yDistanceFromPoint(rspfPoint pt, bool upper)	const;
	// 面到面的距离, index为area中到面的距离最大的顶点序号
	rspfPoint DistanceFromArea(const rspfArea &area, int *index = NULL) const;
	double xDistanceFromArea(const rspfArea &area, bool *upper, int *index = NULL) const;
	double yDistanceFromArea(const rspfArea &area, bool *upper, int *index = NULL) const;
	rspfPoint getPointDistanceDeriv_X(rspfPoint dpt, double  hdelta =1e-5) const;
	rspfPoint getPointDistanceDeriv_Y(rspfPoint dpt, double  hdelta =1e-5) const;
	void getPointDistanceDeriv(rspfPoint dpt, rspfPoint* partial_x, rspfPoint* partial_y, double hdelta =1e-6) const;
	void getPointXDistanceDeriv(rspfPoint dpt, bool upper, rspfPoint* partial, double hdelta =1e-6) const;
	void getPointYDistanceDeriv(rspfPoint dpt, bool upper, rspfPoint* partial, double hdelta =1e-6) const;
	rspfPoint Distance2Segment(rspfPoint pt, rspfPoint a, rspfPoint b, rspfPoint* intersection = NULL) const;
	double CrossMultiplication(rspfPoint pt, rspfPoint a, rspfPoint b)const;
	int X_Intersection(rspfPoint pt, rspfPoint a, rspfPoint b, rspfPoint* intersection = NULL)const;
private:
	bool isPointOnSegment(rspfPoint pt, rspfPoint a, rspfPoint b)	const;
	double Distance2Straightline(rspfPoint pt, rspfPoint a, rspfPoint b, rspfPoint* intersection = NULL)const;
};

class rspfFeature
{
public:
	
enum rspfFeatureType
	{
		rspfPointType,
		rspfLineType,
		rspfAreaType,
		rspfUnknown
	};
	rspfFeatureType m_featureType;
	vector<rspfPoint> m_Points;
	string strId;
   /**
    * Constructor.  The values are assumed to be in DEGREES.
    */
   rspfFeature();
   rspfFeature(const rspfFeature::rspfFeatureType& type);
   rspfFeature(const rspfPoint& Dpt);		//initialize as Gpt
   rspfFeature(const rspfLine& Line);	//initialize as Line
   rspfFeature(const rspfArea& Area);	//initialize as Area
   rspfFeature(const rspfFeature& src);
   const rspfFeature& operator = (const rspfFeature &aFeature);
   void makeNan(){m_featureType = rspfUnknown;m_Points.clear();}
   bool isNan()const
   {
	   return (m_Points.size() == 0);
   }
};
#pragma warning(pop)
#endif
