#ifndef FUNC_HEADER
#define FUNC_HEADER
#include "rspfGeoModel.h"
#include <string>
#include <vector>
using namespace std;

void ReadFeatures(string strFilename, vector < rspfFeature > & dptAreaList);
bool ReadFeatures1(string strFilename, vector < rspfFeature > & dptAreaList, rspfFeature::rspfFeatureType featureType = rspfFeature::rspfUnknown);
bool AppendTieFeatures(vector<rspfTieFeature>& tieFeatureList, const vector<rspfFeature> &warpFeatureList, const vector<rspfFeature> &baseFeatureList);
bool OutputReport(string reportfile, rspfGeoModel* geoModel, const vector < rspfTieFeature > & featureList);
cv::Size getNewSize(int iRows, int iCols, cv::Mat transMat);
bool UpdateFeatures(rspfGeoModel* geoModel, vector<rspfTieFeature>& featureList);
bool SaveFeaturetoShape(string filenametosave,vector<rspfTieFeature> tiefeatureList, rspfFeature::rspfFeatureType featureType);

string SBeforeLast(const string& str, char ch);
string SAfterFirst(const string& str, char ch);
string SAfterLast(const string& str, char ch);
string SBeforeFirst(const string& str, char ch);
static int SplitString(const string& input, const string& delimiter, vector<string>& results, bool includeEmpties = true);
#endif