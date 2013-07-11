#include "func.h"
#include <iostream>
#include <fstream>

#include "gdal_priv.h"
#include "gdal.h"
#include "ogr_srs_api.h"
#include "cpl_string.h"
#include "cpl_conv.h"
#include "cpl_multiproc.h"
#include "ogrsf_frmts.h"

#pragma warning(push)
#pragma warning(disable : 4482)

void ReadFeatures(string strFilename, vector < rspfFeature > & featureList)
{
	char strtmp[1024];
	std::vector<string> strList;
	string str;

	ifstream os(strFilename.c_str());
	
	while(os.getline(strtmp,1024))
	{
		str = strtmp;
		if(str.empty()) continue;
		strList.clear();
		SplitString(str, "	", strList, false);
		if (strList.size()==1) {
			str = strList[0];
			strList.clear();
			SplitString(str, " ", strList, false);
		}
		if (strList.size() < 4) continue;

		if(0 == strList[1].compare("point"))
		{
			rspfFeature tmpPoint(rspfFeature::rspfPointType);
			tmpPoint.strId = strList[0];

			int pos = 2;
			int num = atoi(strList[pos++].c_str());
			if((int)strList.size() < num * 2 + pos)
			{
				continue;
			}
			for(int i = 0;i < num;i++)
			{
				rspfPoint pt;
				pt.x = atof(strList[pos++].c_str());
				pt.y = atof(strList[pos++].c_str());
				tmpPoint.m_Points.push_back(pt);
			}
			featureList.push_back(tmpPoint);
		}
		else if(0 == strList[1].compare("line"))
		{
			rspfFeature tmpLine(rspfFeature::rspfLineType);
			tmpLine.strId = strList[0];

			int pos = 2;
			int num = atoi(strList[pos++].c_str());
			if((int)strList.size() < num * 2 + pos)
			{
				continue;
			}
			for(int i = 0;i < num;i++)
			{
				rspfPoint pt;
				pt.x = atof(strList[pos++].c_str());
				pt.y = atof(strList[pos++].c_str());
				tmpLine.m_Points.push_back(pt);
			}
			featureList.push_back(tmpLine);
		}
		else if(0 == strList[1].compare("area"))
		{
			rspfFeature tmpArea(rspfFeature::rspfAreaType);
			tmpArea.strId = strList[0];

			int pos = 2;
			int num = atoi(strList[pos++].c_str());
			if((int)strList.size() < num * 2 + pos)
			{
				continue;
			}
			for(int i = 0;i < num;i++)
			{
				rspfPoint pt;
				pt.x = atof(strList[pos++].c_str());
				pt.y = atof(strList[pos++].c_str());
				tmpArea.m_Points.push_back(pt);
			}
			featureList.push_back(tmpArea);
		}
	}
}

bool ReadFeatures1(string strFilename, vector < rspfFeature > & featureList, rspfFeature::rspfFeatureType featureType/* = rspfFeature::rspfUnknown*/)
{
	char strtmp[1024];
	std::vector<string> strList;
	string str;

	ifstream os(strFilename.c_str());

	while(os.getline(strtmp,1024))
	{
		str = strtmp;
		if(str.empty()) continue;
		strList.clear();
		SplitString(str, "	", strList, false);
		if (strList.size()==1) {
			str = strList[0];
			strList.clear();
			SplitString(str, " ", strList, false);
		}
		if (strList.size() != 3) continue;

		rspfFeature tmpFeature;
		tmpFeature.strId = strList[0];
		int num = atoi(strList[2].c_str());

		if(0 == strList[1].compare("point"))
		{
			if(rspfFeature::rspfUnknown != featureType && rspfFeature::rspfPointType != featureType ) continue;
			tmpFeature.m_featureType = rspfFeature::rspfPointType;
		}
		else if(0 == strList[1].compare("line"))
		{
			if(rspfFeature::rspfUnknown != featureType && rspfFeature::rspfLineType != featureType ) continue;
			tmpFeature.m_featureType = rspfFeature::rspfLineType;
		}
		else if(0 == strList[1].compare("area"))
		{
			if(rspfFeature::rspfUnknown != featureType && rspfFeature::rspfAreaType != featureType ) continue;
			tmpFeature.m_featureType = rspfFeature::rspfAreaType;
		}
		else continue;

		for(int i = 0;i < num;i++)
		{
			if(!os.getline(strtmp, 1024)) return false;
			string strPoint = strtmp;
			if(strPoint.empty()) continue;
			vector<string> strCorList;
			SplitString(strPoint, "	", strCorList, false);
			if(strCorList.size()==1){
				strPoint = strCorList[0];
				strCorList.clear();
				SplitString(strPoint, "	", strCorList, false);
			}
			if(strCorList.size() != 2) continue;
			rspfPoint pt(atof(strCorList[0].c_str()), atof(strCorList[1].c_str()));
			tmpFeature.m_Points.push_back(pt);
		}
		featureList.push_back(tmpFeature);
	}
	return true;
}

int SplitString(const string& input, 
							 const string& delimiter, vector<string>& results, 
							 bool includeEmpties)
{
	int iPos = 0;
	int newPos = -1;
	int sizeS2 = (int)delimiter.size();
	int isize = (int)input.size();

	if( 
		( isize == 0 )
		||
		( sizeS2 == 0 )
		)
	{
		return 0;
	}

	vector<int> positions;

	newPos = input.find (delimiter, 0);

	if( newPos < 0 )
	{ 
		return 0; 
	}

	int numFound = 0;

	while( newPos >= iPos )
	{
		numFound++;
		positions.push_back(newPos);
		iPos = newPos;
		newPos = input.find (delimiter, iPos+sizeS2);
	}

	if( numFound == 0 )
	{
		return 0;
	}

	for( int i=0; i <= (int)positions.size(); ++i )
	{
		string s("");
		if( i == 0 ) 
		{ 
			s = input.substr( i, positions[i] ); 
			if( includeEmpties || ( s.size() > 0 ) )
			{
				results.push_back(s);
			}
			continue;
		}
		int offset = positions[i-1] + sizeS2;
		if( offset < isize )
		{
			if( i == positions.size() )
			{
				s = input.substr(offset);
			}
			else if( i > 0 )
			{
				s = input.substr( positions[i-1] + sizeS2, 
					positions[i] - positions[i-1] - sizeS2 );
			}
		}
		if( includeEmpties || ( s.size() > 0 ) )
		{
			results.push_back(s);
		}
	}
	return numFound;
}

bool AppendTieFeatures(vector<rspfTieFeature>& tieFeatureList, const vector<rspfFeature> &warpFeatureList, const vector<rspfFeature> &baseFeatureList)
{
	for(unsigned int i = 0;i < warpFeatureList.size();i++)
	{
		if("NULL" == warpFeatureList[i].strId) continue;
		for(unsigned int j = 0;j <baseFeatureList.size();j++)
		{
			if(warpFeatureList[i].strId == baseFeatureList[j].strId)
			{
				rspfTieFeature tieFeature;
				tieFeature.setWarpFeature(warpFeatureList[i]);
				tieFeature.setBaseFeature(baseFeatureList[j]);
				tieFeature.setId(warpFeatureList[i].strId);
				tieFeatureList.push_back(tieFeature);
				//gptFeatureList.erase(gptFeatureList.begin() + j);
			}
		}
	}
	return true;
}

cv::Size getNewSize(int iRows, int iCols, cv::Mat transMat)
{
	double pos[8] = {0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	pos[3] = double(iRows);
	pos[4] = double(iCols);
	pos[6] = double(iCols);
	pos[7] = double(iRows);


	double *m = new double [transMat.rows * transMat.cols];
	for(int i = 0;i < transMat.rows;i++)
	{
		for(int j = 0;j < transMat.cols;j++)
		{
			m[i * transMat.cols + j] = transMat.at<double>(i, j);
		}
	}

	double newPos[8];
	for(int i = 0;i < 4;i++)
	{
		double element_x = pos[i * 2 + 0];
		double element_y = pos[i * 2 + 1];
		//float newX = m[3 * 1 + 1] * element_x + m[3 * 0 + 1] * element_y - m[3 * 0 + 2] * m[3 * 1 + 1] + m[3 * 0 + 1] * m[3 * 1 + 2];
		//newX = newX / (m[3 * 0 + 0] * m[3 * 1 + 1] - m[3 * 0 + 1] * m[3 * 1 + 0]);

		//float newY = m[3 * 1 + 0] * element_x + m[3 * 0 + 0] * element_y - m[3 * 0 + 2] * m[3 * 1 + 0] + m[3 * 0 + 0] * m[3 * 1 + 2];
		//newY = newY / (m[3 * 0 + 1] * m[3 * 1 + 0] - m[3 * 0 + 0] * m[3 * 1 + 1]);

		double newX = m[3 * 0 + 0] * element_x + m[3 * 0 + 1] * element_y + m[3 * 0 + 2];
		double newY = m[3 * 1 + 0] * element_x + m[3 * 1 + 1] * element_y + m[3 * 1 + 2];

		newPos[i * 2 + 0] = newX;
		newPos[i * 2 + 1] = newY;
	}
	double element_x = newPos[0];
	double element_y = newPos[1];
	double minX = element_x;
	double maxX = element_x;
	double minY = element_y;
	double maxY = element_y;
	for(int i = 1;i < 4;i++)
	{
		double element_x = newPos[i * 2 + 0];
		double element_y = newPos[i * 2 + 1];
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

	int newCols = int(fabs(maxX) + 0.5);
	int newRows = int(fabs(maxY) + 0.5);
	//int newRows = int(max(fabs(maxX), fabs(minX)) + 0.5);
	//int newCols = int(max(fabs(maxY), fabs(minY)) + 0.5);
	delete []m;
	cv::Size newSize(newCols, newRows);
	return newSize;
}

bool OutputReport(string reportfile, rspfGeoModel* geoModel, const vector < rspfTieFeature > & featureList)
{
	fstream fs;
	fs.open(reportfile.c_str(), ios_base::out);
	fs.setf(ios::fixed, ios::floatfield);
	fs.precision(2);

	fs<<"残差报告\n\n"<<"残差单位：像素\n";

	cv::Mat residue = geoModel->getResidue(featureList);


	for(unsigned int i = 0;i < featureList.size();i++)
	{
		double residueX = residue.at<double>(i * 2 + 0, 0);
		double residueY = residue.at<double>(i * 2 + 1, 0);
		if(rspfTieFeature::rspfTiePointPoint == featureList[i].getTieFeatureType())
		{
			fs<<featureList[i].strId<<"\t"<<"point\t";
		}
		else if(rspfTieFeature::rspfTieLineLine == featureList[i].getTieFeatureType())
		{
			fs<<featureList[i].strId<<"\t"<<"line\t";
		}
		else if(rspfTieFeature::rspfTieAreaArea == featureList[i].getTieFeatureType())
		{
			fs<<featureList[i].strId<<"\t"<<"area\t";
		}
		else
		{
			fs<<featureList[i].strId<<"\t"<<"unknown\t";
		}
		fs<<sqrt(residueX*residueX + residueY*residueY)<<"\t"
			<<residueX<<"\t"
			<<residueY<<"\n";
	}

	fs.close();

	return true;
}

bool SaveFeaturetoShape(string filenametosave,vector<rspfTieFeature> tiefeatureList, rspfFeature::rspfFeatureType featureType/* = rspfGptFeature::rspfGptFeatureType::rspfGptAreaType*/)
{
	GDALAllRegister();
	OGRRegisterAll();//注册所有的文件格式驱动
	const char *pszDriverName = "ESRI Shapefile";
	OGRSFDriver *poDriver;
	poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(
		pszDriverName );
	if( poDriver == NULL )
	{
		return false;//创建文件驱动失败
	}

	poDriver->DeleteDataSource(filenametosave.c_str());
	OGRDataSource *poDS;
	poDS = poDriver->CreateDataSource( filenametosave.c_str(), NULL );
	if( poDS == NULL )
	{
		return false;//创建源失败
	}

	string layerName = SBeforeLast(SAfterLast(filenametosave, '\\'), '.');

	if(rspfFeature::rspfFeatureType::rspfLineType == featureType)
	{
		// 如果是直线
		OGRLayer *poLayer;
		poLayer = poDS->CreateLayer( layerName.c_str(), NULL, wkbLineString, NULL );
		if( poLayer == NULL )
		{
			return false;//创建图层对象失败
		} 

		OGRFieldDefn oField( "id", OFTString );
		oField.SetWidth(32);
		if( poLayer->CreateField( &oField ) != OGRERR_NONE )
		{
			return false;
		}
		for(unsigned int i = 0;i < tiefeatureList.size();i++)
		{
			if(rspfFeature::rspfFeatureType::rspfLineType == tiefeatureList[i].getBaseFeature().m_featureType)
			{
				OGRFeature *poFeature;
				poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
				poFeature->SetField( "id", tiefeatureList[i].strId.c_str() );

				OGRLineString lineString;
				int nPt = tiefeatureList[i].getBaseFeature().m_Points.size();
				lineString.setNumPoints(nPt);
				for(int j = 0;j < nPt;j++)
				{
					lineString.setPoint(j, tiefeatureList[i].getBaseFeature().m_Points[j].x, tiefeatureList[i].getBaseFeature().m_Points[j].y);
				}

				poFeature->SetGeometry( &lineString ); 

				if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
				{
					return false;
				}

				OGRFeature::DestroyFeature( poFeature );
			}
		}
	}
	else if(rspfFeature::rspfFeatureType::rspfPointType == featureType)
	{
		//如果是点
		OGRLayer *poLayer;
		poLayer = poDS->CreateLayer( layerName.c_str(), NULL, wkbPoint, NULL );
		if( poLayer == NULL )
		{
			return false;//创建图层对象失败
		} 

		OGRFieldDefn oField( "id", OFTString );
		oField.SetWidth(32);
		if( poLayer->CreateField( &oField ) != OGRERR_NONE )
		{
			return false;
		}
		for(unsigned int i = 0;i < tiefeatureList.size();i++)
		{
			if(rspfFeature::rspfFeatureType::rspfPointType == tiefeatureList[i].getBaseFeature().m_featureType)
			{
				OGRFeature *poFeature;
				poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
				poFeature->SetField( "id", tiefeatureList[i].strId.c_str() );

				OGRPoint pt;
				pt.setX(tiefeatureList[i].getBaseFeature().m_Points[0].x);
				pt.setY(tiefeatureList[i].getBaseFeature().m_Points[0].y);

				poFeature->SetGeometry( &pt ); 

				if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
				{
					return false;
				}
				OGRFeature::DestroyFeature( poFeature );
			}
		}
	}
	else if(rspfFeature::rspfFeatureType::rspfAreaType == featureType)
	{
		// 如果是面
		OGRLayer *poLayer;
		poLayer = poDS->CreateLayer( layerName.c_str(), NULL, wkbPolygon, NULL );
		if( poLayer == NULL )
		{
			return false;//创建图层对象失败
		} 

		OGRFieldDefn oField( "id", OFTString );
		oField.SetWidth(32);
		if( poLayer->CreateField( &oField ) != OGRERR_NONE )
		{
			return false;
		}
		for(unsigned int i = 0;i < tiefeatureList.size();i++)
		{
			if(rspfFeature::rspfFeatureType::rspfAreaType == tiefeatureList[i].getBaseFeature().m_featureType)
			{
				OGRFeature *poFeature;
				poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
				poFeature->SetField( "id", tiefeatureList[i].strId.c_str() );

				OGRPolygon polygon;
				OGRLinearRing *pOGRLinearRing = new OGRLinearRing();
				int nPoint = tiefeatureList[i].getBaseFeature().m_Points.size();
				OGRRawPoint* pointsList = new OGRRawPoint[nPoint];
				for(int j = 0;j < nPoint;j++)
				{
					pointsList[j].x = tiefeatureList[i].getBaseFeature().m_Points[j].x;
					pointsList[j].y = tiefeatureList[i].getBaseFeature().m_Points[j].y;
				}
				pOGRLinearRing->setNumPoints(nPoint);
				pOGRLinearRing->setPoints(nPoint, pointsList);
				pOGRLinearRing->closeRings();
				polygon.addRing(pOGRLinearRing);

				poFeature->SetGeometry( &polygon ); 

				if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
				{
					return false;
				}
				delete []pointsList;
				OGRFeature::DestroyFeature( poFeature );
			}
		}
	}

	OGRDataSource::DestroyDataSource( poDS );
	return true;
}



string SBeforeLast(const string& str, char ch)
{
	int pos = str.find_last_of(ch);
	return str.substr(0, pos);
}

string SAfterFirst(const string& str, char ch)
{
	int pos = str.find_first_of(ch);
	return str.substr(pos + 1, str.size() - 1);
}

string SAfterLast(const string& str, char ch)
{
	int pos = str.find_last_of(ch);
	return str.substr(pos + 1, str.size() - 1);
}

string SBeforeFirst(const string& str, char ch)
{
	int pos = str.find_first_of(ch);
	return str.substr(0, pos);
}

bool UpdateFeatures(rspfGeoModel* geoModel, vector<rspfTieFeature>& featureList)
{
	for(unsigned int i = 0;i < featureList.size();i++)
	{
		rspfFeature newFeature(featureList[i].getWarpFeature().m_featureType);
		for(unsigned int j = 0;j < featureList[i].getWarpFeature().m_Points.size();j++)
		{
			rspfPoint pt = geoModel->forward(featureList[i].getWarpFeature().m_Points[j]);
			newFeature.m_Points.push_back(pt);
		}
		featureList[i].setBaseFeature(newFeature);
	}
	return true;
}

#pragma warning(pop)