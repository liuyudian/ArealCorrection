#include "rspfGeoModel.h"
#include "levmar.h"
#pragma warning(push)
#pragma warning(disable : 4482)

double rspfGeoModel::NormInfinity(const cv::Mat& m)const
{
	if(1 != m.cols && 1 != m.rows) return 0.0;
	else if(1 == m.cols)
	{
		int n = m.rows;
		double inf = 0.0;
		for(int i = 0;i < n;i++)
		{
			double dt = m.at<double>(i, 0);
			if(fabs(dt) > inf) inf = fabs(dt);
		}
		return inf;
	}
	else
	{
		int n = m.cols;
		double inf = 0.0;
		for(int i = 0;i < n;i++)
		{
			double dt = m.at<double>(0, i);
			if(fabs(dt) > inf) inf = fabs(dt);
		}
		return inf;
	}

}

double rspfGeoModel::SumSquare(const cv::Mat& m)const
{
	if(1 != m.cols && 1 != m.rows) return 0.0;
	else if(1 == m.cols)
	{
		int n = m.rows;
		double sumsqrt = 0.0;
		for(int i = 0;i < n;i++)
		{
			double dt = m.at<double>(i, 0);
			sumsqrt += dt * dt;
		}
		return sumsqrt;
		//return sqrt(sumsqrt / n);
	}
	else
	{
		int n = m.cols;
		double sumsqrt = 0.0;
		for(int i = 0;i < n;i++)
		{
			double dt = m.at<double>(0, i);
			sumsqrt += dt * dt;
		}
		return sumsqrt;
		//return sqrt(sumsqrt / n);
	}
}

void rspfGeoModel::setAdjustableParameter(int idx, double value, bool notify)
{
	if(!m_parmlist.size())
	{
		return;
	}
	if(idx < static_cast<int>(m_parmlist.size()))
	{
		m_parmlist[idx] = value;
		if(notify)
		{
			adjustableParametersChanged();
		}
	}
}

void rspfGeoModel::funcErrorEquation(double *param, double *hx, int m, int n, void *adata)
{
	optimizeStruct *pOptimizeStruct = (optimizeStruct*)adata;
	int nobs = pOptimizeStruct->tieFeatureList.size();
	int np = m;
	std::vector<rspfPoint> imDerp(np*2);
	int c = 0;
	int i;
	for(i=0;i<np;++i)
	{
		pOptimizeStruct->pThis->setAdjustableParameter(i, param[i], false); //do not update right now
	}
	for (i = 0 ; i < nobs ; ++i)
	{
		switch(pOptimizeStruct->tieFeatureList[i].getTieFeatureType())
		{
		case rspfTieFeature::rspfTieFeatureType::rspfTiePointPoint:
			{
				rspfPoint resIm;
				resIm = pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points[0] - pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[0]);
				hx[c++] = -resIm.x;
				hx[c++] = -resIm.y;
				break;
			}
		case rspfTieFeature::rspfTieFeatureType::rspfTieLineLine:
			{
				rspfPoint newDpt1 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[0]);
				rspfPoint gpt1 = pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points[0];
				rspfPoint gpt2 = pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points[1];
				double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
				double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
				double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

				//for the first endpoint of a line
				hx[c++] = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt1.x + k2 * newDpt1.y;

				//for the second endpoint of a line
				rspfPoint newDpt2 = pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[1]);
				hx[c++] = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt2.x + k2 * newDpt2.y;
				break;
			}
		case rspfTieFeature::rspfTieFeatureType::rspfTieAreaArea:
			{
				//
				vector<rspfPoint> newPoints;
				for(unsigned int iVertex = 0;iVertex < pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points.size();iVertex++)
				{
					newPoints.push_back(pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[iVertex]));
				}
				rspfArea oldArea(pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points);
				rspfArea newArea(newPoints);
				int nxIndex, nyIndex;
				bool bxUpper, byUpper;
				double xDis = oldArea.xDistanceFromArea(newArea, &bxUpper, &nxIndex);
				double yDis = oldArea.yDistanceFromArea(newArea, &byUpper, &nyIndex);
				hx[c++] = xDis;
				hx[c++] = yDis;

				break;
			}

		default:
			{

			}
		}
	}

}
void rspfGeoModel::jacErrorEquation(double *param, double *j, int m, int n, void *adata)
{
	optimizeStruct *pOptimizeStruct = (optimizeStruct*)adata;

	int nobs = pOptimizeStruct->tieFeatureList.size();
	int np = m;
	double pstep_scale = 1e-4;
	std::vector<rspfPoint> imDerp(np*2);
	int c = 0;
	int i;
	for(i=0;i<np;++i)
	{
		pOptimizeStruct->pThis->setAdjustableParameter(i, param[i], false); //do not update right now
	}
	for (i = 0 ; i < nobs ; ++i)
	{
		switch(pOptimizeStruct->tieFeatureList[i].getTieFeatureType())
		{
		case rspfTieFeature::rspfTieFeatureType::rspfTiePointPoint:
			{
				rspfPoint resIm;
				resIm = pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points[0] - pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[0]);
				//compute all image derivatives regarding parametres for the tie point position
				for(int p=0;p<np;++p)
				{
					imDerp[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[0], pstep_scale);
				}

				for(int p=0;p<np;++p)
				{
					j[c++] = imDerp[p].x;
				}
				for(int p=0;p<np;++p)
				{
					j[c++] = imDerp[p].y;
				}

				break;
			}
		case rspfTieFeature::rspfTieFeatureType::rspfTieLineLine:
			{
				rspfPoint gpt1 = pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points[0];
				rspfPoint gpt2 = pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points[1];
				double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
				double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
				double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

				//for the first endpoint of a line				
				for(int p=0;p<np;++p)
				{
					imDerp[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[0], pstep_scale);
				}
				for(int p=0;p<np;++p)
				{
					j[c++] = k1 * imDerp[p].x - k2 * imDerp[p].y;
				}

				//for the second endpoint of a line
				for(int p=0;p<np;++p)
				{
					imDerp[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[1], pstep_scale);
				}
				for(int p=0;p<np;++p)
				{
					j[c++] = k1 * imDerp[p].x - k2 * imDerp[p].y;
				}
				break;
			}
		case rspfTieFeature::rspfTieFeatureType::rspfTieAreaArea:
			{
				//
				vector<rspfPoint> newPoints;
				for(unsigned int iVertex = 0;iVertex < pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points.size();iVertex++)
				{
					newPoints.push_back(pOptimizeStruct->pThis->forward(pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[iVertex]));
				}
				rspfArea oldArea(pOptimizeStruct->tieFeatureList[i].getBaseFeature().m_Points);
				rspfArea newArea(newPoints);	

				int nxIndex, nyIndex;
				bool bxUpper, byUpper;
				double xDis = oldArea.xDistanceFromArea(newArea, &bxUpper, &nxIndex);
				double yDis = oldArea.yDistanceFromArea(newArea, &byUpper, &nyIndex);

				//for(int p=0;p<np;++p)
				//{
				//	imDerp[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[nxIndex], pstep_scale);
				//	imDerp[p+np] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[nyIndex], pstep_scale);
				//}


				//// ¾àÀëXº¯Êý¶ÔÏñËØ×ø±êÎ¢·Ö
				//rspfPoint patialDistance_X;
				//// ¾àÀëYº¯Êý¶ÔÏñËØ×ø±êÎ¢·Ö
				//rspfPoint patialDistance_Y;
				//oldArea.getPointXDistanceDeriv(newArea.m_Points[nxIndex], bxUpper, &patialDistance_X);
				//oldArea.getPointYDistanceDeriv(newArea.m_Points[nyIndex], byUpper, &patialDistance_Y);

				//for(int p=0;p<np;++p)
				//{
				//	j[c++] = patialDistance_X.x * imDerp[p].x + patialDistance_X.y * imDerp[p].y;
				//}
				//for(int p=0;p<np;++p)
				//{
				//	j[c++] = patialDistance_Y.x * imDerp[p+np].x + patialDistance_Y.y * imDerp[p+np].y;
				//}

				for(int p=0;p<np;++p)
				{
					imDerp[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[nxIndex], pstep_scale);
					j[c++] = imDerp[p].x;
				}

				for(int p=0;p<np;++p)
				{
					imDerp[p] = pOptimizeStruct->pThis->getForwardDeriv( p , pOptimizeStruct->tieFeatureList[i].getWarpFeature().m_Points[nyIndex], pstep_scale);
					j[c++] = imDerp[p].y;
				}

				break;
			}

		default:
			{

			}
		}
	}
}

double rspfGeoModel::optimizeFit(const vector< rspfTieFeature >& tieFeatureList)
{
	//use a simple Levenberg-Marquardt non-linear optimization
	//note : please limit the number of tie points
	//
	//INPUTS: requires Jacobian matrix (partial derivatives with regards to parameters)
	//OUPUTS: will also compute parameter covariance matrix
	//
	//TBD: use targetVariance!
	int nparam = getNumberOfAdjustableParameters();
	int nobs = 2*tieFeatureList.size();
	//setup initail values
	int iter=0;
	int iter_max = 200;    //ww1130
	double minResidue = 1e-10; //TBC
	double minDelta = 1e-10; //TBC

	std::vector<double> cparm;
	getAdjustment(cparm);
	double *p = &cparm[0];
	double *x = new double[nobs];
	for(int i=0; i<nobs; i++) x[i]=0.0;

	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
	opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing 
	
	//build Least Squares initial normal equation
	// don't waste memory, add samples one at a time
	optimizeStruct optStruct;
	optStruct.tieFeatureList = tieFeatureList;
	optStruct.pThis = this;

	//double errors;
	//dlevmar_chkjac(funcErrorEquation, jacErrorEquation, p, nparam, nobs, &optStruct, &errors);

	/* Rosenbrock function */
	//int ret = dlevmar_der(funcErrorEquation, jacErrorEquation, p, x, nparam, nobs, 1000, opts, info, NULL, NULL, &optStruct); // with analytic Jacobian
	int ret = dlevmar_dif(funcErrorEquation, p, x, nparam, nobs, 1000, opts, info, NULL, NULL, &optStruct);  // no Jacobian

	return 0.0/nobs;
}

//double rspfGeoModel::optimizeFit(const vector< rspfTieFeature >& tieFeatureList)
//{
//	//use a simple Levenberg-Marquardt non-linear optimization
//	//note : please limit the number of tie points
//	//
//	//INPUTS: requires Jacobian matrix (partial derivatives with regards to parameters)
//	//OUPUTS: will also compute parameter covariance matrix
//	//
//	//TBD: use targetVariance!
//	int np = getNumberOfAdjustableParameters();
//	int nobs = tieFeatureList.size();
//	//setup initail values
//	int iter=0;
//	int iter_max = 200;    //ww1130
//	double minResidue = 1e-10; //TBC
//	double minDelta = 1e-10; //TBC
//	//build Least Squares initial normal equation
//	// don't waste memory, add samples one at a time
//	cv::Mat A;
//	cv::Mat residue;
//	cv::Mat projResidue;
//	double deltap_scale = 1e-4; //step_Scale is 1.0 because we expect parameters to be between -1 and 1
//	bool useImageObs = true;	//2010.1.18 loong
//	buildNormalEquation(tieFeatureList, A, residue, projResidue, deltap_scale);	//2010.1.18 loong
//	double ki2 = SumSquare(residue);
//	
//	//get current adjustment (between -1 and 1 normally) and convert to ColumnVector
//	std::vector<double> cparm;
//	std::vector<double> nparm;
//	getAdjustment(cparm);
//	nparm = cparm;
//	double damping_speed = 2.0;
//	//find max diag element for A
//	double maxdiag=0.0;
//
//	for(int d=0;d<np;++d) {
//		double a = A.at<double>(d, d);
//		if (maxdiag < a) maxdiag = a;
//	}
//	double damping = 1e-3 * maxdiag;
//	double olddamping = 0.0;
//	bool found = false;
//	//DEBUG TBR
//	// cout<<"rms="<<sqrt(ki2/nobs)<<" ";
//	// cout.flush();
//	while ( (!found) && (iter < iter_max) ) //non linear optimization loop
//	{
//		bool decrease = false;
//		do
//		{
//			//add damping update to normal matrix
//			for(int d = 0;d < np;++d) 
//			{
//				A.at<double>(d, d) += damping - olddamping;
//			}
//
//			olddamping = damping;
//
//			cv::Mat deltap = solveLeastSquares(A, projResidue);
//			//cout<<deltap<<endl; //2010.1.18 loong
//			if (NormInfinity(deltap) <= minDelta) 
//			{
//				found = true;
//			} else {
//				//update adjustment
//				for(int n = 0;n < np;++n)
//				{
//					nparm[n] = cparm[n] + deltap.at<double>(n, 0);
//				}
//				for(int n=0;n<np;++n)
//				{
//					setAdjustableParameter(n, nparm[n], false); //do not update now, wait
//				}
//				updateModel();
//				//rspfKeywordlist ge;
//				//saveState(ge);
//				//cout<<ge<<endl<<endl;
//				//check residue is reduced
//				cv::Mat newresidue = getResidue(tieFeatureList);	//2010.1.18 loong
//				double newki2 = SumSquare(newresidue);
//				double res_reduction = (ki2 - newki2) / NormInfinity((deltap.t()*(deltap*damping + projResidue)));
//				//DEBUG TBR
//				cout<<sqrt(newki2/nobs)<<" ";
//				cout.flush();
//				if (res_reduction > 0)
//				{
//					//accept new parms
//					cparm = nparm;
//					ki2=newki2;
//					deltap_scale = max(1e-15, SumSquare(deltap)*1e-4);
//					buildNormalEquation(tieFeatureList, A, residue, projResidue, deltap_scale);	//2010.1.18 loong
//					olddamping = 0.0;
//					found = (NormInfinity(projResidue) <= minResidue );
//					//update damping factor
//					damping *= std::max( 1.0/3.0, 1.0-std::pow((2.0*res_reduction-1.0),3));
//					damping_speed = 2.0;
//					decrease = true;
//				} else {
//					//cancel parameter update
//					for(int n=0;n<np;++n)
//					{
//						setAdjustableParameter(n, cparm[n], false); //do not update right now
//					}
//					updateModel();
//					damping *= damping_speed;
//					damping_speed *= 2.0;
//				}
//			}
//		} while (!decrease && !found);
//		++iter;
//	}
//	//DEBUG TBR
//	cout<<endl;
//	//compute parameter correlation
//	// use normal matrix inverse
//	//TBD
//	return ki2/nobs;
//}


void rspfGeoModel::buildNormalEquation(const vector< rspfTieFeature >& tieFeatureList,
									 cv::Mat& A,
									 cv::Mat& residue,
									 cv::Mat& projResidue,
									 double pstep_scale)
{
	//goal:       build Least Squares system
	//constraint: never store full Jacobian matrix in memory (can be huge)
	//            so we build the matrices incrementally
	// the system can be built using forward() or inverse() depending on the projection capabilities : useForward()
	//
	//TBD : add covariance matrix for each tie point
	//init
	int np = getNumberOfAdjustableParameters();
	//bool useImageObs = useForward(); //caching			//2010.1.18 loong
	int nobs = tieFeatureList.size();
	//int no = dimObs * nobs; //number of observations
	int no = getNumofObservations(tieFeatureList);
	A.create(np, np, CV_64F);
	residue.create(no, 1, CV_64F);
	projResidue.create(np, 1, CV_64F);
	//Zeroify matrices that will be accumulated
	A           = 0.0;
	projResidue = 0.0;
	unsigned long c = 0;
	double N=0;
	double e2=0;
	double a=6378137;
	double b=6356752.3;
	e2=(a*a-b*b)/(a*a) ;
	int index=0;
	int i;
	std::vector<rspfPoint> imDerp(np*2);
	for (i = 0 ; i < nobs ; ++i)
	{
		switch(tieFeatureList[i].getTieFeatureType())
		{
		case rspfTieFeature::rspfTieFeatureType::rspfTiePointPoint:
			{
				rspfPoint resIm;
				resIm = tieFeatureList[i].getBaseFeature().m_Points[0] - forward(tieFeatureList[i].getWarpFeature().m_Points[0]);
				residue.at<double>(c++, 0) = resIm.x;
				residue.at<double>(c++, 0) = resIm.y;
				//compute all image derivatives regarding parametres for the tie point position
				for(int p=0;p<np;++p)
				{
					imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getWarpFeature().m_Points[0], pstep_scale);
				}
				//compute influence of tie point on all sytem elements
				for(int p1=0;p1<np;++p1)
				{        
					//proj residue: J * residue
					projResidue.at<double>(p1, 0) += imDerp[p1].x * resIm.x + imDerp[p1].y * resIm.y;
					//normal matrix A = transpose(J)*J
					for(int p2=p1;p2<np;++p2)
					{
						A.at<double>(p1, p2) += imDerp[p1].x * imDerp[p2].x + imDerp[p1].y * imDerp[p2].y;
					}
				}
				break;
			}
		case rspfTieFeature::rspfTieFeatureType::rspfTieLineLine:
			{
				rspfPoint newDpt1 = forward(tieFeatureList[i].getWarpFeature().m_Points[0]);
				rspfPoint gpt1 = tieFeatureList[i].getBaseFeature().m_Points[0];
				rspfPoint gpt2 = tieFeatureList[i].getBaseFeature().m_Points[1];
				double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
				double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
				double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

				//for the first endpoint of a line
				residue.at<double>(c++, 0) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt1.x + k2 * newDpt1.y;

				for(int p=0;p<np;++p)
				{
					imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getWarpFeature().m_Points[0], pstep_scale);
				}
				//compute influence of tie point on all system elements
				for(int p1=0;p1<np;++p1)
				{        
					//proj residue: J * residue
					projResidue.at<double>(p1, 0) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * residue.at<double>(c - 1, 0);
					//normal matrix A = transpose(J)*J
					for(int p2=p1;p2<np;++p2)
					{
						A.at<double>(p1, p2) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * (k1 * imDerp[p2].x - k2 * imDerp[p2].y);
					}
				}

				//for the second endpoint of a line

				rspfPoint newDpt2 = forward(tieFeatureList[i].getWarpFeature().m_Points[1]);
				residue.at<double>(c++, 0) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt2.x + k2 * newDpt2.y;
				for(int p=0;p<np;++p)
				{
					imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getWarpFeature().m_Points[1], pstep_scale);
				}
				//compute influence of tie point on all system elements
				for(int p1=0;p1<np;++p1)
				{        
					//proj residue: J * residue
					projResidue.at<double>(p1, 0) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * residue.at<double>(c - 1, 0);
					//normal matrix A = transpose(J)*J
					for(int p2=p1;p2<np;++p2)
					{
						A.at<double>(p1, p2) += (k1 * imDerp[p1].x - k2 * imDerp[p1].y) * (k1 * imDerp[p2].x - k2 * imDerp[p2].y);
					}
				}
				break;
			}
		case rspfTieFeature::rspfTieFeatureType::rspfTieAreaArea:
			{
				//
				vector<rspfPoint> newPoints;
				for(unsigned int iVertex = 0;iVertex < tieFeatureList[i].getWarpFeature().m_Points.size();iVertex++)
				{
					newPoints.push_back(forward(tieFeatureList[i].getWarpFeature().m_Points[iVertex]));
				}
				rspfArea oldArea(tieFeatureList[i].getBaseFeature().m_Points);
				rspfArea newArea(newPoints);	
				//int nIndex;
				//rspfPoint tmpDisVector = oldArea.DistanceFromArea(newArea, &nIndex);
				//residue.at<double>(c++, 0) = -tmpDisVector.x;
				//residue.at<double>(c++, 0) = -tmpDisVector.y;

				//for(int p=0;p<np;++p)
				//{
				//	imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getWarpFeature().m_Points[nIndex], pstep_scale);
				//}


				//// ¾àÀëXº¯Êý¶ÔÏñËØ×ø±êÎ¢·Ö
				//rspfPoint patialDistance_X;
				//// ¾àÀëYº¯Êý¶ÔÏñËØ×ø±êÎ¢·Ö
				//rspfPoint patialDistance_Y;
				//oldArea.getPointDistanceDeriv(newArea.m_Points[nIndex], &patialDistance_X, &patialDistance_Y);

				////compute influence of tie point on all system elements
				//for(int p1=0;p1<np;++p1)
				//{        
				//	//proj residue: J * residue
				//	projResidue.at<double>(p1, 0) += (patialDistance_X.x * imDerp[p1].x + patialDistance_X.y * imDerp[p1].y) * (-tmpDisVector.x);
				//	projResidue.at<double>(p1, 0) += (patialDistance_Y.x * imDerp[p1].x + patialDistance_Y.y * imDerp[p1].y) * (-tmpDisVector.y);
				//	//normal matrix A = transpose(J)*J
				//	for(int p2=p1;p2<np;++p2)
				//	{
				//		A.at<double>(p1, p2) += (patialDistance_X.x * imDerp[p1].x + patialDistance_X.y * imDerp[p1].y) * (patialDistance_X.x * imDerp[p2].x + patialDistance_X.y * imDerp[p2].y);
				//		A.at<double>(p1, p2) += (patialDistance_Y.x * imDerp[p1].x + patialDistance_Y.y * imDerp[p1].y) * (patialDistance_Y.x * imDerp[p2].x + patialDistance_Y.y * imDerp[p2].y);
				//	}
				//}


				int nxIndex, nyIndex;
				bool bxUpper, byUpper;
				double xDis = oldArea.xDistanceFromArea(newArea, &bxUpper, &nxIndex);
				double yDis = oldArea.yDistanceFromArea(newArea, &byUpper, &nyIndex);
				residue.at<double>(c++, 0) = -xDis;
				residue.at<double>(c++, 0) = -yDis;

				for(int p=0;p<np;++p)
				{
					imDerp[p] = getForwardDeriv( p , tieFeatureList[i].getWarpFeature().m_Points[nxIndex], pstep_scale);
					imDerp[p+np] = getForwardDeriv( p , tieFeatureList[i].getWarpFeature().m_Points[nyIndex], pstep_scale);
				}


				// ¾àÀëXº¯Êý¶ÔÏñËØ×ø±êÎ¢·Ö
				rspfPoint patialDistance_X;
				// ¾àÀëYº¯Êý¶ÔÏñËØ×ø±êÎ¢·Ö
				rspfPoint patialDistance_Y;
				oldArea.getPointXDistanceDeriv(newArea.m_Points[nxIndex], bxUpper, &patialDistance_X);
				oldArea.getPointYDistanceDeriv(newArea.m_Points[nyIndex], byUpper, &patialDistance_Y);

				//compute influence of tie point on all system elements
				for(int p1=0;p1<np;++p1)
				{        
					//proj residue: J * residue
					projResidue.at<double>(p1, 0) += (patialDistance_X.x * imDerp[p1].x + patialDistance_X.y * imDerp[p1].y) * (-xDis);
					projResidue.at<double>(p1, 0) += (patialDistance_Y.x * imDerp[p1+np].x + patialDistance_Y.y * imDerp[p1+np].y) * (-yDis);
					//normal matrix A = transpose(J)*J
					for(int p2=p1;p2<np;++p2)
					{
						A.at<double>(p1, p2) += (patialDistance_X.x * imDerp[p1].x + patialDistance_X.y * imDerp[p1].y) * (patialDistance_X.x * imDerp[p2].x + patialDistance_X.y * imDerp[p2].y);
						A.at<double>(p1, p2) += (patialDistance_Y.x * imDerp[p1+np].x + patialDistance_Y.y * imDerp[p1+np].y) * (patialDistance_Y.x * imDerp[p2+np].x + patialDistance_Y.y * imDerp[p2+np].y);
					}
				}
				break;
			}

		default:
			{

			}
		}
	}
}


cv::Mat rspfGeoModel::getResidue(const vector< rspfTieFeature >& tieFeatureList)
{

	//init
	cv::Mat residue;
	rspfPoint tmplast,tmpnew;
	int nobs = tieFeatureList.size();
	int no = getNumofObservations(tieFeatureList);
	residue.create(no, 1, CV_64F);
	unsigned long c = 0;
	int i;
	// loop on tie points
	for (i = 0 ; i < nobs ; ++i)
	{
		switch(tieFeatureList[i].getTieFeatureType())
		{
		case  rspfTieFeature::rspfTiePointPoint:
			{
				rspfPoint resIm;
				resIm = tieFeatureList[i].getBaseFeature().m_Points[0] - forward(tieFeatureList[i].getWarpFeature().m_Points[0]);
				residue.at<double>(c++, 0) = resIm.x;
				residue.at<double>(c++, 0) = resIm.y;
				break;
			}
		case rspfTieFeature::rspfTieLineLine:
			{
				rspfPoint newDpt1 = forward(tieFeatureList[i].getWarpFeature().m_Points[0]);
				rspfPoint newDpt2 = forward(tieFeatureList[i].getWarpFeature().m_Points[1]);
				rspfPoint gpt1 = tieFeatureList[i].getBaseFeature().m_Points[0];
				rspfPoint gpt2 = tieFeatureList[i].getBaseFeature().m_Points[1];
				double hemline = sqrt((gpt1.x - gpt2.x)*(gpt1.x - gpt2.x) + (gpt1.y - gpt2.y)*(gpt1.y - gpt2.y));
				double k1 = (gpt1.y - gpt2.y) / (hemline + small_quantity);
				double k2 = (gpt1.x - gpt2.x) / (hemline + small_quantity);

				//for the first endpoint of a line
				residue.at<double>(c++, 0) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt1.x + k2 * newDpt1.y;
				residue.at<double>(c++, 0) = (gpt2.x * gpt1.y - gpt1.x * gpt2.y) / (hemline + small_quantity) - k1 * newDpt2.x + k2 * newDpt2.y;
				break;
			}
		case rspfTieFeature::rspfTieAreaArea:
			{
				vector<rspfPoint> newPoints;
				for(unsigned int iVertex = 0;iVertex < tieFeatureList[i].getWarpFeature().m_Points.size();iVertex++)
				{
					newPoints.push_back(forward(tieFeatureList[i].getWarpFeature().m_Points[iVertex]));
				}
				rspfArea oldArea(tieFeatureList[i].getBaseFeature().m_Points);
				rspfArea newArea(newPoints);	
				int nIndex;
				rspfPoint tmpDisVector = oldArea.DistanceFromArea(newArea, &nIndex);
				residue.at<double>(c++, 0) = -tmpDisVector.x;
				residue.at<double>(c++, 0) = -tmpDisVector.y;
				break;
			}
		default:
			{

			}
		}
	}
	return residue;
}

/*!
* solves Ax = r , with A symmetric positive definite
* A can be rank deficient
* size of A is typically between 10 and 100 rows
*/
cv::Mat rspfGeoModel::solveLeastSquares(cv::Mat& A,  cv::Mat& r)const
{
	return A.inv() * r;
}

int rspfGeoModel::getNumofObservations(const vector< rspfTieFeature >& tieFeatureList)
{
	int no = 0;
	for(unsigned int i = 0 ;i < tieFeatureList.size();i++)
	{
		switch(tieFeatureList[i].getTieFeatureType())
		{
		case  rspfTieFeature::rspfTiePointPoint:
			{
				no += 2;
				break;
			}
		case rspfTieFeature::rspfTieLineLine:
			{
				no += 2;
				break;
			}
		case rspfTieFeature::rspfTieAreaArea:
			{
				no += 2;
				break;
			}
		case  rspfTieFeature::rspfTieLinePoint:
			{
				break;
			}
		case rspfTieFeature::rspfTieAreaLine:
			{
				break;
			}
		case rspfTieFeature::rspfTiePointArea:
			{
				break;
			}
		case  rspfTieFeature::rspfTieAreaPoint:
			{
				break;
			}
		case rspfTieFeature::rspfTiePointLine:
			{
				break;
			}
		case rspfTieFeature::rspfTieLineArea:
			{
				break;
			}
		default:
			{
				break;
			}
		}
	}
	return no;
}

void rspfGeoModel::getAdjustment(std::vector<double>& adj)
{
	adj = m_parmlist;
}

double rspfGeoModel::getAdjustableParameter(int idx)
{
	if(idx < static_cast<int>(m_parmlist.size()))
	{
		return m_parmlist[idx];
	}
	else return 0.0;
}

rspfPoint
rspfGeoModel::getInverseDeriv(int parmIdx, const rspfPoint& pt,double hdelta)
{   
	double den = 0.5/hdelta;
	rspfPoint res;
	double middle = getAdjustableParameter(parmIdx);
	//set parm to high value
	setAdjustableParameter(parmIdx, middle + hdelta, true);
	res = inverse(pt);
	//set parm to low value and gte difference
	setAdjustableParameter(parmIdx, middle - hdelta, true);
	res -= inverse(pt);
	res = res*den;
	setAdjustableParameter(parmIdx, middle, true);
	return res;
}
rspfPoint
rspfGeoModel::getForwardDeriv(int parmIdx, const rspfPoint& pt, double hdelta)
{   
	double den = 0.5/hdelta;
	rspfPoint res;
	double middle = getAdjustableParameter(parmIdx);
	//set parm to high value
	setAdjustableParameter(parmIdx, middle + hdelta, true);
	res = forward(pt);
	//set parm to low value and get difference
	setAdjustableParameter(parmIdx, middle - hdelta, true);
	res -= forward(pt);
	//get partial derivative
	res = res*den;
	//reset parm
	setAdjustableParameter(parmIdx, middle, true);
	return res;
}

int rspfGeoModel::degreesOfFreedom()const
{
	return getNumberOfAdjustableParameters();
}

void rspfGeoModel::warpToBase(const rspfPoint& warp_point,
						rspfPoint&       base_point) const
{
	base_point = forward(warp_point);
}
void  rspfGeoModel::baseToWarp(const rspfPoint& base_point,
							   rspfPoint&       warp_point) const
{
	warp_point = inverse(base_point);
}

std::ostream& rspfGeoModel::print(std::ostream& out) const
{
	int num = getNumberOfAdjustableParameters();
	out<<num<<":"<<endl;
	for(int i = 0;i < num;i++)
	{
		out<<m_parmlist[i]<<endl;
	}
	return out;
}
#pragma warning(pop)