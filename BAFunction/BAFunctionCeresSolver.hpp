//
//  BAFunctionCeresSolver.hpp
//  BAFunction
//
//  Created by Kalpesh Modi on 09/10/23.
//

#if defined (__cplusplus)


#ifndef BAFunctionCeresSolver_hpp
#define BAFunctionCeresSolver_hpp

 

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <ceres/solver.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>


namespace BAFunctionCeres{

constexpr int PARAMETERS_COUNT_FOR_VIEWS = 7;
constexpr int PARAMETERS_COUNT_FOR_LANDMARK_POINTS = 3;
#define TOTALPARAMETERS(x,y) (((x)*(PARAMETERS_COUNT_FOR_LANDMARK_POINTS))+ ((y) * (PARAMETERS_COUNT_FOR_VIEWS)))

std::string LANDMARKCOUNT = "LANDMARKCOUNT";
std::string POINTCOUNT = "POINTCOUNT";
std::string VIEWCOUNT = "VIEWCOUNT";

class BAFunctionCeresSolver {
public:
    BAFunctionCeresSolver();
    
};

class MyToolScalarCostFunctorForBundleAdjustment{
public:
    MyToolScalarCostFunctorForBundleAdjustment(){}
    MyToolScalarCostFunctorForBundleAdjustment(const double u, const double v): uValue(u), vValue(v){
        
    }
    template <typename T>
    bool operator() (const T * const iValue, const T * const jValue, T * error) const {
        
        //obtain view parameters
        T p [3];
        ceres::AngleAxisRotatePoint(iValue, jValue, p);
        p[0] += iValue[3];
        p[1] += iValue[4];
        p[2] += iValue[5];
        const T& f = iValue[6];
        
        T iValueX = -p[0] / p[2]; // check for -ve sign
        T iValueY = -p[1] / p[2];
        
        T calcUValue = f * iValueX;
        T calcVValue = f * iValueY;
        
        error[0] = calcUValue - uValue; // compute error between observed values.x and measured u values
        error[1] = calcVValue - vValue; // computer error between observed values.y and measured v values
        return true;
    }
    static ceres::CostFunction * Create(const double u, const double v) {
         return (new ceres::AutoDiffCostFunction<MyToolScalarCostFunctorForBundleAdjustment,2,3,7>(new MyToolScalarCostFunctorForBundleAdjustment(u,v)));
    }
private:
      double uValue;
      double vValue;
};


class PrepareData{
public:
    ~PrepareData()
    {
        delete [] landmarks;
        delete [] landmarkPoints;
        delete [] landmarkViews;
        delete [] parameters;
    }
    PrepareData() {};
bool prepareDataFromFile(std::string & filename,int & o, int & p, int &v);
    
inline void setLandmarkPoint(int count) { landmarksCount = count; }
inline void setPointCount (int count) { pointsCount = count; }
inline void setViewCount (int count) {viewCount = count;}

inline int obtainLandMarksCount () const {return landmarksCount;}
inline double * obtainLandMarks () const {return landmarks;}
inline double * obtainLandMarkPoint (int index) {return (parameters + ((PARAMETERS_COUNT_FOR_VIEWS* viewCount) + ( landmarkPoints[index] * PARAMETERS_COUNT_FOR_LANDMARK_POINTS))); }
inline double * obtainLandMarkView (int index) {return parameters + landmarkViews[index]  * PARAMETERS_COUNT_FOR_VIEWS;}

    //prepare cost function params
void setParameters(double * data) {
    int parameterCount =  TOTALPARAMETERS(pointsCount,viewCount);
    parameters = new double [parameterCount];
    for(int index = 0; index < parameterCount; ++index){
        *(parameters + index) = data[index];
    }
}
    
void setLandmarks(double * data) {
    landmarks = new double [2 * landmarksCount];
    for(int index =0; index < landmarksCount; ++index){
        for(int j = 0; j < 2; j++){ //XYZ
            *(landmarks + (2 * index + j)) = data [index];
        }
    }
};
    
    void setLandmarkPoints(int * data) {
        landmarkPoints  = new int [landmarksCount];
        for(int index =0; index < landmarksCount; ++index){
            *(landmarkPoints + index) = data[index];
        }
    };
    
    void setLandmarkViews(int * data) {
        landmarkViews = new int [landmarksCount];
        for(int index = 0; index < landmarksCount; index++){
            *(landmarkViews + index) = data[index];
        }
    };
    
    
    
private:
    int pointsCount = 0;
    int viewCount = 0;
    int landmarksCount = 0;

    double * landmarks;
    int * landmarkPoints;
    int * landmarkViews;
    double * parameters;

};

}



#endif /* BAFunctionCeresSolver_hpp */

#endif /* end #define cpp*/
