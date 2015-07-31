#ifndef VISOMONO_H
#define VISOMONO_H

#include "pointstruct.h"
#include "matrix.h"
#include<algorithm>
#include<vector>


  template<class T> struct idx_cmp {
    idx_cmp(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const { return arr[a] < arr[b]; }
    const T arr;
  }; 
vector<double> estimateMotion (vector <p_match> p_matched, double K_data[9],parameters param);
Matrix smallerThanMedian (Matrix &X,double &median);
bool normalizeFeaturePoints(vector <p_match> &p_matched,Matrix &Tp,Matrix &Tc);
void fundamentalMatrix (const vector <p_match> &p_matched,const vector<int32_t> &active,Matrix &F);
vector<int32_t> getInlier (vector <p_match> &p_matched,Matrix &F,parameters param);
void EtoRt(Matrix &E,Matrix &K,vector <p_match> &p_matched,Matrix &X,Matrix &R,Matrix &t);
int32_t triangulateChieral (vector <p_match> &p_matched,Matrix &K,Matrix &R,Matrix &t,Matrix &X);
vector<int32_t> getRandomSample(int32_t N,int32_t num);
Matrix transformationVectorToMatrix (vector<double> tr);
bool updateMotion (vector<double> tr_delta,Matrix &Tr_delta);
#endif