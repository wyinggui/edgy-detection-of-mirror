#pragma once
#include <iostream>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class LineFind
{
  public:
    LineFind();

    bool  lineFilter(vector<Vec4f>& lines_std,vector<float>& lines_dis,float thresh);
//    Vec2f getPolarLine(Vec4f p,bool mark,float thresh);
    Vec2f getPolarLine(Vec4f p);
    vector<Vec2f> vecPolarLine(vector<Vec4f> lines_std);
    bool getIndexWithPolarLine(vector<int>& _index,vector<Vec2f> polarLines);
    void indexSort(vector<int> _index,vector<vector<int>> &indexContent,vector<int> &indexLen);
    void lineDisFilter(vector<Vec4f> lines_filter,vector<float> lines_dis,vector<Vec2f> polarLines,
                       vector<vector<Vec4f>>& lines_filter_shim_all,vector<vector<Vec2f>>& polarLines_shim_all,
                       vector<vector<float>>& lines_dis_shim_all,vector<float>& lines_dis_sum,vector<vector<int>>& indexContent,
                       vector<int>& indexLen,float thresh);
    void  lineFusion(Vec2f imageSize, vector<vector<Vec4f>> lines_filter_shim_all,vector<vector<Vec2f>> polarLines_shim_all,
                                vector<vector<float>> lines_dis_shim_all,vector<float> lines_dis_sum,vector<vector<int>> indexContent,
                                vector<int> indexLen,vector<Vec2f>& polarLinesFusion,vector<Vec4f>& pixelFusion);
    bool  frameLineDetection(Mat image);
    bool  frameLineDetection_noneImageShow(Mat frame);
    bool  imageLineDetection(string imageName);
    bool  videoLineDetection(string videoName);

    vector<Vec2f> polarLines;//是检测出的所有线段对应的极坐标表示

};
