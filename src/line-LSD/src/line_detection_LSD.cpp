#include <iostream>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "line_find.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    string in;
    if (argc != 2)
    {
        cout << "Usage: lsd_lines [input image]. Now loading ../data/building.jpg" << endl;
        in = "../data/building.jpg";
    }
    else
    {
        in = argv[1];
    }

    LineFind linefind;
    bool mark=false;

    double start = double(getTickCount());

//    mark=linefind.imageLineDetection(in);
    mark=linefind.videoLineDetection(in);
    double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
    std::cout << "It took " << duration_ms << " ms." << std::endl;

    if(mark==true)
    {
        cout << "Detecting lines are successful!" << endl;
    }
    else
    {
        cout << "Detecting lines are unsuccessful!" << endl;
    }
    waitKey();
    return 0;
}



























//    Mat image = imread(in, IMREAD_GRAYSCALE);

//    Size dsize(720,480);
//    resize(image, image, dsize);
//    Mat image1=image.clone();
//    //int temp2=image.col();
//#if 1
//    Canny(image, image, 50, 200, 3); // Apply canny edge
//    imshow("edge", image);
//#endif

//    int rows=image.rows;
//    int cols=image.cols;
//    Vec2f imageSize;
//    imageSize[0]=cols;
//    imageSize[1]=rows;
////    imageSize[0]=rows;
////    imageSize[1]=cols;
//    // Create and LSD detector with standard or no refinement.
//#if 1
//    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
//#else
//    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
//#endif

//    double start = double(getTickCount());
//    vector<Vec4f> lines_std;

//    // Detect the lines
//    ls->detect(image, lines_std);

//    double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
//    std::cout << "It took " << duration_ms << " ms." << std::endl;

//    // Show found lines
//    Mat drawnLines(image);
//    ls->drawSegments(drawnLines, lines_std);
//    imshow("Standard refinement", drawnLines);

//    LineFind linefind;
//    vector<Vec4f> lines_filter = lines_std;
//    float thresh=(float)cols<rows ? 0.1*cols : 0.1*rows;
//    vector<float> lines_dis;
//    bool mark=linefind.lineFilter(lines_filter,lines_dis,thresh);

//    bool mark1=false;
//    vector<Vec2f> polarLines;
//    vector<int> _index;

//    if(mark==true)
//    {
//        Mat drawnLinesFilter(image);
//        ls->drawSegments(drawnLinesFilter, lines_filter);
//        imshow("Standard refinement2", drawnLinesFilter);

//        polarLines=linefind.vecPolarLine(lines_filter);

//        mark1=linefind.getIndexWithPolarLine(_index,polarLines);


//    }

//    vector<vector<int>> indexContent((int)_index.size());
//    vector<int> indexLen((int)_index.size());
//    linefind.indexSort(_index,indexContent,indexLen);

//    thresh=(float)cols<rows ? 0.6*cols : 0.6*rows;
//    vector<float> lines_dis_sum;
//    vector<vector<float>> lines_dis_shim_all;
//    vector<vector<Vec4f>> lines_filter_shim_all;
//    vector<vector<Vec2f>> polarLines_shim_all;
//    linefind.lineDisFilter(lines_filter,lines_dis,polarLines,lines_filter_shim_all,polarLines_shim_all,lines_dis_shim_all,
//                           lines_dis_sum,indexContent,indexLen,thresh);
//    vector<Vec2f> polarLinesFusion;
//    vector<Vec4f> pixelFusion;
//    linefind.lineFusion(imageSize,lines_filter_shim_all,polarLines_shim_all,lines_dis_shim_all,lines_dis_sum,indexContent,
//                                indexLen,polarLinesFusion,pixelFusion);

//    // Show found lines
//    Mat drawnLinesFusion(image1);
//    ls->drawSegments(drawnLinesFusion, pixelFusion);
//    imshow("Fusion Line", drawnLinesFusion);

//    if(mark1==true)
//    {}
