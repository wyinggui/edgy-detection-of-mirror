#include "line_find.h"

using namespace std;
using namespace cv;

LineFind::LineFind()
{}

bool  LineFind::lineFilter(vector<Vec4f>& lines_std,vector<float>& lines_dis,float thresh)
{
    int polar_num = lines_std.size();

    if(polar_num == 0)
    {
        return false;
    }

   lines_dis.clear();
   vector<Vec4f>::iterator itor2;
    for (vector<Vec4f>::iterator iter=lines_std.begin(); iter!=lines_std.end();)
    {
        Vec4f p=*iter;
        float p_x=fabs(p[0]-p[2]);
        float p_y=fabs(p[1]-p[3]);
        float p_dis=sqrt(p_x*p_x+p_y*p_y);

        if(p_dis<thresh)
        {
            itor2=iter;
            lines_std.erase(itor2);
        }
        else
        {
            iter ++ ;
            lines_dis.push_back(p_dis);
        }
    }

    polar_num = lines_std.size();
    if(polar_num == 0)
    {
        return false;
    }

    return true;
}


//p[0] u1 p[1] v1
//p[2] u2 p[3] v2
//Vec2f  LineFind::getPolarLine(Vec4f p,bool mark,float thresh)
Vec2f  LineFind::getPolarLine(Vec4f p)
{
//    mark = true;
//    float p_x=fabs(p[0]-p[2]);
//    float p_y=fabs(p[1]-p[3]);
//    float p_dis=sqrt(p_x*p_x+p_y*p_y);

//    if(p_dis<thresh)
//    {
//        mark = false;
//        return Vec2f(0,0);
//    }


    if(fabs(p[0]-p[2]) < 1e-5 )//垂直直线
    {
        if(p[0] > 0)
            return Vec2f(p[0],0);
        else
            return Vec2f(p[0],CV_PI);
    }

    if(fabs(p[1]-p[3]) < 1e-5 ) //水平直线
    {
        if(p[1] > 0)
            return Vec2f(p[1],CV_PI/2);
        else
            return Vec2f(p[1],3*CV_PI/2);
    }

    float k = (p[1]-p[3])/(p[0]-p[2]);
    float y_intercept =  p[1] - k*p[0];

    float theta;

    if( k < 0 && y_intercept > 0)
        theta =  atan(-1/k);
    else if( k > 0 && y_intercept > 0)
        theta = CV_PI + atan(-1/k);
    else if( k< 0 && y_intercept < 0)
        theta = CV_PI + atan(-1/k);
    else if( k> 0 && y_intercept < 0)
        theta = 2*CV_PI +atan(-1/k);

    float _cos = cos(theta);
    float _sin = sin(theta);

    float r = p[0]*_cos + p[1]*_sin;

    return Vec2f(r,theta);

}


vector<Vec2f> LineFind::vecPolarLine(vector<Vec4f> lines_std)
{
    vector<Vec2f> polarLines;

    for(int i=0; i<(int)lines_std.size();i++)
    {
        Vec4f p=lines_std[i];
        Vec2f pp=getPolarLine(p);
        polarLines.push_back(pp);
    }

    return polarLines;
}



bool LineFind::getIndexWithPolarLine(vector<int>& _index,vector<Vec2f> polarLines)
{
    int polar_num = polarLines.size();

    if(polar_num == 0)
    {
        return false;
    }

    _index.clear();
    _index.resize(polar_num);

    //初始化标签号
    for (int i=0; i < polar_num;i++)
        _index[i] = i;


    for (int i=0; i < polar_num-1 ;i++)
    {

        float minTheta = CV_PI;
        float minR = 50;
        Vec2f polar1 = polarLines[i];

        for (int j = i+1; j < polar_num; j++)
        {

            Vec2f polar2 = polarLines[j];

            float dTheta = fabs(polar2[1] - polar1[1]);
            float dR = fabs(polar2[0] - polar1[0]);

            if(dTheta < minTheta )
                minTheta = dTheta;

            if(dR < minR)
                minR = dR;
            //同类直线角度误差不超过1.8°，距离误差不超过8%
            if(dTheta < 1.8*CV_PI/180 && dR < polar1[0]*0.08)
                _index[j] = _index[i];
        }
    }

    return true;
}



//vector<Vec4f>::iterator itor2;
// for (vector<Vec4f>::iterator iter=lines_std.begin(); iter!=lines_std.end();)
// {
//     Vec4f p=*iter;
//     float p_x=fabs(p[0]-p[2]);
//     float p_y=fabs(p[1]-p[3]);
//     float p_dis=sqrt(p_x*p_x+p_y*p_y);

//     if(p_dis<thresh)
//     {
//         itor2=iter;
//         lines_std.erase(itor2);
//     }
//     else
//     {
//         iter ++ ;
//         lines_dis.push_back(p_dis);
//     }
// }

void LineFind::indexSort(vector<int> _index,vector<vector<int>> &indexContent,vector<int> &indexLen)
{
    //indexContent.clear();
    //indexLen.clear();

    vector<int> temp1;

    for(int i=0; i<(int)_index.size();i++)
    {
        indexContent[_index[i]].push_back(i);
        indexLen[_index[i]]=indexLen[_index[i]]+1;
    }

    vector<vector<int>>::iterator itor2;
    vector<vector<int>>::iterator iter=indexContent.begin();
    vector<int>::iterator itor22;
    for(vector<int>::iterator iter2=indexLen.begin(); iter2!=indexLen.end();)
    {
        if(*iter2==0)
        {
            itor22=iter2;
            indexLen.erase(itor22);
            itor2=iter;
            indexContent.erase(itor2);
        }
        else {
            iter++;
            iter2++;
        }
    }

}


void  LineFind::lineDisFilter(vector<Vec4f> lines_filter,vector<float> lines_dis,vector<Vec2f> polarLines,
                              vector<vector<Vec4f>>& lines_filter_shim_all,vector<vector<Vec2f>>& polarLines_shim_all,
                              vector<vector<float>>& lines_dis_shim_all,vector<float>& lines_dis_sum,
                              vector<vector<int>>& indexContent,vector<int>& indexLen,float thresh)
{
    vector<vector<int>>::iterator itor2;
    vector<vector<int>>::iterator iter=indexContent.begin();
    vector<int>::iterator itor22;
    vector<int> temp1;
    float sumDis;
    vector<float> temp2;
    vector<Vec2f> temp3;
    vector<Vec4f> temp4;
    for(vector<int>::iterator iter2=indexLen.begin(); iter2!=indexLen.end();)
    {
        sumDis=0;
        temp1=*iter;

        temp2.clear();
        temp3.clear();
        temp4.clear();
        for(int i=0;i<temp1.size();i++)
        {
            temp4.push_back(lines_filter[temp1[i]]);
            temp3.push_back(polarLines[temp1[i]]);
            temp2.push_back(lines_dis[temp1[i]]);
            sumDis=sumDis+lines_dis[temp1[i]];
        }

        if(sumDis<thresh)
        {
            itor22=iter2;
            indexLen.erase(itor22);
            itor2=iter;
            indexContent.erase(itor2);
        }
        else {
            lines_filter_shim_all.push_back(temp4);
            polarLines_shim_all.push_back(temp3);
            lines_dis_sum.push_back(sumDis);
            lines_dis_shim_all.push_back(temp2);
            iter++;
            iter2++;
        }
    }
}

void  LineFind::lineFusion(Vec2f imageSize, vector<vector<Vec4f>> lines_filter_shim_all,vector<vector<Vec2f>> polarLines_shim_all,
                            vector<vector<float>> lines_dis_shim_all,vector<float> lines_dis_sum,vector<vector<int>> indexContent,
                            vector<int> indexLen,vector<Vec2f>& polarLinesFusion,vector<Vec4f>& pixelFusion)
{
    if(((int)indexLen.size())>0)
    {
        vector<int> temp1;
        Vec2f temp2;
        vector<float> temp3;
        vector<Vec2f> temp4;
        float temp5;
        Vec4f temp6;
        for(int i=0;i<indexLen.size();i++)
        {
//            temp1=indexContent[i];
            temp2[0]=0;
            temp2[1]=0;
            temp3=lines_dis_shim_all[i];
            temp4=polarLines_shim_all[i];
            for(int j=0; j<indexLen[i];j++)
            {
                temp5=temp3[j]/lines_dis_sum[i];
                temp2[0]=temp2[0]+temp5*temp4[j][0];
                temp2[1]=temp2[1]+temp5*temp4[j][1];
            }
            polarLinesFusion.push_back(temp2);

            float _cos=cos(temp2[1]);
            float _sin=sin(temp2[1]);
            if(temp2[1]==0)
            {
                temp6[0]=temp2[1];
                temp6[1]=0;
                temp6[2]=temp2[1];
                temp6[3]=imageSize[1];
            }
            else if (temp2[1]==CV_PI/2) {
                temp6[0]=0;
                temp6[1]=temp2[1];
                temp6[2]=imageSize[0];
                temp6[3]=temp2[1];
            }
            else {
                int mark[4]={0,0,0,0};
//                float *x= new float[4];
//                float *y= new float[4];
                float x[4]= {0,0,0,0};
                float y[4]= {0,0,0,0};
                x[0]=0;
                y[0]=temp2[0]/_sin;
                x[1]=temp2[0]/_cos;
                y[1]=0;
                x[2]=imageSize[0];
                y[2]=(temp2[0]-x[2]*_cos)/_sin;
                y[3]=imageSize[1];
                x[3]=(temp2[0]-y[3]*_sin)/_cos;
                if(y[0]>=0&y[0]<=imageSize[1])
                    mark[0]=1;
                if(x[1]>=0&x[1]<=imageSize[0])
                    mark[1]=1;
                if(y[2]>=0&y[2]<=imageSize[1])
                    mark[2]=1;
                if(x[3]>=0&x[3]<=imageSize[0])
                    mark[3]=1;
                int jj=0;
                for(int ii=0;ii<4;ii++)
                {
                    if(mark[ii]==1)
                    {
                        temp6[jj++]=x[ii];
                        temp6[jj++]=y[ii];

                    }
                }

//                delete[] x;
//                delete[] y;
            }

            pixelFusion.push_back(temp6);
        }
    }
}



bool  LineFind::imageLineDetection(string imageName)
{
    Mat image = imread(imageName, IMREAD_GRAYSCALE);
    bool mark=frameLineDetection(image);
    return mark;
}

bool  LineFind::frameLineDetection(Mat image)
{
    Size dsize(720,480);
//    Size dsize(80,60);
    resize(image, image, dsize);
    Mat image1=image.clone();
    //int temp2=image.col();
#if 1
    Canny(image, image, 50, 200, 3); // Apply canny edge
    imshow("edge", image);
#endif

    int rows=image.rows;
    int cols=image.cols;
    Vec2f imageSize;
    imageSize[0]=cols;
    imageSize[1]=rows;

    // Create and LSD detector with standard or no refinement.
#if 1
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
#else
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
#endif

    vector<Vec4f> lines_std;
    // Detect the lines
    ls->detect(image, lines_std);

    // Show found lines
    Mat drawnLines(image);
    ls->drawSegments(drawnLines, lines_std);
    imshow("Standard refinement", drawnLines);

    //  LineFind linefind;
    vector<Vec4f> lines_filter = lines_std;
    float thresh=(float)cols<rows ? 0.1*cols : 0.1*rows;
    vector<float> lines_dis;
    bool mark=lineFilter(lines_filter,lines_dis,thresh);
    if(mark==false)
        return false;
    bool mark1=false;
    vector<Vec2f> polarLines;
    vector<int> _index;

    Mat drawnLinesFilter(image);
    ls->drawSegments(drawnLinesFilter, lines_filter);
    imshow("Standard refinement2", drawnLinesFilter);
    polarLines=vecPolarLine(lines_filter);
    mark1=getIndexWithPolarLine(_index,polarLines);

    vector<vector<int>> indexContent((int)_index.size());
    vector<int> indexLen((int)_index.size());
    indexSort(_index,indexContent,indexLen);

    thresh=(float)cols<rows ? 0.6*cols : 0.6*rows;
    vector<float> lines_dis_sum;
    vector<vector<float>> lines_dis_shim_all;
    vector<vector<Vec4f>> lines_filter_shim_all;
    vector<vector<Vec2f>> polarLines_shim_all;
    lineDisFilter(lines_filter,lines_dis,polarLines,lines_filter_shim_all,polarLines_shim_all,lines_dis_shim_all,
                           lines_dis_sum,indexContent,indexLen,thresh);
    if(((int)indexLen.size())==0)
        return false;
    vector<Vec2f> polarLinesFusion;
    vector<Vec4f> pixelFusion;
    lineFusion(imageSize,lines_filter_shim_all,polarLines_shim_all,lines_dis_shim_all,lines_dis_sum,indexContent,
                                indexLen,polarLinesFusion,pixelFusion);
    if(((int)pixelFusion.size())==0)
        return false;
    // Show found lines
    Mat drawnLinesFusion(image1);
    ls->drawSegments(drawnLinesFusion, pixelFusion);
    imshow("Fusion Line", drawnLinesFusion);

    return true;
}

bool  LineFind::frameLineDetection_noneImageShow(Mat frame)
{
    Mat image=frame.clone();
    cvtColor(image, image, COLOR_BGR2GRAY);
    Size dsize(720,480);
//    Size dsize(480,720);
//    Size dsize(80,60);
    resize(image, image, dsize);
    Mat image1=image.clone();
    //int temp2=image.col();
//    imshow("edge", image);

//    imwrite('image.jpg', image);
//    image = imread('image.jpg', IMREAD_GRAYSCALE);
#if 1
    Canny(image, image, 50, 200, 3); // Apply canny edge
//    imshow("edge", image);
#endif

    int rows=image.rows;
    int cols=image.cols;
    Vec2f imageSize;
    imageSize[0]=cols;
    imageSize[1]=rows;

    // Create and LSD detector with standard or no refinement.
#if 1
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
#else
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
#endif

    vector<Vec4f> lines_std;
    // Detect the lines
    ls->detect(image, lines_std);

//    // Show found lines
//    Mat drawnLines(image);
//    ls->drawSegments(drawnLines, lines_std);
//    imshow("Standard refinement", drawnLines);

    //  LineFind linefind;
    vector<Vec4f> lines_filter = lines_std;
    float thresh=(float)cols<rows ? 0.2*cols : 0.2*rows;
    vector<float> lines_dis;
    bool mark=lineFilter(lines_filter,lines_dis,thresh);
    if(mark==false)
    {
//        cvtColor(image1, image1, COLOR_GRAY2BGR);
        imshow("Fusion Line", image1);
        return false;
    }
    bool mark1=false;
    vector<Vec2f> polarLines;
    vector<int> _index;

//    Mat drawnLinesFilter(image);
//    ls->drawSegments(drawnLinesFilter, lines_filter);
//    imshow("Standard refinement2", drawnLinesFilter);
    polarLines=vecPolarLine(lines_filter);
    mark1=getIndexWithPolarLine(_index,polarLines);

    vector<vector<int>> indexContent((int)_index.size());
    vector<int> indexLen((int)_index.size());
    indexSort(_index,indexContent,indexLen);

    thresh=(float)cols<rows ? 1.5*cols : 1.5*rows;
    vector<float> lines_dis_sum;
    vector<vector<float>> lines_dis_shim_all;
    vector<vector<Vec4f>> lines_filter_shim_all;
    vector<vector<Vec2f>> polarLines_shim_all;
    lineDisFilter(lines_filter,lines_dis,polarLines,lines_filter_shim_all,polarLines_shim_all,lines_dis_shim_all,
                           lines_dis_sum,indexContent,indexLen,thresh);
    if(((int)indexLen.size())==0)
    {
//        cvtColor(image1, image1, COLOR_GRAY2BGR);
        imshow("Fusion Line", image1);
        return false;
    }
    vector<Vec2f> polarLinesFusion;
    vector<Vec4f> pixelFusion;
    lineFusion(imageSize,lines_filter_shim_all,polarLines_shim_all,lines_dis_shim_all,lines_dis_sum,indexContent,
                                indexLen,polarLinesFusion,pixelFusion);
    if(((int)pixelFusion.size())==0)
    {
//        cvtColor(image1, image1, COLOR_GRAY2BGR);
        imshow("Fusion Line", image1);
        return false;
    }
    // Show found lines
//    cvtColor(image1, image1, COLOR_GRAY2BGR);
    Mat drawnLinesFusion(image1);
//    Mat drawnLinesFusion(image);
    ls->drawSegments(drawnLinesFusion, pixelFusion);
    imshow("Fusion Line", drawnLinesFusion);

    return true;
}


bool  LineFind::videoLineDetection(string videoName)
{
    //string video_filename = "vtest.avi";
    VideoCapture vc;
    Mat frame;
    vc.open(videoName);//videoName.c_str()为了与c语言兼容，在c语言中没有string类型，故必须通过string类对象的成员函数c_str()把string 对象转换成c中的字符串样式
    if (!vc.isOpened())
        throw runtime_error(string("can't open video file: " + videoName));

    CvCapture *capture;
    capture = cvCreateFileCapture("//home//wyinggui//catkin_ws//lineDtection//2.MP4");
    int frameH = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
    int frameW = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
    int fps = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
    int numFrames = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);
    printf("\tvideo height : %d\n\tvideo width : %d\n\tfps : %d\n\tframe numbers : %d\n", frameH, frameW, fps, numFrames);
    cout<<frameH<<"  "<<frameW<<"  "<<fps<<"  "<<numFrames<<endl;

    for (int i=0;;i++)
    {
        vc >> frame;
        cout << i << endl;
        if (frame.empty())
            continue;
        frameLineDetection_noneImageShow(frame);
/*        namedWindow("frame");
        imshow("frame", frame)*/;
        int c = waitKey(vc.isOpened() ? 10 : 0) & 255;
//        if (c == 'q' || c == 'Q' || c == 27)
//            break;
//        /*while (waitKey(10) != 27); destroyWindow("frame");*/
    }

}
























