#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/core/core.hpp"
#include "highgui.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "time_measurement.hpp"

#include "pointstruct.h"
#include "matrix.h"
#include"visomono.h"
using namespace cv;
using namespace std;

//before you run the algorithm with you own data
//you should change 3 places,which are 'image file path','the intrinsic calibration parameters'and'The ground true trajectory file'

//get the scale from ground true trajectory
double getAbsoluteScale(int frame_id, int sequence_id, double z_cal);

//write the motion data in file
int abWrite(const Mat &im, const string &fname);
void record_txt(Matrix pose,const string &fname);
// monocular visual odometry process algorithm function
bool run(Mat &img2,int &number_frame,Matrix &P0);

	
	vector<KeyPoint> keypoints2, keypoints1;
		// Prepare the matcher
		SurfFeatureDetector detector;
		SurfDescriptorExtractor extractor;
		Mat descriptors1, descriptors2;
		
void main()
{
		 struct timeval tic;
		struct timeval toc;

	  string dir = "E://Samuel//Kylin//RawData//KITTI//dataset//sequences//00//left//"; //image file path
	  Mat img1=imread("E://Samuel//Kylin//RawData//KITTI//dataset//sequences//00//left//000000.png");
	  // VideoCapture sequence(first_file);
	  Matrix P0 = Matrix::eye(4);
	detector.detect(img1, keypoints1);
		extractor.compute(img1, keypoints1, descriptors1);


	int number_frame=1;
	int Max_number_frame=1000;//根据图片数据的数量修改
	int number_data =0;
	
	while (number_frame<Max_number_frame)
	
	{
		char base_name[256];
		sprintf(base_name,"%06d.png",number_frame);
    string left_img_file_name  = dir +base_name;
	 
		
		
		cout<<"*****************************Read "<<number_frame<<"th image*****************************"<<endl;
		Mat img2 = imread(left_img_file_name);
			
		
		 //cvtColor(img2,img2,CV_BGR2GRAY);
		gettimeofday( &tic, 0 );
		if( run(img2,number_frame,P0))
		{
			cout<<"============================Save "<<++number_data<<"th data===============================\n\n\n\n";
			
			//save the motion_estimation data for matlab visualization
			//abWrite(Ptemp1, "./data.dat"); 
		}
		++number_frame;
		gettimeofday( &toc, 0 );
		double _time = TIMETODOUBLE(timeval_minus(toc,tic)); 
       cout <<"运行时间"<<_time <<endl;

	}

 
}



	bool run(Mat &img2,int &number_frame,Matrix &P0)
	{

		
		
		if(img2.empty())
		{
			cout << "finish reading image " << endl;
			exit(0);

		}
		else 
		{
			//medianBlur(img1,img1,5);
			//medianBlur(img2,img2,5);
			
		//the intrinsic calibration parameters 
			double Ka[9] = {718.856,0,607.1928,0,718.856,185.2517,0,0,1};//相机内参 根据自己的数据改变
			detector.detect(img2, keypoints2);
							// computing descriptors
							extractor.compute(img2, keypoints2, descriptors2);
							// matching descriptors
					FlannBasedMatcher matcher;
					  vector< DMatch > matches;
					  vector<vector<DMatch>> m_knnMatches;

					  matches.clear();
					  const float minRatio=1.f / 1.5f;
					  matcher.knnMatch(descriptors1,descriptors2,m_knnMatches,2);//KNN匹配
					  cout <<"matches number="<<m_knnMatches.size()<<endl;
					  for (int i=0; i<m_knnMatches.size(); i++)
					  {
						  const DMatch& bestMatch=m_knnMatches[i][0];
						  const DMatch& betterMatch=m_knnMatches[i][1];

						  float distanceRatio=bestMatch.distance/betterMatch.distance;

						  if (distanceRatio<minRatio)
						  {
							  matches.push_back(bestMatch);
						  }
					  }

						cout <<"good matches number="<<matches.size()<<endl;
						//change the matches data structure
						vector <p_match>p_matched;
							p_match *i;
							p_match m_temp;
							i=&m_temp;
							if(NULL==i)
							{
								cout <<"内存分配失败"<<endl;
								exit(0);
							}
							
							vector<DMatch>::iterator itr;
							for (itr = matches.begin(); itr!=matches.end(); itr++)  
							{
							 (*i).u1p=keypoints1[ (*itr).queryIdx].pt.x;
							 (*i).v1p=keypoints1[ (*itr).queryIdx].pt.y;
							 (*i).u1c=keypoints2[ (*itr).trainIdx].pt.x; 
							 (*i).v1c=keypoints2[ (*itr).trainIdx].pt.y;
							 p_matched.push_back(*i);
							}
				
				vector<double> tr_delta;
				Matrix P1;
				parameters param;
				tr_delta=estimateMotion (p_matched,Ka,param);//return  the estimateMotion information
				if(updateMotion ( tr_delta,P1) )
				{
					double z_cal=sqrt(P1.val[2][3]*P1.val[2][3]+P1.val[1][3]*P1.val[1][3]+P1.val[0][3]*P1.val[0][3]);
					double scale = getAbsoluteScale(number_frame, 0,z_cal);
					
						P1.val[0][3]=P1.val[0][3]*scale;
							 P1.val[1][3]=P1.val[1][3]*scale;
							 P1.val[2][3]=P1.val[2][3]*scale;
					cout<<" P1="<<P1<<endl;

						P0 = P0 * Matrix::inv(P1);
						record_txt(P0,"./pose.txt");
						descriptors1.release();
						keypoints1.clear();
						if ( descriptors1.empty())//
						{
						descriptors2.copyTo(descriptors1);	
						keypoints1=keypoints2;
						keypoints2.clear();
						descriptors2.release();
						}
						
				return true;	//motion estimation succeed
				}
			else
				return false;
 

}
		
	}
	
	
	
double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
  
  string line;
  int i = 0;
  ifstream myfile ("00.txt");//The ground true trajectory file
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  double truth= sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
   double scale=(truth/z_cal);
   return scale;
}


int abWrite(const Mat &im, const string &fname)
{
  ofstream ouF;
  ouF.open(fname.c_str(), std::ofstream::binary | ios::app);
  if (!ouF)
  {
    cerr << "failed to open the file : " << fname << endl;
    return 0;
  }
  for (int r = 0; r < im.rows; r++)
  {
    ouF.write(reinterpret_cast<const char*>(im.ptr(r)), im.cols*im.elemSize());
  }
  ouF.close();
  return 1;
}



void record_txt(Matrix pose,const string &fname)
{
	
	fstream file(fname.c_str(),ios::out |ios::app);
	if(file.fail())
	{
		cout<<"open file failed !!"<<endl;
	}
	else
	{
		for(int i=0;i<3;++i)
			for(int j=0;j<4;++j)
			{
				file<<pose.val[i][j]<<" ";

			}
			file<<"\n";

	}
	file.close();
}
