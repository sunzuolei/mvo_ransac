#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/core/core.hpp"
#include "highgui.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"

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

// monocular visual odometry process algorithm function
bool run(Mat &img1,Mat &img2,Mat_<double> &Ptemp1,int &number_frame);


void main()
{
	
	  string first_file = "img//%06d.png"; //image file path
	  VideoCapture sequence(first_file);

	if (!sequence.isOpened())
	{
		cerr << "Failed to open the image sequence!\n" << endl;
	}

	Mat img1;
	
	int number_frame=0;
	int number_data =0;
	for(;;)
	{
		Mat_<double> Ptemp1(4,4);
		++number_frame;
		cout<<"*****************************Read "<<number_frame<<"th image*****************************"<<endl;
		Mat img2;

		sequence >> img2;
		 //cvtColor(img2,img2,CV_BGR2GRAY);
		if( run(img1,img2,Ptemp1,number_frame))
		{
			cout<<"============================Save "<<++number_data<<"th data===============================\n\n\n\n";
			
			//save the motion_estimation data for matlab visualization
			abWrite(Ptemp1, "./data.dat"); 
		}
		if(number_frame==2000) break;
		


	}

 
}



	bool run(Mat &img1,Mat &img2,Mat_<double> &Ptemp1,int &number_frame)
	{

		
		if(img1.empty())
		{
			img2.copyTo(img1);
			
			//cout << "read 1th image" << endl;
			return false;
		}
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
			double Ka[9] = {718.856,0,607.1928,0,718.856,185.2517,0,0,1};
			//double Ka[9] = {1607.96,0,1006.019,0,1605.96,578.140,0,0,1};
			//Matrix K(3,3,Ka);
			vector<KeyPoint> keypoints2, keypoints1;
				
						// Prepare the matcher
							SurfFeatureDetector detector;
							
							detector.detect(img1, keypoints1);
							detector.detect(img2, keypoints2);
							// computing descriptors
							SurfDescriptorExtractor extractor;
							Mat descriptors1, descriptors2;
							extractor.compute(img1, keypoints1, descriptors1);
							extractor.compute(img2, keypoints2, descriptors2);
							// matching descriptors
							BruteForceMatcher<L2<float>> matcher;
							vector<DMatch> matches;
							matcher.match(descriptors1, descriptors2, matches);
							   		std::vector<cv::Point2f> points1, points2;	
						//****************************************
							
						//change the matches data structure
						vector <p_match>p_matched;
							p_match *i;
							p_match m_temp;
							i=&m_temp;
						/*	if(NULL==i)
							{
								cout <<"ÄÚ´æ·ÖÅäÊ§°Ü"<<endl;
								exit(0);
							}*/
							
							vector<DMatch>::iterator itr;
							for (itr = matches.begin(); itr!= matches.end(); itr++)  
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
					double z_cal=P1.val[2][3];
					double scale = getAbsoluteScale(number_frame, 0,z_cal);
							//if ((scale>0.1)&&(P1.val[2][3] > P1.val[0][3]) && (P1.val[2][3] > P1.val[1][3]))
							//{
								
							 P1.val[0][3]=P1.val[0][3]*scale;
							 P1.val[1][3]=P1.val[1][3]*scale;
							 P1.val[2][3]=P1.val[2][3]*scale;

							///}	
			    
					cout <<"The abusolute scale is:"<<scale<<endl;
					for( int i = 0; i < Ptemp1.rows; ++i )
					  for( int j = 0; j < Ptemp1.cols; ++j )
					  {
							  Ptemp1(i,j)=P1.val[i][j];//translate the data structure for saving
					  }
					 // cout <<P1.val[0][3]<<"#"<<P1.val[0][2]<<"#"<<P1.val[0][3]<<endl;
					  //cout<<Ptemp1;
						img1.release();
						if (img1.empty())//
						{
						img2.copyTo(img1);
						
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

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

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