# mvo-ransac
The code for paper Monocular Visual Odometry with RANSAC-based Outlier Rejection
##Algorithm
Uses normalized eight-points algorithm for essential matrix estimation ,and SURF descriptor is employed to extract and match feature.
<P>To improve the performance, RANSAC-based outlier rejection approach is implemented.  

The scale informaion is extracted from the KITTI dataset ground truth files.

##Demo Video
we used 0-487 frames of the 00th sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to test.

<P>Demo:http://v.youku.com/v_show/id_XMTI5Nzg0MDA4MA==.html


##Requirements
OpenCV 2.4.9 <p>  VS2012<p>MATLAB

##How to compile?
Before you compile the algorithm,you must configurate the VS2012 with OpenCV 2.4.9<P>
You can use VS2012 to compile this algorithm,and use the MATLAB run 'matlabload.m' to visualize the motion process of camera.

##Attention
Everytime before you run the algorithm,you must check whether there is the 'data.dat' in the source code folder.If the 'data.dat' exists,you must delete it before you run.
<P>In order to run this algorithm, you need to have either your own data, 
or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).
In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.And you must change your own data file path.
##How to run? 
<p>First run the vs2012,you will get the 'data.dat' in the source code folder.
<p>Second use the MATLAB run 'matlabload.m' to visualize the motion process of camera.






##Contact
For any question, contact: szl@mpig.com.cn  <p>   hjm@mpig.com.cn

