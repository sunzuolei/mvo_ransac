# About
The code for paper Monocular Visual Odometry with RANSAC-based Outlier Rejection
##Algorithm
 Normalized eight-points algorithm is used for essential matrix estimation, and SURF descriptor is employed to extract and match feature.
<P>To improve the performance, RANSAC-based outlier rejection approach is implemented.  

The scale informaion is extracted from the KITTI dataset ground truth files.

##Demo Video
We used 487 frames of the 00th sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to test.

<P>Demo:http://v.youku.com/v_show/id_XMTI5Nzg0MDA4MA==.html


##Requirements
OpenCV 2.4.9 <p>  VS2012<p>MATLAB

##How to compile?
Before compiling the algorithm, you must configurate the VS2012 with OpenCV 2.4.9.[OpenCV Installment](http://wenku.baidu.com/link?url=EdbUVHnIIwq8ZQ8WPZD8oSMIj1f9kV5CJIrZ6X6CCAgBOFliBQp7IBl0q1Szc-1qADg1wNJTyUxPvY_YyNaMtBzrDaX3aCUpIJgFa0doBcy).<P>
You can use VS2012 to compile this algorithm, and run 'matlabload.m' by MATLAB to visualize the motion process of camera.

##Attention
Everytime before you run the algorithm, you must check whether there is the 'data.dat' in the source code folder. If the 'data.dat' exists, you must delete it before you run.
<P>In order to run this algorithm, you need to have either your own data, 
or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).
In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.And you must change to your own data file path.
##How to run? 
<p>First run the vs2012, you will get the 'data.dat' in the source code folder.
<p>Second  run 'matlabload.m' by the MATLAB to visualize the motion process of camera.






##Contact
For any question, contact: hjm@mpig.com.cn  <p>  szl@mpig.com.cn

