Kinect Stream Saver- README

>> This application is developed based on a sample called "Kinect Explorer - D2D C++" developed by Microsoft corporation. (Compatible with SDK 1.8)
>> Noncommercial — You may not use this application for commercial purposes.  
>> ©Elham Dolatabadi, Intelligent Assistive Technology and Systems Lab (IATSL), University of Toronto, Toronto Rehab.
-----------------------------------------------------------------------------------------------

To collect data with the Kinect Stream Saver:

1) Run KinectExplorer-D2D.exe:

- In the lower right corner of the window navigate to the 4th tab, labeled Recording.
- Enter subject's name if desired.
- Choose which data streams to collect.
- Press Capture to begin data collection.
- Press Stop to end data collection.

2) The collected data will appear in C:\KinectData.

- LiTimeStamp.binary:
  This file has fields: counter#, frame#, timestamp
  - The counter starts at 1 when the Capture button is pressed and data collection begins.
  - The frame number is from the Kinect API; it starts when KinectExplorer-D2D.exe is launched.
  - The timestamp is in milliseconds, and is useful for recording frame rate. (At present data
    is collected at approximately 30 frames/second, so frames are approx 33 msecs apart.)

- Color images are saved as .bmp files, on each frame.
	. as .bmp files on each frame. 
	. as "binary" data stored in a .binary file. 

- Depth images are saved either 
	. as .bmp files on each frame. 
	. as "binary" data stored in a .binary file. 

- Skeleton data:
  The program can track up to six skeletons at once. For the first four, all joints are tracked. 
  For the last two, only the center of mass is tracked. 
  
  Saving The Joint Positions: one data files is created, named  Joint_Position.binary.

  Saving The Joint Orientations: one data files are created, named  Joint_Orintations.binary.

  One hundred Twenty rows are written to each skeleton data file for each frame. These 120 rows store
  data for the 20 skeleton joints in the following order for 6 tracked users.

	CoM Position	
	HIP_CENTER	
	SPINE		
	SHOULDER_CENTER
	HEAD		
	SHOULDER_LEFT
	ELBOW_LEFT	
	WRIST_LEFT	
	HAND_LEFT	
	SHOULDER_RIGHT
	ELBOW_RIGHT 	
	WRIST_RIGHT	
	HAND_RIGHT	
	HIP_LEFT	
	KNEE_LEFT	
	ANKLE_LEFT	
	FOOT_LEFT	
	HIP_RIGHT	
	KNEE_RIGHT	
	ANKLE_RIGHT	
	FOOT_RIGHT	

 --- "Joint_Position_....binary" File --- 
     
     Each row contains following fields: 
   	- position (x, y, z); joint tracking state (in joint rows)*    	

 * Tracking states: 0=joint is not tracked; 1=joint is inferred; 2=joint is tracked


 --- "Joint_Orientation_....binary" File --- 

     Each row contains following fields: 
    	- hierarchical joint angle as a quaternion (w, x, y, z),
    	- angle status*

 * Angle status:
  - in joint rows: 0=successful, any other value implies angle computation failed  
                   and the angle has been set to the Identity quaternion (1,0,0,0).
  - in CoM rows: Not applicable. Angle status is set to S_FALSE=1.


  Caveats:
  i) If fewer than 6 skeletons are tracked, the data may appear as zeros for all positions in all frames.
 
  ii) If a user that is being tracked leaves the field of view and later returns, the Kinect
      does not know it is the same person, so they will be assigned a new skeleton id. This
      can mean that the person's data appears in more than one data file.

--------------------------------------------------------------------------------------------------

Implementation:

The Kinect Stream Saver application is based on the KinectExplorer-D2D (for C++) sample code. 
Modifications to the sample code are explained below.

1) Two new classes were added.

- StreamSaver:  
  This class has methods that store stream data (color, depth, skeleton) to a dynamic FIFO 
  buffer as the data is received from the Kinect device, and separately write the data from 
  the FIFO buffer to output files as described above. The disk writes are done in a separate
  thread to avoid degradation of frame processing speed during data collection.
   
- SaverViewer:
  This class implements a fourth GUI tab in the KinectExplorer application. The tab is labeled
  "Recording" and it allows users to specify which data streams should be saved to disk.

2) A few classes in the KinectExplorer sample code were modified. The changes were kept to a
minimum, but were necessary to invoke the functionality of the new classes.

- KinectWindow
- NuiColorStream
- NuiDepthStream
- NuiSkeletonStream

3) A new GUI "Recording" tab was designed and included in the application resources. This 
included modifications to the following files.

- KinectExplorer.rc
- resource.h

--------------------------------------------------------------------------------------------------

Instructions to open "Joint_Position_....binary" in MATLAB:

fid = fopen('C:\KinectData\Skel\liTimeStamp.binary');
B = fread(fid,'int64');
fclose(fid);

n = 4; % No. of columns of T
BB = reshape(B, n,[]);
T = permute(BB,[2,1]);

fid = fopen('C:\KinectData\Skel\Joint_Position.binary');
A = fread(fid,'float');
fclose(fid);
A(A>5)= 0;
A(A<-5)= 0;
jointNumber = 20;
tracks = 6;
AAAA = reshape(A, 4,jointNumber,tracks,[]);
sensor.Skeleton = permute(AAAA,[1,2,4,3]);

--------------------------------------------------------------------------------------------------

Acknowledgement:

Special thanks to Marge Coahran for all the help with putting together the README file !

--------------------------------------------------------------------------------------------------

Terms of Use: 

If your research benefits from the Kinect Stream Saver application provided by IATSL, please consider the following Terms of Use:

- please cite the followign paper in any publications about your work:

	* Elham Dolatabadi, Babak Taati, Gemma S. Parra-Dominguez, Alex Mihailidis, “A markerless motion tracking approach to 	understand changes in gait and balance: A Case Study”, the Rehabilitation Engineering and Assistive Technology Society of 	North America (RESNA 2013 Annual Conference). Student Scientific Paper Competition Winner


- please acknowledge this application in any publications about your work. Thank you for your support.

	Acknowledgement Text Examples:

	We would like to acknowledge Intelligent Assisstive Technology and Systems Lab for the application that facilitated this research, including the Kinect Stream Saver application developed by Elham Dolatabadi

--------------------------------------------------------------------------------------------------	

Questions?

Please contact Elham Dolatabadi, at elham.dolatabadi@mail.utoronto.ca and +1 (416) 597-3422 x7391



