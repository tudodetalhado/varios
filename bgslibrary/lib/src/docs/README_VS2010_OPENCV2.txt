---------------------------------------------------
BGSLibrary with Visual Studio 2010 and Opencv 2.4.x
---------------------------------------------------
--- Tutorial for Windows x86 32 bits ---
----------------------------------------

1) Install OpenCV
1.a) Download OpenCV 2.4.x from http://opencv.org/
1.b) Install in: C:\OpenCV2.4.x
1.c) Add OpenCV binaries in your Path
C:\OpenCV2.4.x\build\x86\vc10\bin

2) Download BGSLibrary
2.a) Clone bgslibrary on GitHub at C:\bgslibrary

3) Start Visual Studio 2010
3.a) Create New Project
3.b) Select Visual C++ -> Win32 -> Win32 Console Application
3.c) Set project location: C:\bgslibrary
3.d) Set project name: bgslibrary
3.e) Set Empty project 
3.f) Add Main.cpp in [Source Files]
3.g) Add content of c:\bgslibrary\package_bgs\*.* in [Header Files]
3.h) Add content of c:\bgslibrary\package_analysis\*.* in [Header Files]
3.i) Change to [Release] [Win32] mode
3.j) Click on Project->Properties
3.k) Change [Output Directory] to ..\
3.l) Add OpenCV include in [C/C++] -> [Additional Include Directories]
C:\OpenCV2.4.x\build\include;C:\OpenCV2.4.x\build\include\opencv;
3.m) Add OpenCV libraries in [Linker]->[Input]
C:\OpenCV2.4.x\build\x86\vc10\lib\*.lib
3.n) Click in Build and wait
3.o) Run C:\bgslibrary\bgslibrary.exe
Enjoy!
