@ECHO OFF
REM program.exe [object name] [sequence name] [file extension for images:"jpg"] [num of particles:1] [AR param:0.0] [threshold Neff:0.1] [edgeSmoothSize:1] [cannyLow:20] [cannyHigh:40] [sampleStep:0.005] [maxD:32] [dullEdge:0] [threshold for ransac:0] [noiseRateLow:0.001] [noiseRateHigh:0.01] [detSave:"."] [threshold for FDCM:0.2]

set imgext=jpg
set N=1
set ARparam=0.0
set Neff=0.1
set edgeSmoothSize=3
set cannyLow=20
set cannyHigh=40
REM general obj > 0.005 (5mm), car door > 0.05 (5cm)
set sampleStep=0.005
::set sampleStep=0.002
REM general obj > 32, cardoor > 16
set maxD=32
::set maxD=16
::set maxD=10
REM teabox, opencvbook > 0, cardoor, starbucscup > 1
set dullEdge=0
REM 0 or 20.0
set ransacTh=0.0
::set noiseRateLow=0.001
::set noiseRateHigh=0.01
::set noiseRateLow=0.0005
::set noiseRateHigh=0.005
set noiseRateLow=0.0001
set noiseRateHigh=0.0005
::set noiseRateLow=0.003
::set noiseRateHigh=0.003

set destSave=.


REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)===============
REM N(100)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============

REM set destSave=G:\IJRR11_exp\03182011_icra10
REM set destSave=G:\IJRR11_exp\03222011_icra10
REM set destSave=G:\IJRR11_exp\03222011_icra10b
REM set destSave=G:\IJRR11_exp\03222011_icra10c
REM set destSave=G:\IJRR11_exp\03222011_icra10d
set destSave=G:\IJRR11_exp\03232011_icra10a

REM tex_box
set obj=tea_box
set seq1=seq_%obj%
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)===============
set N=1
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set ARparam=0.5
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=1
set ARparam=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.5
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%



REM opencvbook
set obj=opencvbook
set seq1=seq_%obj%
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)===============
set N=1
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set ARparam=0.5
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=1
set ARparam=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.5
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%



REM starbucks_cup
set obj=starbucks_cup
set seq1=seq_%obj%
set dullEdge=1
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)===============
set N=1
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set ARparam=0.5
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=1
set ARparam=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.5
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%



REM GMcardoor
set obj=GMcardoor
set seq1=seq_%obj%
set dullEdge=0
::set sampleStep=0.02
set sampleStep=0.05
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)===============
set N=1
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set Neff=0.0
set edgeSmoothSize=1
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set ARparam=0.5
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(1)_AR(0.0)_Neff(0.0)_smooth(3)_ransac(0)_noise(0.001,0.01)=============
set N=1
set ARparam=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.0
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)_noise(0.001,0.01)=============
set N=100
set ARparam=0.5
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%


