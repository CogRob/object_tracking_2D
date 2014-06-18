@ECHO OFF
REM program.exe [object name] [sequence name] [file extension for images:"jpg"] [num of particles:1] [AR param:0.0] [threshold Neff:0.1] [edgeSmoothSize:1] [cannyLow:20] [cannyHigh:40] [sampleStep:0.005] [maxD:32] [dullEdge:0] [threshold for ransac:0] [noiseRateLow:0.001] [noiseRateHigh:0.01] [detSave:"."] [threshold for FDCM:0.2]

set imgext=jpg
set N=1
set ARparam=0.0
set Neff=0.1
set edgeSmoothSize=1
set cannyLow=20
set cannyHigh=40
REM general obj > 0.005 (5mm), car door > 0.05 (5cm)
::set sampleStep=0.005
set sampleStep=0.002
REM general obj > 32, cardoor > 16
set maxD=32
REM teabox, opencvbook > 0, cardoor, starbucscup > 1
set dullEdge=0
REM 0 or 20.0
set ransacTh=0.0
set destSave=.
set noiseRateLow=0.0005
set noiseRateHigh=0.005


REM N(1)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)=================================
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================


REM set destSave=G:\IJRR11_exp\03182011_synth
REM set destSave=G:\IJRR11_exp\03202011_synth
REM set destSave=G:\IJRR11_exp\03212011_synth
REM set destSave=G:\IJRR11_exp\03222011_synth_origAR
REM set destSave=G:\IJRR11_exp\03242011_synth_origAR
REM set destSave=G:\IJRR11_exp\03302011_real


REM tea_box
set obj=air_foil
set seq1=seq_%obj%_ijrr11_01
set seq2=seq_%obj%_ijrr11_02
set seq3=seq_%obj%_ijrr11_03
REM N(1)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=0.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=20.0
::edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%


