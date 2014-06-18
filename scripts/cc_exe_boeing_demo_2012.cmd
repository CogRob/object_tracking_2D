@ECHO OFF
REM program.exe [object name] [sequence name] [file extension for images:"jpg"] [num of particles:1] [AR param:0.0] [threshold Neff:0.1] [edgeSmoothSize:1] [cannyLow:20] [cannyHigh:40] [sampleStep:0.005] [maxD:32] [dullEdge:0] [threshold for ransac:0] [noiseRateLow:0.001] [noiseRateHigh:0.01] [detSave:"."] [threshold for FDCM:0.2]

set imgext=jpg
set N=1
set ARparam=0.0
set Neff=0.1
REM 0: bilateral filtering, 1 or bigger: Gaussian smoothing cvSmooth() with the filter size
set edgeSmoothSize=3
::set cannyLow=20
::set cannyHigh=40
set cannyLow=40
set cannyHigh=80
REM general obj > 0.005 (5mm), car door > 0.05 (5cm)
set sampleStep=0.005
::set sampleStep=0.002
::set sampleStep=0.01
REM general obj > 32, cardoor > 16
set maxD=16
REM teabox, opencvbook > 0, cardoor, starbucscup > 1
set dullEdge=0
REM 0 or 20.0
set ransacTh=0.0
set destSave=.
::set noiseRateLow=0.0005
::set noiseRateHigh=0.005
::set noiseRateHigh=0.001

set noiseRateLow=0.0005
set noiseRateHigh=0.005

REM set destSave=G:\IJRR11_exp\03182011_synth
REM set destSave=G:\IJRR11_exp\03202011_synth
REM set destSave=G:\IJRR11_exp\03212011_synth
REM set destSave=G:\IJRR11_exp\03222011_synth_origAR
REM set destSave=G:\IJRR11_exp\03242011_synth_origAR
REM set destSave=G:\IJRR11_exp\03302011_real
REM set destSave=G:\CVPR12_exp\10142011_real
REM set destSave=G:\CVPR12_exp\10172011_real
REM set destSave=G:\CVPR12_exp\10192011_real
REM set destSave=G:\CVPR12_exp\10202011_real
REM set destSave=G:\CVPR12_exp\10252011_real
REM set destSave=G:\CVPR12_exp\10282011_real
REM set destSave=G:\CVPR12_exp\10292011_real_maxd16


REM Object
set obj=boeing_wing_pannel
set seq=seq_camera

::set Neff=0.1
::set Neff=0.0
set Neff=-0.1
set Th_FDCM=0.16
set Neff_reinit=0.1
set Th_FDCM_reinit=0.09
set limitYRotation=0

::set N=50
set N=30
::set N=10
::set N=15
::set N=20
::set N=1
::set ARparam=0.5
set ARparam=0.0
set ransacTh=20.0
set numOfAnealingLayers=5
::set numOfAnealingLayers=1

edgeBasedTracker.exe %obj% %seq% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
