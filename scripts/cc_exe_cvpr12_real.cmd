@ECHO OFF
REM program.exe [object name] [sequence name] [file extension for images:"jpg"] [num of particles:1] [AR param:0.0] [threshold Neff:0.1] [edgeSmoothSize:1] [cannyLow:20] [cannyHigh:40] [sampleStep:0.005] [maxD:32] [dullEdge:0] [threshold for ransac:0] [noiseRateLow:0.001] [noiseRateHigh:0.01] [detSave:"."] [threshold for FDCM:0.2]

set imgext=jpg
set N=1
set ARparam=0.0
set Neff=0.1
set edgeSmoothSize=0 REM 0: bilateral filtering, 1 or bigger: Gaussian smoothing cvSmooth() with the filter size
set cannyLow=20
set cannyHigh=40
REM general obj > 0.005 (5mm), car door > 0.05 (5cm)
::set sampleStep=0.005
set sampleStep=0.002
REM general obj > 32, cardoor > 16
set maxD=16
REM teabox, opencvbook > 0, cardoor, starbucscup > 1
set dullEdge=1
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
REM set destSave=G:\CVPR12_exp\10142011_real
REM set destSave=G:\CVPR12_exp\10172011_real
REM set destSave=G:\CVPR12_exp\10192011_real
REM set destSave=G:\CVPR12_exp\10202011_real
REM set destSave=G:\CVPR12_exp\10252011_real
REM set destSave=G:\CVPR12_exp\10282011_real
set destSave=G:\CVPR12_exp\10292011_real_maxd16


REM IKEA_FARGRIK
set obj=IKEA_FARGRIK
set seq1=seq_%obj%_rimdesk2_gen01
set seq2=seq_%obj%_rimdesk2_gen02
set seq3=seq_%obj%_rimdesk2_gen03
set seq4=seq_%obj%_rimdesk2_hand01
set seq5=seq_%obj%_rimdesk2_hand02
set seq6=seq_%obj%_rimdesk2_hand03
set seq7=seq_%obj%_rimdesk2_reinit01
set seq8=seq_%obj%_rimdesk2_reinit02
set seq9=seq_%obj%_rimdesk2_reinit03

set seq10=seq_%obj%_rimdesk2_hand04

set Neff=0.0
set Th_FDCM=0.16
set Neff_reinit=0.1
set Th_FDCM_reinit=0.09
set limitYRotation=1

REM N(1)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.5
set ransacTh=0.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

REM reinit
::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%


REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set ransacTh=0.0
set numOfAnealingLayers=5


REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set ransacTh=20.0
set numOfAnealingLayers=5


REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set ransacTh=0.0
set numOfAnealingLayers=5
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=5
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

REM reinit
::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)_limitYRotation(false)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=5
set limitYRotation=0
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
set limitYRotation=1

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)numOfAnealingLayers(1)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=1
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
set numOfAnealingLayers=5




REM SmallGlass_800
set obj=SmallGlass_800
set seq1=seq_%obj%_rimdesk2_gen01
set seq2=seq_%obj%_rimdesk2_gen02
set seq3=seq_%obj%_rimdesk2_gen03
set seq4=seq_%obj%_rimdesk2_hand01
set seq5=seq_%obj%_rimdesk2_hand02
set seq6=seq_%obj%_rimdesk2_hand03
set seq7=seq_%obj%_rimdesk2_reinit01
set seq8=seq_%obj%_rimdesk2_reinit02
set seq9=seq_%obj%_rimdesk2_reinit03

set Neff=0.0
set Th_FDCM=0.16
set Neff_reinit=0.1
set Th_FDCM_reinit=0.08
set limitYRotation=1


REM N(1)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.5
set Neff=0.0
set ransacTh=0.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set ransacTh=0.0
set numOfAnealingLayers=5

REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set ransacTh=20.0
set numOfAnealingLayers=5

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set ransacTh=0.0
set numOfAnealingLayers=5
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)_limitYRotation(false)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=5
set limitYRotation=0
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
set limitYRotation=1

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)numOfAnealingLayers(1)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=1
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
set numOfAnealingLayers=5



REM Waterglass_white_800
set obj=Waterglass_white_800
set seq1=seq_%obj%_rimdesk2_gen01
set seq2=seq_%obj%_rimdesk2_gen02
set seq3=seq_%obj%_rimdesk2_gen03
set seq4=seq_%obj%_rimdesk2_hand01
set seq5=seq_%obj%_rimdesk2_hand02
set seq6=seq_%obj%_rimdesk2_hand03
set seq7=seq_%obj%_rimdesk2_reinit01
set seq8=seq_%obj%_rimdesk2_reinit02
set seq9=seq_%obj%_rimdesk2_reinit03

set seq10=seq_%obj%_rimdesk_reinit01
set seq11=seq_%obj%_rimdesk_reinit02
set seq12=seq_%obj%_rimdesk_reinit03

set Neff=0.0
set Th_FDCM=0.16
set Neff_reinit=0.1
::set Th_FDCM_reinit=0.10
set Th_FDCM_reinit=0.14
::set Th_FDCM_reinit=0.13
set limitYRotation=0

REM N(1)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.5
set Neff=0.0
set ransacTh=0.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set ransacTh=0.0
set numOfAnealingLayers=5

REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set ransacTh=20.0
set numOfAnealingLayers=5

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set ransacTh=0.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq10% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq11% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq12% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)numOfAnealingLayers(1)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=1
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
set numOfAnealingLayers=5




REM Wineglass_800
set obj=Wineglass_800
set seq1=seq_%obj%_rimdesk2_gen01
set seq2=seq_%obj%_rimdesk2_gen02
set seq3=seq_%obj%_rimdesk2_gen03
set seq4=seq_%obj%_rimdesk2_hand01
set seq5=seq_%obj%_rimdesk2_hand02
set seq6=seq_%obj%_rimdesk2_hand03
set seq7=seq_%obj%_rimdesk2_reinit01
set seq8=seq_%obj%_rimdesk2_reinit02
set seq9=seq_%obj%_rimdesk2_reinit03

set Neff=0.0
set Th_FDCM=0.16
set Neff_reinit=0.1
set Th_FDCM_reinit=0.11
set limitYRotation=1


REM N(1)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.5
set ransacTh=0.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set ransacTh=0.0
set numOfAnealingLayers=5

REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set ransacTh=20.0
set numOfAnealingLayers=5

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set ransacTh=0.0
set numOfAnealingLayers=5
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=5

::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq7% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq8% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%
edgeBasedTracker.exe %obj% %seq9% %imgext% %N% %ARparam% %Neff_reinit% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM_reinit% %limitYRotation% %numOfAnealingLayers%

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)_limitYRotation(false)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=5
set limitYRotation=0
edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
set limitYRotation=1

REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)numOfAnealingLayers(1)==============================
set N=100
set ARparam=0.5
set ransacTh=20.0
set numOfAnealingLayers=1
::edgeBasedTracker.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%

::edgeBasedTracker.exe %obj% %seq4% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq5% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
::edgeBasedTracker.exe %obj% %seq6% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave% %Th_FDCM% %limitYRotation% %numOfAnealingLayers%
set numOfAnealingLayers=5



