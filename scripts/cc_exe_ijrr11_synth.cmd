@ECHO OFF
REM program.exe [object name] [sequence name] [file extension for images:"jpg"] [num of particles:1] [AR param:0.0] [threshold Neff:0.1] [edgeSmoothSize:1] [cannyLow:20] [cannyHigh:40] [sampleStep:0.005] [maxD:32] [dullEdge:0] [threshold for ransac:0] [noiseRateLow:0.001] [noiseRateHigh:0.01] [detSave:"."] [threshold for FDCM:0.2]

::set obj=tea_box
::set seq=seq_tea_box_synth_wbg_01
set imgext=png
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
set destSave=G:\IJRR11_exp\08252011_synth_PF_addTime
set destSaveIRLS=G:\IJRR11_exp\08252011_synth_IRLS_addTime

REM tex_box
set obj=tea_box
set seq1=seq_%obj%_synth_wbg_01
set seq2=seq_%obj%_synth_ebg_01
set seq3=seq_%obj%_synth_hbg_01
REM N(1)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
::edgeBasedTracker_IRLS.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
edgeBasedTracker_IRLS.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%

edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%



REM opencvbook
set obj=opencvbook
set seq1=seq_%obj%_synth_wbg_01
set seq2=seq_%obj%_synth_ebg_01
set seq3=seq_%obj%_synth_hbg_01
REM N(1)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
::edgeBasedTracker_IRLS.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
edgeBasedTracker_IRLS.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%

edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%



REM starbucks_cup
set obj=starbucks_cup
set seq1=seq_%obj%_synth_wbg_01
set seq2=seq_%obj%_synth_ebg_01
set seq3=seq_%obj%_synth_hbg_01
set dullEdge=1
REM N(1)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
::edgeBasedTracker_IRLS.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
edgeBasedTracker_IRLS.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%

edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%



REM GMcardoor
set obj=GMcardoor
set seq1=seq_%obj%_synth_wbg_01
set seq2=seq_%obj%_synth_ebg_01
set seq3=seq_%obj%_synth_hbg_01
set dullEdge=0
set sampleStep=0.02
REM N(1)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)=================================
set N=1
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_IRLS.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
::edgeBasedTracker_IRLS.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%
edgeBasedTracker_IRLS.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSaveIRLS%

edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.0)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.0
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(0)===============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=0.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
REM N(100)_AR(0.5)_Neff(0.0)_smooth(1)_ransac(20)==============================
set N=100
set ARparam=0.5
set Neff=0.0
set ransacTh=20.0
edgeBasedTracker_PF.exe %obj% %seq1% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
::edgeBasedTracker_PF.exe %obj% %seq2% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%
edgeBasedTracker_PF.exe %obj% %seq3% %imgext% %N% %ARparam% %Neff% %edgeSmoothSize% %cannyLow% %cannyHigh% %sampleStep% %maxD% %dullEdge% %ransacTh% %noiseRateLow% %noiseRateHigh% %destSave%





