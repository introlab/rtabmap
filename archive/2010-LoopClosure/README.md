
 To reproduce results (based on parameters of this [paper](https://www.arxiv.org/abs/2407.15304)):
 
 ```
 rtabmap-console \
       --Rtabmap/StatisticLogged true\
       --Rtabmap/StatisticLoggedHeaders false\
       --Kp/DetectorStrategy 0\
       --Rtabmap/MemoryThr 300\
       --Rtabmap/LoopRatio 0.9\
       --SURF/HessianThreshold 150\
       --Mem/STMSize 30\
       --Vis/MaxFeatures 400\
       --Kp/TfIdfLikelihoodUsed false\
       --Kp/MaxFeatures 400\
       --Kp/BadSignRatio 0.25\
       --Mem/BadSignaturesIgnored true\
       --Mem/RehearsalSimilarity 0.20\
       --Mem/RecentWmRatio 0.2\
       -gt "~/Downloads/UdeS_1Hz.png"\
       ~/Downloads/UdeS_1Hz
 ```
Adding the ground truth file here is optional to show recall at 100% precision at the end of the process directly without using the octave/MATLAB script below. For NewCollege and CityCentre datasets, `rtabmap-imagesJoiner` can be used to assemble the left and right images together.
 
 To analyze with Octave/MATLAB, drop `LogF.txt` and `LogI.txt` generated files from command above in ShowLogs directly, then execute `showLogs.m`.
