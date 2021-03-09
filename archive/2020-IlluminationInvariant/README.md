
## Multi-Session Visual SLAM for Illumination Invariant Localization in Indoor Environments

* Paper: https://arxiv.org/abs/2103.03827

* The setup: we did 6 mapping sessions at dusk to evaluate how well RTAB-Map can localize (only by vision) on maps taken at different illumination conditions. The data has been collected with [RTAB-Map Tango](https://play.google.com/store/apps/details?id=com.introlab.rtabmap&hl=en_CA&gl=US).
![Overview](https://github.com/introlab/rtabmap/raw/master/archive/2020-IlluminationInvariant/images/fig_overview.jpg)]


## Description

This folder contains scripts to re-generate results from the paper. The main idea behind this work is that using Multi-Session mapping can help to localize visually in illumination changing environments even with features that are not very robust to such conditions. We compared common hand-made visual features like SIFT, SURF, BRIEF, BRISK, FREAK, DAISY, KAZE with learned descriptor SuperPoint. The following picture show how robust are the visual features tested when localizing against single session recorded at different time. For example, the bottom-left and top-right cells are when the robot tries to localize the night on a map taken the day or vice-versa. The diagonal is localization performance when the localization session is about the same time than when the map was recorded. SuperPoint has clearly an advantage on this single-session experiment.

![All sessions](https://github.com/introlab/rtabmap/raw/master/archive/2020-IlluminationInvariant/images/fig_single_percentage.jpg)]

The following image shows when we do the same localization experiment at different hours, but against maps created by assembling maps taken at different hours. In this case, we can see that even binary features like BRIEF can work relatively well in illumination-variant environments. See the paper for more detailled results and comments. The line `1+2+3+4+5+6` refers to the assembled map shown below containing all mapping sessions linked together in same database.

![All sessions](https://github.com/introlab/rtabmap/raw/master/archive/2020-IlluminationInvariant/images/fig_merged_percentage.jpg)]


![All sessions](https://github.com/introlab/rtabmap/raw/master/archive/2020-IlluminationInvariant/images/fig_map_merged_999.jpg)]

