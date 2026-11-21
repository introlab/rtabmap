IMU filters taken from ROS imu_tools stack: https://github.com/CCNYRoboticsLab/imu_tools, please look below for licensing. To avoid GPL license, MadgwickFilter can be disabled on compilation with "cmake -DWITH_MADGWICK=OFF ..".
===================================

Overview
-----------------------------------

IMU-related filters:

 * `MadgwickFilter`: a filter which fuses angular velocities,
accelerations, and (optionally) magnetic readings from a generic IMU 
device into an orientation. Based on the work of [1].

 * `ComplementaryFilter`: a filter which fuses angular velocities,
accelerations, and (optionally) magnetic readings from a generic IMU 
device into an orientation quaternion using a novel approach based on a complementary fusion. Based on the work of [2].

License
-----------------------------------

 * `MadgwickFilter`: GPL-3.0-or-later, following the original implementation (imu_filter_madgwick)
 
 * `ComplementaryFilter`: BSD-3-Clause (imu_complementary_filter)

References
-----------------------------------
 [1] http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

 [2] http://www.mdpi.com/1424-8220/15/8/19302
