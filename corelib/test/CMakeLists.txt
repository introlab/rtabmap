IF(G2O_FOUND)
   ADD_EXECUTABLE(test_optimizer_g2o_ba_outliers
      test_optimizer_g2o_ba_outliers.cpp)
   TARGET_LINK_LIBRARIES(test_optimizer_g2o_ba_outliers rtabmap_core)
   ADD_TEST(NAME optimizer_g2o_ba_outliers COMMAND test_optimizer_g2o_ba_outliers)
ENDIF()
