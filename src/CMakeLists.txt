AUX_SOURCE_DIRECTORY(armour ARMOUR_DIR)
AUX_SOURCE_DIRECTORY(IndusCamera CAMERA_DIR)
AUX_SOURCE_DIRECTORY(serial SERIAL_DIR)
AUX_SOURCE_DIRECTORY(classifier CLASSIFIER_DIR)
AUX_SOURCE_DIRECTORY(predictor PREDICTOR_DIR)
AUX_SOURCE_DIRECTORY(energy ENERGY_DIR)
AUX_SOURCE_DIRECTORY(topaimer AIMER_DIR)
AUX_SOURCE_DIRECTORY(base BASE_DIR)
AUX_SOURCE_DIRECTORY(topaimer TOP_DIR)
AUX_SOURCE_DIRECTORY(filter Filter_DIR)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(src SHARED ${ARMOUR_DIR}
        ${CAMERA_DIR}
        ${SERIAL_DIR}
        ${CLASSIFIER_DIR}
        ${PREDICTOR_DIR}
        ${AIMER_DIR}
        ${BASE_DIR}
        ${TOP_DIR}
        ${ENERGY_DIR}
        ${Filter_DIR}

        common.cpp shooter/shooter.cpp shooter/shooter.h energy/data_tpye.h predictor/EKFPredictor.cpp predictor/EKFPredictor.h
        predictor/kalman_filter.cpp classifier/num_classifier.cpp classifier/num_classifier.h)

target_link_libraries(src
        #${CERES_LIBRARIES}
        #                          ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${Eigen3_LIBRARIES}
        /usr/lib/libgxiapi.so
        yaml-cpp
        #                          matplotlib_cpp::matplotlib_cpp
        #                          ${CERES_INCLUDE_DIRS}

        )