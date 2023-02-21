#ifndef RANSAC_LOCATOR_H
#define RANSAC_LOCATOR_H

#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <assert.h>


#include <unordered_map>
#include <unordered_set>

#include <gsl/gsl_multifit.h>

#include "nav_msgs/msg/odometry.hpp"

#define RANSAC_LEAST_CNT 4
#define CALCULABLE_LEAST_CNT 3
#define ENABLE_RANSAC_REFINE 1
#define PI 3.1415926535
#define EPS 0.0000001

typedef struct
{
    double x;
    double y;
    double z;
} Position3D;

enum LOCATOR_RETURN
{
    CALCULATE_SUCCESS,
    EFFECTIVE_LANDMARK_TOO_SMALL,
    EFFECTIVE_LANDMARK_LESS_2,
    anchor_ON_ONE_LINE,
    CONDITIONAL_NUMBER_TOO_LARGE,
    FIT_FAILED

};

double angle_normalize(double angle);

bool fequal(double f1, double f2);

LOCATOR_RETURN
calculate_pos_robust_ransac(std::unordered_map<int, Position3D> anchorPoseMap,
                            std::unordered_map<int, double> distances,
                            Position3D &res);

LOCATOR_RETURN
calculate_pos_for_all_(std::unordered_map<int, Position3D> anchorPoseMap,
                       std::unordered_map<int, double> distances,
                       Position3D &res);

LOCATOR_RETURN
calculate_pos_with_index_(std::unordered_map<int, Position3D> anchorPoseMap,
                          std::unordered_map<int, double> distances,
                          std::vector<int> effective_landmark_indexes,
                          Position3D &res);
bool
is_landmarks_in_one_line(std::unordered_map<int, Position3D> anchorPoseMap,
                         std::vector<int> effective_landmark_indexes);

bool get_sample_indexes_(std::vector<int> indexes,
                         std::vector<int> &sample_indexes,
                         int each_sample_cnt);

void get_all_fit_indexes_(std::unordered_map<int, Position3D> anchorPoseMap,
                         std::unordered_map<int, double> distances,
                         std::vector<int> sample_indexes,
                         Position3D p,
                         double threshold,
                         std::vector<int> &all_fit_indexes_out,
                         double &total_residual_out);
                         
#endif // RANSAC_LOCATOR