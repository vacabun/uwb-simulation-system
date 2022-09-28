#ifndef RANSAC_LOCATOR_H
#define RANSAC_LOCATOR_H

#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <assert.h>



#include <gsl/gsl_multifit.h>

#define RANSAC_LEAST_CNT 4
#define CALCULABLE_LEAST_CNT 3
#define ENABLE_RANSAC_REFINE 1
#define PI 3.1415926535
#define EPS 0.0000001

typedef struct
{
    double x;
    double y;
} Position2D;

double angle_normalize(double angle);
bool fequal(double f1, double f2);
bool is_landmarks_in_one_line(const Position2D landmarks[], const int effective_landmark_indexes[], int effective_landmark_num);
bool calculate_pos_with_index_(const Position2D landmarks[], const double distances[], const int effective_landmark_indexes[], int effective_landmark_num, double *x_out, double *y_out);
bool calculate_pos_for_all_(const Position2D landmarks[], const double distances[], int effective_landmark_num, double *x_out, double *y_out);
bool getSampleIndexes(int index_size, int sample_indexes[], int each_sample_cnt);
int get_all_fit_indexes(const Position2D landmarks[], const double distances[], int effective_landmark_num, const int sample_indexes[], int sample_size, double x, double y, double threshold, int all_fit_indexes_out[], double *total_residual_out);
bool calculate_pos_robust_ransac(const Position2D landmarks[], const double distances[], int effective_landmark_num, double *x_out, double *y_out);

#endif // RANSAC_LOCATOR_H