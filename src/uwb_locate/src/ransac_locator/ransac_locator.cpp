#include "ransac_locator/ransac_locator.hpp"

double angle_normalize(double angle)
{
    return angle >= 0 ? angle : angle + PI;
}

bool fequal(double f1, double f2)
{
    return fabs(f1 - f2) < EPS;
}

LOCATOR_RETURN calculate_pos_robust_ransac(std::unordered_map<int, Position3D> anchorPoseMap,
                                           std::unordered_map<int, double> distances,
                                           Position3D &res)
{
    int max_iterations = 10;

    double threshold = 0.1;

    if (distances.size() < CALCULABLE_LEAST_CNT)
    {
        // effective_landmark_num too small, can't calculate.
        return EFFECTIVE_LANDMARK_TOO_SMALL;
    }
    else if (distances.size() <= RANSAC_LEAST_CNT)
    {
        // effective_landmark_num too small, calculate all.
        return calculate_pos_for_all_(anchorPoseMap, distances, res);
    }
    else
    {
        bool fit_success = false;
        // ransac choose the best, then refine
        double residual_best = DBL_MAX;

        std::vector<int> indexes;

        for (auto it : distances)
        {
            indexes.push_back(it.first);
        }

        std::vector<int> all_fit_indexes_best;

        for (int i = 0; i < max_iterations; i++)
        {
            // choose sample landmarks
            // iteration
            std::vector<int> sample_indexes;
            get_sample_indexes_(indexes, sample_indexes, RANSAC_LEAST_CNT);

            // calculate pos based on the chosen sample
            Position3D res_temp;
            if (CALCULATE_SUCCESS == calculate_pos_with_index_(anchorPoseMap, distances, sample_indexes, res))
            {

                double total_residual = DBL_MAX;
                double threshold = 0.1;

                std::vector<int> all_fit_indexes;

                get_all_fit_indexes_(anchorPoseMap, distances, sample_indexes, res_temp, threshold, all_fit_indexes, total_residual);

                if (fit_success == false ||
                    all_fit_indexes.size() > all_fit_indexes_best.size() ||
                    (all_fit_indexes.size() == all_fit_indexes_best.size() && total_residual < residual_best))
                {
                    residual_best = total_residual;

                    all_fit_indexes_best = all_fit_indexes;

                    res = res_temp;

                    fit_success = true;
                }
            }
        }
        // refine model
        if (fit_success)
        {
            if (ENABLE_RANSAC_REFINE && all_fit_indexes_best.size() >= RANSAC_LEAST_CNT)
            {
                Position3D res_temp;
                if (CALCULATE_SUCCESS == calculate_pos_with_index_(anchorPoseMap, distances, all_fit_indexes_best, res_temp))
                {
                    double residual_refined = 0.0;

                    for (int i = 0; i < all_fit_indexes_best.size(); i++)
                    {
                        double estimated_distance = sqrt(pow((anchorPoseMap[all_fit_indexes_best[i]].x - res.x), 2) +
                                                         pow((anchorPoseMap[all_fit_indexes_best[i]].y - res.y),
                                                             2));
                        double difference_of_distances = fabs(distances[all_fit_indexes_best[i]] - estimated_distance);
                        residual_refined += difference_of_distances * difference_of_distances;
                    }

                    if (residual_refined < residual_best)
                    {
                        // order sensitive, so residual_refined may be big than residual_best
                        res = res_temp;
                    }
                }
            }
        }
        if (fit_success == true)
        {
            return CALCULATE_SUCCESS;
        }
        else
        {
            return FIT_FAILED;
        }
    }
}

LOCATOR_RETURN calculate_pos_for_all_(std::unordered_map<int, Position3D> anchorPoseMap,
                                      std::unordered_map<int, double> distances,
                                      Position3D &res)
{
    std::vector<int> effective_landmark_indexes;

    for (auto it : distances)
    {
        effective_landmark_indexes.push_back(it.first);
    }

    return calculate_pos_with_index_(anchorPoseMap, distances, effective_landmark_indexes, res);
}

LOCATOR_RETURN calculate_pos_with_index_(std::unordered_map<int, Position3D> anchorPoseMap,
                                         std::unordered_map<int, double> distances,
                                         std::vector<int> effective_landmark_indexes,
                                         Position3D &res)
{
    if (is_landmarks_in_one_line(anchorPoseMap, effective_landmark_indexes))
    {
        // all landmarks in one line, cannot locate the position.
        return anchor_ON_ONE_LINE;
    }

    int matrix_row_num = effective_landmark_indexes.size() - 1;
    int coefficient_num = 2;
    // Ax=b
    gsl_matrix *A = gsl_matrix_alloc(matrix_row_num, coefficient_num);
    gsl_matrix *a = gsl_matrix_alloc(matrix_row_num, coefficient_num);
    gsl_vector *b = gsl_vector_alloc(matrix_row_num);

    // result
    gsl_vector *sx = gsl_vector_alloc(coefficient_num);

    // Clear data

    gsl_matrix_set_zero(A);
    gsl_vector_set_zero(b);

    int index_last = effective_landmark_indexes[effective_landmark_indexes.size() - 1];
    for (int i = 0; i < effective_landmark_indexes.size() - 1; i++)
    {
        int index_i = effective_landmark_indexes[i];
        gsl_matrix_set(A, i, 0, (anchorPoseMap[index_i].x - anchorPoseMap[index_last].x) * 2);
        gsl_matrix_set(A, i, 1, (anchorPoseMap[index_i].y - anchorPoseMap[index_last].y) * 2);
        double bi = anchorPoseMap[index_i].x * anchorPoseMap[index_i].x -
                    anchorPoseMap[index_last].x * anchorPoseMap[index_last].x +
                    anchorPoseMap[index_i].y * anchorPoseMap[index_i].y -
                    anchorPoseMap[index_last].y * anchorPoseMap[index_last].y +
                    distances[index_last] * distances[index_last] -
                    distances[index_i] * distances[index_i];
        gsl_vector_set(b, i, bi);
    }
    gsl_matrix_memcpy(a, A);
    gsl_vector *tau = gsl_vector_alloc(coefficient_num); // matrix_row_num, coefficient_num
    gsl_vector *residuals = gsl_vector_alloc(matrix_row_num);

    gsl_multifit_linear_workspace *w = gsl_multifit_linear_alloc(matrix_row_num, coefficient_num);
    gsl_multifit_linear_svd(A, w);
    double rcond = gsl_multifit_linear_rcond(w); // reciprocal condition number

    if (rcond < 1e-4)
    {
        // conditional number is too large, indicating that the problem is ill-conditioned.
        return CONDITIONAL_NUMBER_TOO_LARGE;
    }
    double lambda_gcv = 0.1;
    double chisq, rnorm, snorm;
    gsl_multifit_linear_solve(0.0, A, b, sx, &rnorm, &snorm, w);

    res.x = gsl_vector_get(sx, 0);
    res.y = gsl_vector_get(sx, 1);

    gsl_matrix_free(A);
    gsl_matrix_free(a);
    gsl_vector_free(b);
    gsl_vector_free(tau);
    gsl_vector_free(sx);
    gsl_vector_free(residuals);
    return CALCULATE_SUCCESS;
}

bool
is_landmarks_in_one_line(std::unordered_map<int, Position3D> anchorPoseMap,
                         std::vector<int> effective_landmark_indexes)
{
    int index_0 = effective_landmark_indexes[0];

    int index_last = effective_landmark_indexes[effective_landmark_indexes.size() - 1];

    double angle_0 = angle_normalize(atan2(anchorPoseMap[index_0].y - anchorPoseMap[index_last].y,
                                           anchorPoseMap[index_0].x - anchorPoseMap[index_last].x));

    for (int i = 1; i < effective_landmark_indexes.size() - 1; i++)
    {
        int index_i = effective_landmark_indexes[i];
        double angle_i = angle_normalize(atan2(anchorPoseMap[index_i].y - anchorPoseMap[index_last].y,
                                               anchorPoseMap[index_i].x - anchorPoseMap[index_last].x));
        if (!fequal(angle_0, angle_i))
        {
            return false;
        }
    }
    return true;
}

bool get_sample_indexes_(std::vector<int> indexes,
                         std::vector<int> &sample_indexes,
                         int each_sample_cnt)
{
    // index_size > each_sample_cnt

    for (int i = 0; i < each_sample_cnt; i++)
    {
        // swap i and random index afterwards
        int rand_index = rand() % indexes.size();

        sample_indexes.push_back(indexes[rand_index]);

        indexes.erase(indexes.begin() + rand_index);
    }
    return true;
}

void get_all_fit_indexes_(std::unordered_map<int, Position3D> anchorPoseMap,
                          std::unordered_map<int, double> distances,
                          std::vector<int> sample_indexes,
                          Position3D p,
                          double threshold,
                          std::vector<int> &all_fit_indexes,
                          double &total_residual_out)
{
    std::unordered_set<int> is_fit_vec;
    std::unordered_map<int, double> residuals;

    for (auto it : distances)
    {
        int id = it.first;

        double estimated_distance = sqrt(pow((anchorPoseMap[id].x - p.x), 2) + pow((anchorPoseMap[id].y - p.y), 2));
        double difference_of_distances = fabs(distances[id] - estimated_distance);
        if (difference_of_distances < threshold)
        {
            is_fit_vec.insert(id);
        }
        residuals[id] = difference_of_distances * difference_of_distances;
    }

    total_residual_out = 0;

    for (auto it : distances)
    {
        int id = it.first;
        if (is_fit_vec.find(id) != is_fit_vec.end())
        {
            all_fit_indexes.push_back(id);
            total_residual_out += residuals[id];
        }
    }
}