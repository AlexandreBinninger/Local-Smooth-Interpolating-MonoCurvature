#include "increase.h"


/*
    Increase functions
*/

Increase_Type increase_function = Max_Linear_incr;

/*
    Generic functions
*/



void increase(const double& K_0, const double& K_1, const double& alpha, double& new_K_0, double& new_K_1){
    switch (increase_function)
    {
    case Linear_incr:
        /* code */
        linear_increase(K_0, K_1, alpha, new_K_0, new_K_1);
        break;
    case Max_Linear_incr:
        max_linear_increase(K_0, K_1, alpha, new_K_0, new_K_1);
        break;
    default:
        break;
    }

}
void increase(const double &K_0, const double &K_1, const double &K_2, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2){
    switch (increase_function)
    {
    case Linear_incr:
        /* code */
        linear_increase(K_0, K_1, K_2, alpha, new_K_0, new_K_1, new_K_2);
        break;
    case Max_Linear_incr:
        max_linear_increase(K_0, K_1, K_2, alpha, new_K_0, new_K_1, new_K_2);
        break;
    default:
        break;
    }
}
void increase(const double &K_0, const double &K_1, const double &K_2, const double &K_3, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2, double &new_K_3){
    switch (increase_function)
    {
    case Linear_incr:
        /* code */
        linear_increase(K_0, K_1, K_2, K_3, alpha, new_K_0, new_K_1, new_K_2, new_K_3);
        break;
    case Max_Linear_incr:
        max_linear_increase(K_0, K_1, K_2, K_3, alpha, new_K_0, new_K_1, new_K_2, new_K_3);
        break;
    default:
        break;
    }
}


/*
    Linear increase functions
*/

void linear_increase(const double& K_0, const double& K_1, const double& alpha, double& new_K_0, double& new_K_1){
    new_K_0 = alpha*K_0;
    new_K_1 = alpha*K_1;
}
void linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2){
    new_K_0 = alpha*K_0;
    new_K_1 = alpha*K_1;
    new_K_2 = alpha*K_2;
}
void linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &K_3, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2, double &new_K_3){
    new_K_0 = alpha*K_0;
    new_K_1 = alpha*K_1;
    new_K_2 = alpha*K_2;
    new_K_3 = alpha*K_3;
}


/*
    Max-Linear increase functions
*/
void max_linear_increase_ordered(const std::vector<std::pair<double, int>> &K, const double &alpha, std::vector<std::pair<double, int>> &new_K)
{
    // Suppose that the curvatures K are increasingly ordered by absolute value.
    double alpha_left = alpha;
    new_K.clear();
    for (auto it = K.begin(); it != K.end(); it++)
    {
        new_K.push_back(std::pair<double, int>((*it).first, (*it).second));
    }


    bool stop = false;
    for (int i = 0; i < new_K.size() - 1; i++)
    {
        if (!stop)
        {
            double big_K = abs(new_K[i + 1].first);
            double small_K = abs(new_K[i].first);
            double alpha_max = big_K / small_K;
            if (alpha_max > alpha_left)
            { // we won't be able to continue
                stop = true;
                double K_limit = small_K * alpha_left;
                for (int j = 0; j <= i; j++)
                {
                    new_K[i].first = sign(new_K[i].first) * K_limit;
                }
            }
            else
            {
                alpha_left = alpha_left / alpha_max;
                for (int j = 0; j <= i; j++)
                {
                    new_K[i].first = sign(new_K[i].first) * big_K;
                }
            }
        }
    }

    if (!stop)
    {
        double K_limit = abs(new_K[0].first) * alpha_left;
        for (int i = 0; i < new_K.size(); i++)
        {
            new_K[i].first = sign(new_K[i].first) * K_limit;
        }
    }
}


bool personal_compare(std::pair<double, int> K1, std::pair<double, int> K2)
{
    return abs(K1.first) < abs(K2.first);
}

void max_linear_increase(const double &K_0, const double &K_1, const double &alpha, double &new_K_0, double &new_K_1)
{
    // increases linearly until it reaches the maximum of both curvature
    std::vector<std::pair<double, int>> K, new_K;
    K.push_back(std::pair<double, int>(K_0, 0));
    K.push_back(std::pair<double, int>(K_1, 1));
    sort(K.begin(), K.end(), personal_compare);
    max_linear_increase_ordered(K, alpha, new_K);

    for (int i = 0; i < new_K.size(); i++)
    {
        auto n_K = new_K[i];
        if (n_K.second == 0)
        {
            new_K_0 = n_K.first;
        }
        else if (n_K.second == 1)
        {
            new_K_1 = n_K.first;
        }
    }
}

void max_linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2)
{
    // increases linearly until it reaches the maximum of both curvature
    std::vector<std::pair<double, int>> K, new_K;
    K.push_back(std::pair<double, int>(K_0, 0));
    K.push_back(std::pair<double, int>(K_1, 1));
    K.push_back(std::pair<double, int>(K_2, 2));
    sort(K.begin(), K.end(), personal_compare);
    max_linear_increase_ordered(K, alpha, new_K);

    for (int i = 0; i < new_K.size(); i++)
    {
        auto n_K = new_K[i];
        if (n_K.second == 0)
        {
            new_K_0 = n_K.first;
        }
        else if (n_K.second == 1)
        {
            new_K_1 = n_K.first;
        }
        else if (n_K.second == 2)
        {
            new_K_2 = n_K.first;
        }
    }
}
void max_linear_increase(const double &K_0, const double &K_1, const double &K_2, const double &K_3, const double &alpha, double &new_K_0, double &new_K_1, double &new_K_2, double &new_K_3)
{
    // increases linearly until it reaches the maximum of both curvature
    std::vector<std::pair<double, int>> K, new_K;
    K.push_back(std::pair<double, int>(K_0, 0));
    K.push_back(std::pair<double, int>(K_1, 1));
    K.push_back(std::pair<double, int>(K_2, 2));
    K.push_back(std::pair<double, int>(K_3, 3));
    sort(K.begin(), K.end(), personal_compare);
    max_linear_increase_ordered(K, alpha, new_K);

    for (int i = 0; i < new_K.size(); i++)
    {
        auto n_K = new_K[i];
        if (n_K.second == 0)
        {
            new_K_0 = n_K.first;
        }
        else if (n_K.second == 1)
        {
            new_K_1 = n_K.first;
        }
        else if (n_K.second == 2)
        {
            new_K_2 = n_K.first;
        }
        else if (n_K.second == 3)
        {
            new_K_3 = n_K.first;
        }
    }
}