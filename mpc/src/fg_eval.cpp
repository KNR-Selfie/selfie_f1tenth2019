#include "fg_eval.h"

using Eigen::VectorXd;
using CppAD::AD;

FG_eval::FG_eval (VectorXd pathCoeffs)
{
    this->pathCoeffs = pathCoeffs;
    //Stala promienia skretu
    //TODO dokladna wartosc dla naszego auta
    //Lf = 2.67;
    //Prediction horizon
    N = 10;
    //Timestep length
    dt = 0.1;
    //Target velocity
    ref_v = 2.0;

    //Index of the first variable representing:
    //Position on map
    x_start = 0;
    //Position on map
    y_start = x_start + N;
    //Orientation on map
    psi_start = y_start + N;
    //Velocity
    v_start = psi_start + N;
    //Path position offset
    cte_start = v_start + N;
    //Path heading offset
    epsi_start = cte_start + N;
    //Controlled variables:
    //Steering angle
    delta_start = epsi_start + N;
    //Acceleration
    a_start = delta_start + N - 1;
}



/*
fg[0] zawiera koszt f(x) - wszystkie skladowe kosztu powinny byc dodane do fg[0]
Pozostale elementy fg to funkcje g(x) - wszystkie ograniczenia ktore nakladamy
W tym przypadku zarowno stany jak i komendy naleza do zmiennych, wiec nakladamy
ograniczenie, ktore zapewnia zgodnosc stanow z wartosciami, ktore wynikaja z poprzedniego
stanu, wartosci komend i modelu
*/
void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
    fg[0] = 0;

    // The part of the cost based on the reference state
    for (unsigned int t = 0; t < N; ++t)
    {
        fg[0] += CTE_WEIGHT*CppAD::pow(vars[cte_start + t], 2);
        fg[0] += EPSI_WEIGHT*CppAD::pow(CppAD::sin(vars[epsi_start + t]), 2);
        fg[0] += V_WEIGHT*CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (unsigned int t = 0; t < N - 1; ++t)
    {
        fg[0] += DELTA_WEIGHT*CppAD::pow(vars[delta_start + t], 2);
        fg[0] += A_WEIGHT*CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (unsigned int t = 0; t < N - 2; ++t)
    {
        fg[0] += DIFF_DELTA_WEIGHT*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += DIFF_A_WEIGHT*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //Optimizer constraints - g(x)
    fg[1 + x_start] = 0;
    fg[1 + y_start] = 0;
    fg[1 + psi_start] = 0;
    fg[1 + v_start] = 0;
    fg[1 + cte_start] = 0;
    fg[1 + epsi_start] = 0;

    for (unsigned int t = 1; t < N; ++t)
    {
        // The state at time t+1 .
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        // The state at time t.
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> a0 = vars[a_start + t - 1];

        AD<double> f1 = pathCoeffs[0] + pathCoeffs[1] * x1 + pathCoeffs[2] * x1*x1;
        AD<double> psides1 = CppAD::atan(pathCoeffs[1] + 2 * pathCoeffs[2] * x1);

        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 + v0 / Lf * delta0 * dt);
        fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
        fg[1 + cte_start + t] = cte1 - (f1 - y1);
        fg[1 + epsi_start + t] = epsi1 - (psides1 - psi1);
    }
}
