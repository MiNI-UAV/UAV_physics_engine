VectorXd RK4_step(double t,
                          VectorXd y0,
                          std::function<VectorXd(double,VectorXd)> rhs_fun,
                          double h)
{
    auto k1 = rhs_fun(t      , y0);
    auto k2 = rhs_fun(t + h/2, y0+ (h/2)*k1);
    auto k3 = rhs_fun(t + h/2, y0+ (h/2)*k2);
    auto k4 = rhs_fun(t + h  , y0+ h*k3);
    return y0 + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);
}