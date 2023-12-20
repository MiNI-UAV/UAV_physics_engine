#include "aircraft.hpp"

void Aircraft::calcImpulseForce(double COR, double mi_static, double mi_dynamic,
    Eigen::Vector3d collisionPoint, Eigen::Vector3d surfaceNormal)
{
    std::scoped_lock lck(mtx);
    const std::lock_guard<std::mutex> lock(state.state_mtx);
    auto Y = state.getY();
    Eigen::Matrix3d R_nb = Matrices::R_nb(Y);
    Eigen::Matrix3d R_bn = R_nb.inverse();
    Eigen::Matrix<double,6,6> T, T_inv;
    T.setZero();
    T_inv.setZero();
    T.block<3,3>(0,0) = R_bn;
    T.block<3,3>(3,3) = R_bn;
    T_inv.block<3,3>(0,0) = R_nb;
    T_inv.block<3,3>(3,3) = R_nb;      
    Eigen::Vector<double,6> X_g = T * state.getX();
    Eigen::Vector3d r = collisionPoint - Y.head<3>();
    Eigen::Vector3d vr = X_g.head<3>() + X_g.tail<3>().cross(r);
    double vn = vr.dot(surfaceNormal);
    if(vn >= 0.0)
    {
        return;
    }
    std::cout << "Energy before collision: " << X_g.transpose()*massMatrix*X_g << std::endl;
    double den_n = (invMassMatrix(0,0) 
        + ((invMassMatrix.block<3,3>(3,3)*r.cross(surfaceNormal)).cross(r)).dot(surfaceNormal));
    if(vn > -def::GENTLY_PUSH) vn = -def::GENTLY_PUSH;
    double jr = (-(1+COR)*vn)/den_n;
    Eigen::Vector<double,6> delta_n;
    delta_n << surfaceNormal, r.cross(surfaceNormal);
    X_g = X_g + jr*invMassMatrix*delta_n;

    Eigen::Vector3d vt = vr - (vr.dot(surfaceNormal))*surfaceNormal;
    if(vt.squaredNorm() > def::FRICTION_EPS)
    {
        Eigen::Vector3d tangent = vt.normalized();
        double js = mi_static*jr;
        double jd = mi_dynamic*jr;
        double den_t = (invMassMatrix(0,0) 
        + ((invMassMatrix.block<3,3>(3,3)*r.cross(tangent)).cross(r)).dot(tangent));
        double jf = vt.norm()/den_t;
        if(jf > js) jf = jd;
        Eigen::Vector<double,6> delta_t;
        delta_t << tangent, r.cross(tangent);
        X_g = X_g - jf*invMassMatrix*delta_t;
    }
    std::cout << "Energy after collision: " << X_g.transpose()*massMatrix*X_g << std::endl;
    Vector<double,6> newX = T_inv * X_g;
    state.setX(newX);
}

std::tuple<int, Eigen::Vector3d> Aircraft::dropCargo(int index)
{
  if (index < 0 || index >= noOfCargo)
    return std::tuple<int, Eigen::Vector3d>(-10, Eigen::Vector3d::Zero());

  int res = cargo[index].release(state.real_time.load());
  if (res < 0)
    return std::tuple<int, Eigen::Vector3d>(res, Eigen::Vector3d::Zero());

  Eigen::Vector3d lin_vel = calcMomentumConservanceConservation(
      cargo[index].getMass(), Eigen::Vector3d::Zero(), cargo[index].getOffset());

  return std::tuple<int, Eigen::Vector3d>(res, lin_vel);
}

std::tuple<int, Eigen::Vector3d> Aircraft::shootAmmo(int index)
{
    if (index < 0 || index >= noOfAmmo)
        return std::tuple<int, Eigen::Vector3d>(-10, Eigen::Vector3d::Zero());

    int res = ammo[index].release(state.real_time.load());
    if (res < 0)
        return std::tuple<int, Eigen::Vector3d>(res, Eigen::Vector3d::Zero());

    Eigen::Vector3d lin_vel = calcMomentumConservanceConservation(
        ammo[index].getMass(), ammo[index].getV0(), ammo[index].getOffset());

    return std::tuple<int, Eigen::Vector3d>(res, lin_vel);
}

Eigen::Vector3d
Aircraft::calcMomentumConservanceConservation(double m, Eigen::Vector3d speed,
                                              Eigen::Vector3d r) 
{
    std::scoped_lock lck(mtx);
    const std::lock_guard<std::mutex> lock(state.state_mtx);
    Matrix3d R_nb = Matrices::R_nb(state.getY());
    Matrix3d R_bn = R_nb.inverse();
    Matrix<double, 6, 6> T, T_inv;
    T.setZero();
    T_inv.setZero();
    T.block<3, 3>(0, 0) = R_bn;
    T.block<3, 3>(3, 3) = R_bn;
    T_inv.block<3, 3>(0, 0) = R_nb;
    T_inv.block<3, 3>(3, 3) = R_nb;
    Vector<double, 6> momentum = massMatrix * (T * state.getX());
    Vector3d obj_vel = state.getX().head<3>() + state.getX().tail<3>().cross(r);
    obj_vel += speed;
    Vector3d obj_linear_speed = R_bn * obj_vel;
    Vector3d obj_linear_momentum = m * obj_linear_speed;
    Vector<double, 6> obj_momentum;
    r = R_bn * r;
    obj_momentum << obj_linear_momentum, r.cross(obj_linear_momentum);
    reduceMass(m,r);
    Vector<double, 6> newX = T_inv * invMassMatrix * (momentum - obj_momentum);
    state.setX(newX);
    return obj_linear_speed;
}