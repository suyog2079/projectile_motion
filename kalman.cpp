#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <random>
#include <sys/types.h>
#define g 9.81

class Projectile {
public:
  double x;
  double y;
  double vx;
  double vy;

  int time = 0;
  double period = 0.1;

  Projectile(double x_, double y_, double vx_, double vy_)
      : x(x_), y(y_), vx(vx_), vy(vy_) {};

  /**
   * @brief to change the state of system. Time goes up once. actual ground
   * truth.
   */
  void tick() {
    time++;

    x = vx * period + x;
    y = vy * period - (1 / 2.0) * g * period * period;
    vy = vy - g * period;
  }
};

/**
 * @class RandomVector
 * @brief I took this directly from ujwol. Don't know how it works.
 *
 */
class RandomVector {
public:
  float m1_, m2_, v1_, c12_, v2_;

  RandomVector(float m1, float m2, float v1, float c12, float v2) {
    m1_ = m1;
    m2_ = m2;
    v1_ = v1;
    v2_ = v2;
    c12_ = c12;
  }

  float l11, l22, l12;

  float x, y;

  void set_vector() {
    // Cholesky Decompositinn
    l11 = sqrt(v1_);
    l12 = c12_ / l11;

    if (v2_ - l12 * l12 < 0)
      throw -1;

    l22 = sqrt(v2_ - l12 * l12);

    float z1, z2;

    // Define random generator with Gaussian distribution
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> dist(0.0, 1.0);

    z1 = dist(generator);
    z2 = dist(generator);

    x = l11 * z1;
    y = l12 * z1 + l22 * z2;

    x += m1_;
    y += m2_;
  }
};

class Sensor {
public:
  double x;
  double y;
  double vx;
  double vy;

public:
  Sensor(double x_, double y_, double vx_, double vy_)
      : x(x_), y(y_), vx(vx_), vy(vy_) {}

  // TODO: change this thing later. The random matrix generator needs to be
  // changed

  /**
   * @brief to change later
   *
   * @param p
   */
  void update_sensor_data(const Projectile &p) {
    RandomVector r(0, 0, 0.09, 0.01, 0.3);
    r.set_vector();

    x = p.x + r.x;
    y = p.y + r.y;
  }
};

class KalmanFilter {
public:
  Eigen::Matrix<double, 4, 4> A; // state transition matrix (model)
  Eigen::Matrix<double, 4, 4> C; // measurement model matrix

  Eigen::Matrix<double, 4, 1> B; // this is control input model
  Eigen::Matrix<double, 4, 1> state;
  Eigen::Matrix<double, 4, 4> state_cov;
  Eigen::Matrix<double, 1, 1> u;

  Eigen::Matrix<double, 4, 1> measurement;
  Eigen::Matrix<double, 4, 4> measurement_cov;

  Eigen::Matrix<double, 4, 4> v; // this is the sensor noise
  Eigen::Matrix<double, 4, 4> w; // process noise

  double period = 0.1;
  KalmanFilter() {
    state << 0.5, 0.2;

    state_cov.setIdentity();
    measurement_cov.setIdentity();

    w.setZero();
    v << 0.09, 0.01, 0.01, 0.3;
  }

  void update(Sensor s) {
    C.setIdentity();

    Eigen::Matrix<double, 4, 1> sensor;
    sensor << s.x, s.y,s.vx,s.vy;

    Eigen::Matrix<double, 4, 4> S = C * state_cov * C.transpose() + v;

    Eigen::Matrix<double, 4, 4> kalman_gain =
        state_cov * C.transpose() * S.inverse();

    state = state + kalman_gain * (sensor - C * state);

    Eigen::Matrix<double, 4, 4> I;
    I.setIdentity();

    state_cov = (I - kalman_gain * C) * state_cov;
  }

  void predict() {
    A << 1, 0, period, 0, 0, 1, 0, period, 0, 0, 1, 0, 0, 0, 0, 1;
    u << g;
    B << 0, 0, 0, (-1 / 2.0f) * period * period;

    state = A * state + B * u;

    state_cov = A * state_cov * A.transpose() + w;
  }
};

std::ostream &operator<<(std::ostream &os, const Projectile p) {
  return os << p.time * p.period << "\t" << p.x << "\t" << p.y;
}

std::ostream &operator<<(std::ostream &os, const Sensor s) {
  return os << "\t" << s.x << "t" << s.y;
}

int main() {
  Projectile P(0,0,10,10);
  Sensor S(0,0,0,0);
  KalmanFilter K;

  std::ofstream fs("data.txt", std::ios::out);

  for (int i = 0; i < 100; i++) {
    P.tick();
    K.predict();
    S.update_sensor_data(P);
    fs << S;
    K.update(S);
    fs << "\t" << K.state(0, 0);
    fs << "\n";
  }
  return 0;
}
