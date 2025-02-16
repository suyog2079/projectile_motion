#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Eigenvalues/EigenSolver.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <random>
#include <sys/types.h>
#define g 9.81

/**
 * @class RandomVector
 * @brief I took this directly from ujwol. Don't know how it works.
 *
 */
Eigen::Matrix<double, 4, 1> Random_vector(Eigen::Matrix<double, 4, 4> A) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, 1.0);

  Eigen::Matrix<double, 4, 1> R;

	R << distribution(generator),distribution(generator),distribution(generator),distribution(generator);
  return A * R;
}

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
    y = y + vy * period - (1 / 2.0) * g * period * period;
    vy = vy - g * period;
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

  void update_sensor_data(const Projectile &p) {
    Eigen::Matrix<double, 4, 4> cov; // covarience of sensor noise
    cov << 
				0.4, 0.5, 0.6, 0.9,
				0.5, 0.6, 0.7, 0.6, 
				0.4, 0.3, 0.4, 0.6,
				0.7, 0.6, 0.9, 0.5;

    Eigen::Matrix<double, 4, 1> noise = Random_vector(cov);
    x = p.x + noise(0, 0);
    y = p.y + noise(1, 0);
    vx = p.vx + noise(2, 0);
    vy = p.vy + noise(3, 0);
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
    state << 0,0,0,0;

    state_cov.setIdentity();
    measurement_cov.setIdentity();

    w.setZero();
		// v.setIdentity();
    v <<1.28, 1.22, 1.64, 1.47,
			1.2,  1.18, 1.54, 1.53,
			0.89, 0.86, 1.15, 1.08,
			1.29, 1.28, 1.65, 1.78;
  }

  void update(Sensor s) {
    C.setIdentity();

    Eigen::Matrix<double, 4, 1> sensor;
    sensor << s.x, s.y, s.vx, s.vy;

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
    B << 0, (-1 / 2.0f) * period * period, 0, -period;

    state = A * state + B * u;

    state_cov = A * state_cov * A.transpose() + w;
  }
};

std::ostream &operator<<(std::ostream &os, const Projectile p) {
  return os << p.time * p.period << "\t" << p.x << "\t" << p.y;
}

std::ostream &operator<<(std::ostream &os, const Sensor s) {
  return os << "\t" << s.x << "\t" << s.y;
}

int main() {
  Projectile P(0, 0, 10, 50);
  Sensor S(0, 0, 5, 40);
  KalmanFilter K;

  std::ofstream fs("data.txt", std::ios::out);

  for (int i = 0; i < 100; i++) {
    P.tick();
    K.predict();
    S.update_sensor_data(P);
    fs << S;
    K.update(S);
    fs << "\t" << K.state(0, 0);
		fs << "\t" << K.state(1, 0);
    fs << "\n";
  }
  return 0;
}
