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
Eigen::Matrix<double, 2, 1> Random_vector(Eigen::Matrix<double, 2, 2> A) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, 1.0);

  Eigen::Matrix<double, 2, 1> R;

	R << distribution(generator),distribution(generator);
	// distribution(generator),distribution(generator);
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
    Eigen::Matrix<double, 2, 2> cov; // covarience of sensor noise
    cov << 
		1.5,1.9,1.9,2.0;
		// 0.5, 0.6, 0.7, 0.6, 
		// 0.4, 0.3, 0.4, 0.6,
		// 0.7, 0.6, 0.9, 0.5;

		// cov << 
		// 1.93, 1.47, 1.94, 1.92,
		// 1.45, 1.18, 1.54, 1.53,
		// 1.09, 0.86, 1.15, 1.08,
		// 1.64, 1.28, 1.65, 1.78 ;
    Eigen::Matrix<double, 2, 1> noise = Random_vector(cov);

		// std::cout << noise(0,0) << "\t" << noise(1,0) << "\n";
    x = p.x + noise(0, 0);
    y = p.y + noise(1, 0);
    // vx = p.vx + noise(2, 0);
    // vy = p.vy + noise(3, 0);
  }
};

class KalmanFilter {
public:
  Eigen::Matrix<double, 4, 4> A; // state transition matrix (model)
  Eigen::Matrix<double, 2, 4> C; // measurement model matrix

  Eigen::Matrix<double, 4, 1> B; // this is control input model
  Eigen::Matrix<double, 4, 1> state;
  Eigen::Matrix<double, 4, 4> state_cov;
  Eigen::Matrix<double, 1, 1> u;

  // Eigen::Matrix<double, 2, 1> measurement;
  // Eigen::Matrix<double, 2, 2> measurement_cov;

  Eigen::Matrix<double, 2, 2> sensor_noise_covarience; // this is the sensor noise covarience
  Eigen::Matrix<double, 4, 4> process_noise_covarience; // process noise covarience

  double period = 0.1;
  KalmanFilter() {
    state << 0,0,0,0;

    state_cov.setIdentity();
    // measurement_cov.setIdentity();
   //  measurement_cov  << 
			// 1.93, 1.47, 1.94, 1.92;
			// 1.45, 1.18, 1.54, 1.53,
			// 1.09, 0.86, 1.15, 1.08,
			// 1.64, 1.28, 1.65, 1.78;

    process_noise_covarience.setZero();
		sensor_noise_covarience.setIdentity();
		// v << 
		// 	1.93, 1.47, 1.94, 1.92,
		// 	1.45, 1.18, 1.54, 1.53,
		// 	1.09, 0.86, 1.15, 1.08,
		// 	1.64, 1.28, 1.65, 1.78;
  }

  void update(Sensor s) {
    C << 1,0,0,0,
			0,1,0,0;

    Eigen::Matrix<double, 2, 1> sensor;
    sensor << s.x, s.y;
			// , s.vx, s.vy;

    Eigen::Matrix<double, 2, 2> S = C * state_cov * C.transpose() + sensor_noise_covarience;

    Eigen::Matrix<double, 4, 2> kalman_gain =
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

    state_cov = A * state_cov * A.transpose() + process_noise_covarience;
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
  Sensor S(0, 0, 0, 0);
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

		std::cout << "====================================================state covarience===============================================================\n";
		std::cout << K.state_cov;
		std::cout << "\n";
  }
  return 0;
}
