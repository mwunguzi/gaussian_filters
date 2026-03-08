/**
 * @headerfile extended_kalman_filter.hpp "extended_kalman_filter/include/"
 * @brief extended kalman filter implementation header file.
 *
 * @author Herve Mwunguzi (mwunguziher@gmail.com)
 * @bugs not yet known.
 * 
 */

#ifndef EXTENDED_KALMAN_FILTER_HEADER_
#define EXTENDED_KALMAN_FILTER_HEADER_

#include "Eigen/Dense"
#include <utility>
#include <functional>
/**
 * @brief class declaration for kalman filter implementation.
 *
 */
class extended_kalman_filter
{

public:
	/**
	 * @brief extended kalman filter class constructor
	 * 
	 * For extended kalman filter implementation, motion model of the system and 
     * sensor model or measurement model are the key ingredients.
	 * These models are usual constructed as state space equations.
	 * 
	 * x_t =  g(x_t-1,u_t) + epsl_t
	 * z_t =  h(x_t)   + sig_t
	 * 
	 * x_t    : State of the system at time t, usually given as a vector of size N.
	 * x_t-1  : State of the system at time t-1, vector of size N.
	 * z_t    : Sensor measurement vector at time t, vector pf size K.
	 * u_t    : Control vector of size M
	 * epsl_t : A gaussian random vector, with mean of 0 and covariance of R_t
	 * sig_t  : Measurement noise vector , with mean of 0 and covariance of Q_t.
	 * g      : A nonlinear state transition function.
	 * h      : A nonlinear sensor model that maps sensor into measurement space.
	 *
	 *
	 * @param [in] state_vector x_t0.
	 * @param [in] control_vector u_t0. 
	 * @param [in] state_noise_cov_vector R_t
	 * @param [in] measurement_noise_cov_vector Q_t
	 * @param [in] inital_belief_mean 
	 * @param [in] inital_belief_covariance 
	 *
	 */
	extended_kalman_filter(Eigen::VectorXf state_vector, Eigen::VectorXf control_vector,
        Eigen::VectorXf state_noise_cov_vector, Eigen::VectorXf measurement_noise_cov_vector,
        Eigen::VectorXf initial_belief_mean, Eigen::VectorXf initial_belief_covariance);


	/**
	 * @brief Function member to computer Kalman filter's corrected state
	 *
	 * @param [in]  control_vector u_t.
	 * @param [in]  sensor_measurement_vector z_t, new measurement from the sensor.
	 * @param [out] posterior belief mean and covariance. 
	 */
	std::pair<Eigen::VectorXf, Eigen::VectorXf> 
    compute_ekf_corrected_state(Eigen::MatrixXf control_vector, Eigen::MatrixXf sensor_measurement_vector);


	/**
	 * @brief function set the motion model and its jacobian from user callable objects with same signature 
     *
	 * @param [in] g  callable objects with a same signature. : g(x_t-1, u_t) 
	 * @param [in] G  callable objects with a same signature. Jacobian matrix of g(x_t-1, u_t) 
	 */
    void set_motion_model(std::function<Eigen::VectorXf(const Eigen::VectorXf&,const Eigen::VectorXf&)> g,
        std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> G);

    
	/**
	 * @brief function set the measurement model and its jacobian from user callable objects with same signature 
     *
	 * @param [in] h  callable objects with a same signature. : h(x_t) 
	 * @param [in] H  callable objects with a same signature. Jacobian matrix of h(x_t)  
	 */
    void set_measurement_model(std::function<Eigen::VectorXf(const Eigen::VectorXf&)> h,
        std::function<Eigen::MatrixXf(const Eigen::VectorXf&)> H);


private: 
	
	Eigen::VectorXf m_state_vector; // x_t-1
	Eigen::VectorXf m_control_vector; // u_t
	Eigen::VectorXf m_state_noise_cov_vector; //R_t
	Eigen::VectorXf m_measurement_noise_cov_vector; //Q_t
	Eigen::VectorXf m_previous_belief_mean;
	Eigen::VectorXf m_previous_belief_covariance;
	std::function<Eigen::VectorXf(const Eigen::VectorXf&)> m_h; // h(x_t)
	std::function<Eigen::MatrixXf(const Eigen::VectorXf&)> m_H; // H, jacobian matrix
	std::function<Eigen::VectorXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> m_g; // g(x_t-1,u_t)
	std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> m_G; // G, jacobian matrix
	
	/**
	 * @brief helper function to calculate predicted mean from control input,
	 * state matrix, control matrix, and prior belief mean of the system.
	 *
	 * @param [in]  control_vector u_t.
	 * @param [out] predicted mean as a vector.
	 */
	Eigen::VectorXf calculate_ekf_predicted_mean(Eigen::VectorXf control_vector);

	
	/**
	 * @brief helper function to calculate predicted covariance from state noise covariance,
	 * state matrix, and prior belief covariance of the system.
	 *
	 * @param [in]  control_vector u_t.
	 * @param [out] predicted covariance as a vector.
	 */
	Eigen::VectorXf calculate_ekf_predicted_covariance(Eigen::VectorXf control_vector);


	/**
	 * @brief helper function for calculating kalman gain from predicted covariance,
	 * sensor measurement matrix, and measurement noise covariance.
	 *
	 * @param [in]  pred_mean, predicted mean from @ref calculate_ekf_predicted_mean() function
	 * @param [in]  pred_cov, predicted covariance from @ref calculate_kf_predicted_covariance() function
	 * @param [out] K_t, Kalman gain
	 */
	Eigen::MatrixXf calculate_ekf_kalman_gain(Eigen::VectorXf pred_mean, Eigen::VectorXf pred_cov);


	/**
	 * @brief helper function to calculate corrected mean from predicted mean, kalman gain,
	 * sensor matrix, and sensor measurement. 
	 *
	 * @param [in]  pred_mean , calculated predicted mean from @ref calculate_kf_predicted_mean() function.
	 * @param [in]  k_gain , calculated kalman gain K_t from @ref calculate_kf_kalman_gain() function.
	 * @param [in]  sensor_measu_vec z_t.
	 * @param [out] corrected mean as a vector.
	 *
	 */
	Eigen::MatrixXf calculate_ekf_corrected_mean(Eigen::VectorXf pred_mean,Eigen::MatrixXf k_gain, Eigen::MatrixXf sensor_measu_vec);
	

	/**
	 * @brief helper function to calculate corrected mean from kalman gain, measurement matrix,
	 * and predicted covariance.
	 * 
	 * @param [in]  pred_mean , calculated predicted mean from @ref calculate_kf_predicted_mean() function.
 	 * @param [in]  pred_cov , calculated predicted covariance from @ref calculate_kf_predicted_cov() function.
 	 * @param [in]  kf_gain , calculated kalman gain K_t from @ref calculate_kf_kalman_gain() function.
	 * @param [out] corrected mean as a vector.
	 */
	Eigen::MatrixXf calculate_ekf_corrected_covariance(Eigen::VectorXf pred_mean, Eigen::VectorXf pred_cov, Eigen::MatrixXf kf_gain);


};

#endif //EXTENDED_KALMAN_FILTER_HEADER_
