
/**
 * @headerfile extended_kalman_filter.cpp "extended_kalman_filter/src/"
 * @brief extended kalman filter implementation file.
 *
 * @author Herve Mwunguzi (mwunguziher@gmail.com)
 * @bugs not yet known.
 * 
 */

#include "extended_kalman_filter.hpp"

/**
 * @details
 * extended Kalman filter class's constructor definition initializing all member variables needed for
 */
extended_kalman_filter::extended_kalman_filter(Eigen::VectorXf state_vector, Eigen::VectorXf control_vector,
		             Eigen::VectorXf state_noise_cov_vector, Eigen::VectorXf measurement_noise_cov_vector,
		             Eigen::VectorXf initial_belief_mean, Eigen::VectorXf initial_belief_covariance )
			     : m_state_vector(state_vector),                               
			       m_control_vector(control_vector),
			       m_state_noise_cov_vector(state_noise_cov_vector), 
			       m_measurement_noise_cov_vector(measurement_noise_cov_vector),
			       m_previous_belief_mean(initial_belief_mean),
			       m_previous_belief_covariance(initial_belief_covariance)
{

}

/**
 * @brief Function member to computer extended Kalman filter's corrected state
 * 
 * @details
 * This function uses Kalman filter's algorithm to compute the corrected state of a system.
 *
 * -----------------------------------------------------------------------------------------------------
 * Algorithm used: source(Book: Probablistic Robotics, page 42)
 *
 * predicted_mean = g(x_t-1,u_t)
 * predicted_cov  = G_t * prev_cov * G_t.transpose() + R_t
 *
 * K_t            = predicted_cov * H_t.transpose() * (H_t * predicted_cov * H_t.transpose() + Q_t).inverse()
 * corrected_mean = predicted_mean + K_t * (z_t - h(predicted_mean))
 * corrected_cov  = (I - K_t * H_t) * predicted_cov 
 *
 * -----------------------------------------------------------------------------------------------------
 * @param [in]  control_vector u_t.
 * @param [in]  sensor_measurement_vector z_t, new measurement from the sensor.
 * @param [out] posterior belief mean and covariance. 
 */
std::pair<Eigen::VectorXf, Eigen::VectorXf>
extended_kalman_filter::compute_ekf_corrected_state(Eigen::MatrixXf control_vector,
										      Eigen::MatrixXf sensor_measurement_vector)
{
	Eigen::VectorXf pred_mean;
	Eigen::VectorXf pred_cov;
	Eigen::MatrixXf K_t;
	Eigen::VectorXf corr_mean;
	Eigen::VectorXf corr_cov;

	std::pair<Eigen::VectorXf, Eigen::VectorXf> corr_states;

	pred_mean = calculate_ekf_predicted_mean(control_vector);
	pred_cov  = calculate_ekf_predicted_covariance(control_vector);

	K_t       = calculate_ekf_kalman_gain(pred_mean, pred_cov);
	corr_mean = calculate_ekf_corrected_mean(pred_mean, K_t, sensor_measurement_vector);
	corr_cov  = calculate_ekf_corrected_covariance(pred_mean, pred_cov, K_t);

	corr_states.first  = corr_mean;
	corr_states.second = corr_cov;

	//update the previous belief 
	m_previous_belief_mean = corr_mean;
	m_previous_belief_covariance  = corr_cov;

	return corr_states;

}



/**
 * @brief helper function to calculate predicted mean from control input,
 * state matrix, control matrix, and prior belief mean of the system.
 *
 * @details
 * It used to calculte the first step of the Kalman filter's algorithm.
 *
 * predicted_mean = g(x_t-1,u_t)
 *
 * @param [in]  control_vector u_t.
 * @param [out] predicted mean as a vector.
 */
Eigen::VectorXf extended_kalman_filter::calculate_ekf_predicted_mean(Eigen::VectorXf control_vector)
{
	Eigen::VectorXf pred_mean = m_g(m_previous_belief_mean,control_vector);
	
	return pred_mean;
}



/**
 * @brief helper function to calculate predicted covariance from state noise covariance,
 * state matrix, and prior belief covariance of the system.
 *
 * @details 
 *
 * predicted_cov  = G_t * prev_cov * G_t.transpose() + R_t
 *
 * @param [in]  control_vector u_t.
 * @param [out] predicted covariance as a vector.
 */
Eigen::VectorXf extended_kalman_filter::calculate_ekf_predicted_covariance(Eigen::VectorXf control_vector)
{
    Eigen::MatrixXf G_t = m_G(m_previous_belief_mean, control_vector); //computing jacobian of the motion model

	Eigen::VectorXf pred_cov = G_t * m_previous_belief_covariance * G_t.transpose() + m_state_noise_cov_vector;

	return pred_cov;
}


/**
 * @brief helper function for calculating kalman gain from predicted covariance,
 * sensor measurement matrix, and measurement noise covariance.
 *
 * @details
 * Formula used for kalman gain or innovation is given by:
 *
 * K_t = predicted_cov * H_t.transpose() * (H_t * predicted_cov * H_t.transpose() + Q_t).inverse()
 *
 * @param [in]  pred_cov, predicted covariance from @ref calculate_kf_predicted_covariance() function.
 * @param [out] K_t, Kalman gain.
 */
Eigen::MatrixXf extended_kalman_filter::calculate_ekf_kalman_gain(Eigen::VectorXf pred_mean, Eigen::VectorXf pred_cov)
{
    Eigen::MatrixXf H_t = m_H(pred_mean); //computing jacobian of the measurement model

	Eigen::MatrixXf K_t = pred_cov * H_t.transpose() * 
        (H_t* pred_cov * H_t.transpose() + m_measurement_noise_cov_vector).inverse();

	return K_t;
}


/**
 * @brief helper function to calculate corrected mean from predicted mean, kalman gain,
 * sensor matrix, and sensor measurement. 
 *
 * @details
 * Formula:
 *
 * corrected_mean = predicted_mean + K_t * (z_t - h(predicted_mean))
 * 
 * @param [in]  pred_mean , calculated predicted mean from @ref calculate_kf_predicted_mean() function.
 * @param [in]  k_gain , calculated kalman gain K_t from @ref calculate_kf_kalman_gain() function.
 * @param [in]  sensor_measu_vec z_t.
 * @param [out] corrected mean as a vector.
 */
Eigen::MatrixXf 
extended_kalman_filter::calculate_ekf_corrected_mean(Eigen::VectorXf pred_mean,Eigen::MatrixXf k_gain, Eigen::MatrixXf sensor_measu_vec)
{
	Eigen::VectorXf corr_mean = pred_mean + k_gain * (sensor_measu_vec - m_h(pred_mean));

	return corr_mean;
}


/**
 * @brief helper function to calculate corrected mean from kalman gain, measurement matrix,
 * and predicted covariance.
 *
 * @details
 * Formula:
 *
 * corrected_cov  = (I - K_t * H_t) * predicted_cov 
 *
 * @param [in]  pred_cov , calculated predicted covariance from @ref calculate_kf_predicted_cov() function.
 * @param [in]  kf_gain , calculated kalman gain K_t from @ref calculate_kf_kalman_gain() function.
 * @param [out] corrected mean as a vector.
 */
Eigen::MatrixXf extended_kalman_filter::calculate_ekf_corrected_covariance(Eigen::VectorXf pred_mean, Eigen::VectorXf pred_cov, Eigen::MatrixXf kf_gain)
{
    Eigen::MatrixXf H_t = m_H(pred_mean); //computing jacobian of the measurement model

	Eigen::MatrixXf corr_cov = ( Eigen::MatrixXf::Identity(pred_cov.rows(),pred_cov.cols()) -  kf_gain * H_t) * pred_cov; 

	return corr_cov;
}


/**
 * @brief function set the motion model and its jacobian from user callable objects with same signature 
 *
 * @details
 * Formula:
 *     Just storing the function to be used in computing motion model and jacobian
 *
 * @param [in] g  callable objects with a same signature. : g(x_t-1, u_t) 
 * @param [in] G  callable objects with a same signature. Jacobian matrix of g(x_t-1, u_t) 
 */
 void extended_kalman_filter::set_motion_model(std::function<Eigen::VectorXf(const Eigen::VectorXf&,const Eigen::VectorXf&)> g,
    std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> G)
{
    m_g = g;
    m_G = G;
}


/**
 * @brief function set the measurement model and its jacobian from user callable objects with same signature 
 *
 * @details
 * Formula:
 *     Just storing the function to be used in computing measurement model and jacobian
 *
 * @param [in] h  callable objects with a same signature. : h(x_t) 
 * @param [in] H  callable objects with a same signature. Jacobian matrix of h(x_t)  
 */
 void extended_kalman_filter::set_measurement_model(std::function<Eigen::VectorXf(const Eigen::VectorXf&)> h,
    std::function<Eigen::MatrixXf(const Eigen::VectorXf&)> H)
{
    m_h = h;
    m_H = H;
}



