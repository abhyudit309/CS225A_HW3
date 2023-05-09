#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

//********* saturation function for Q4 *********//

double sat(double x){
	if (abs(x) <= 1.0){
		return x;
	}
	else {
		return x/abs(x);
	}
}

//********************************//

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// ************** CODE ADDED *************** //

	// Q1
	VectorXd q_desired = VectorXd::Zero(dof);
	VectorXd g = VectorXd::Zero(dof);
	Vector3d x = Vector3d::Zero();
	Vector3d x_dot = Vector3d::Zero();
	Vector3d xd = Vector3d::Zero();
	Vector3d xd_dot = Vector3d::Zero();
	Vector3d xd_ddot = Vector3d::Zero();
	double Amp = 0.1;
	double w = M_PI;
	// Q2
	VectorXd q_lower(dof);
	q_lower << -165.0, -100.0, -165.0, -170.0, -165.0, 0.0, -165.0; // in degrees
	q_lower *= M_PI/180.0;
	VectorXd q_upper(dof);
	q_upper << 165.0, 100.0, 165.0, -30.0, 165.0, 210.0, 165.0; // in degrees
	q_upper *= M_PI/180.0;
	VectorXd q_mid = 0.5 * (q_lower + q_upper);
	VectorXd tau_mid = VectorXd::Zero(dof);
	VectorXd tau_damp = VectorXd::Zero(dof);
	// Q3
	MatrixXd R_desired(3, 3);
	R_desired(0, 0) = cos(M_PI/3);  R_desired(0, 1) = 0.0;  R_desired(0, 2) = sin(M_PI/3);
	R_desired(1, 0) = 0.0;          R_desired(1, 1) = 1.0;  R_desired(1, 2) = 0.0;
	R_desired(2, 0) = -sin(M_PI/3); R_desired(2, 1) = 0.0;  R_desired(2, 2) = cos(M_PI/3);
	Matrix3d R = Matrix3d::Zero();
	Vector3d omega = Vector3d::Zero();
	Vector3d dphi;
	MatrixXd J0 = MatrixXd::Zero(6, dof);
	MatrixXd Lambda0 = MatrixXd::Zero(6, 6);
	MatrixXd N0 = MatrixXd::Zero(dof, dof);
	// Q4
	double V_max = 0.1;
	// writing trajectories to .txt file
	ofstream file;
	file.open("../../hw3/data_files/q4-b.txt"); // change this according to the question

	// ************** END ********************* //

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;

		robot->Jv(Jv, link_name, pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->nullspaceMatrix(N, Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->gravityVector(g);
		robot->position(x, link_name, pos_in_link);
		robot->linearVelocity(x_dot, link_name, pos_in_link);
		robot->rotation(R, link_name);
		robot->angularVelocity(omega, link_name);
		robot->J_0(J0, link_name, pos_in_link);
		robot->taskInertiaMatrix(Lambda0, J0);
		robot->nullspaceMatrix(N0, J0);

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 100.0;
			double kv = 20.0;
			double kpj = 50.0;
			double kvj = 14.0;

			xd = Vector3d(0.3, 0.1, 0.5) + Amp * Vector3d(sin(w * time), cos(w * time), 0.0);
			xd_dot = Amp * w * Vector3d(cos(w * time), -sin(w * time), 0.0);
			xd_ddot = -Amp * w * w * Vector3d(sin(w * time), cos(w * time), 0.0);

			// auto F = Lambda * (-kp * (x - xd) - kv * x_dot); // for part (a)
			auto F = Lambda * (xd_ddot - kp * (x - xd) - kv * (x_dot - xd_dot)); // for part (c)

			command_torques = Jv.transpose() * F + N.transpose() * (-kpj * (robot->_q - q_desired) - kvj * robot->_dq) + g;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 100.0;
			double kv = 20.0;
			double kpj = 50.0;
			double kvj = 14.0;

			Vector3d x_desired = Vector3d(-0.65, -0.45, 0.7);

			auto F = Lambda * (-kp * (x - x_desired) - kv * x_dot);

			tau_mid = -kpj * (robot->_q - q_mid);
			tau_damp = -kvj * robot->_dq;

			// command_torques = Jv.transpose() * F + N.transpose() * tau_damp + g; // for part (d)
			// command_torques = Jv.transpose() * F + N.transpose() * tau_mid + N.transpose() * tau_damp + g; // for parts (e) and (f)
			command_torques = Jv.transpose() * F + tau_mid + N.transpose() * tau_damp + g; // for part (g)
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 100.0;
			double kv = 20.0;
			double kpj = 50.0;
			double kvj = 14.0;

			Vector3d x_desired = Vector3d(0.6, 0.3, 0.5);
			dphi = Vector3d::Zero();
			Vector3d Ri, Rdi;

			for (int i = 0; i < 3; i++){
				Ri = R.col(i);
				Rdi = R_desired.col(i);
				dphi = dphi - 0.5 * Ri.cross(Rdi);
			}

			Vector3d vec1, vec2;
			VectorXd vec(6);
			vec1 = -kp * (x - x_desired) - kv * x_dot;
			vec2 = -kp * dphi - kv * omega;
			vec << vec1, vec2;

			auto F = Lambda0 * vec;

			command_torques = J0.transpose() * F - N0.transpose() * (kvj * robot->_dq) + g;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double kp = 200.0;
			double kv = 28.0;
			double kpj = 50.0;
			double kvj = 14.0;

			Vector3d x_desired = Vector3d(0.6, 0.3, 0.4);
			Vector3d x_desired_dot;

			// auto F = Lambda * (-kp * (x - x_desired) - kv * x_dot); // for part (a)

			x_desired_dot = -(kp/kv) * (x - x_desired);
			double nu = sat(V_max/x_desired_dot.norm());
			auto F = Lambda * (-kv * (x_dot - nu * x_desired_dot)); // for part (b)

			command_torques = Jv.transpose() * F + N.transpose() * robot->_M * (-kpj * (robot->_q - q_desired) - kvj * robot->_dq) + g;
		}

		// writing trajectory to txt file
		if (controller_number == QUESTION_3){
			file << time << "\t" << robot->_q.transpose() << "\t" << x.transpose() << "\t" << dphi.transpose() << "\n";
		}
		else {
			file << time << "\t" << robot->_q.transpose() << "\t" << x.transpose() << "\t" << x_dot.transpose() << "\n";
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	file.close();
	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
