#include "pch.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <utility>
#include "json.hpp"
#include "httplib.h"

using namespace std;
using json = nlohmann::json;
using namespace httplib;

enum class State {
	MOVING,
	PAUSED,
	STOPPED,
	GOAL_REACHED
};

struct Position {
	double x;
	double y;
	double theta;
};

class Robot {
public:
	Robot() {
		pos.x = 0.0;
		pos.y = 0.0;
		pos.theta = 0.0;
		v = 0.0;
	}

	void ReadConfig(const string& fileName) {
		ifstream in;
		json j;

		try {
			in.open(fileName);
			in >> j;
			radius = j["bot"]["radius"];
			wheelbase = j["bot"]["wheelBase"];
			eps = j["epsilon"];
		}
		catch (const ifstream::failure& ex) {
			cout << "Exception while opening the config file" << endl;
		}
	}

	json GetStatus() {
		/*
			Возвращает текущее положение, ориентацию
			и скорость робота в формате json
		*/
		json status;
		status["position"]["x"] = pos.x;
		status["position"]["y"] = pos.y;
		status["theta"] = pos.theta;
		status["velocity"] = v;

		return status;
	}

	void Stop() {
		/*
			Остановка и сброс цели
		*/
		state = State::STOPPED;
		v = 0.0;
	}

	void Pause() {
		/*
			Остановка без сброса цели
		*/
		state = State::PAUSED;
		v = 0.0;
	}

	void Resume() {
		/*
			Возобновить движение к цели
		*/
		state = State::MOVING;
	}

	void UpdateOdometry(const double& delta_right, const double& delta_left) {
		/*
			Вычисляем дистанцию, пройденную каждым колесом и центром.
			Обновляем положение и ориентацию робота.
			Входные параметры:
			 - угол поворота правого колеса;
			 - угол поворота левого колеса.
		*/
		double distance_right = radius * delta_right;
		double distance_left = radius * delta_left;
		double distance_center = (distance_left + distance_right) / 2.0;

		double prev_x = pos.x;
		double prev_y = pos.y;
		double prev_theta = pos.theta;

		pos.x = prev_x + distance_center * cos(prev_theta);
		pos.y = prev_y + distance_center * sin(prev_theta);
		pos.theta = prev_theta + (distance_right - distance_left) / wheelbase;
	}
	
	pair<double, double> UnicycleToDifferential(const double& v, const double& omega) {
		/*
			Переход от unicycle (V, omega) модели к differential drive (Vl, Vr)
			V_r, V_l - задание скорости вращения колес
		*/
		double v_r = (2 * v - omega * wheelbase) / (2 * radius);
		double v_l = (2 * v + omega * wheelbase) / (2 * radius);

		return (make_pair(v_r, v_l));
	}

	pair<double, double> GetWheelsRad() {
		/*
			Считываем положение колес в радианах
		*/
		double delta_right = 0.5;
		double delta_left = 0.5;
		return make_pair(delta_right, delta_left);
	}

	void GoTo(const json& goal) {
		/*
			Контур управления движением робота: поворот на заданный угол с дальнейшим движением к цели по прямой.
			Входные параметры:
			 - заданные ориентация и координаты цели (x, y, theta);
			 - максимальная скорость движения.
		*/
		double omega;
		pair<double, double> delta_wheels;
		Position pos_dest;
		pos_dest.x = goal["x"];
		pos_dest.y = goal["y"];
		pos_dest.theta = goal["theta"];
		double Vmax = goal["Vmax"];

		while (state == State::MOVING) {
			double error = atan2(sin(pos_dest.theta - pos.theta), cos(pos_dest.theta - pos.theta));
			while (error > eps) {
				omega = Kp * error;
				v = 0;
				cmd_wheels = UnicycleToDifferential(v, omega);
				delta_wheels = GetWheelsRad();
				UpdateOdometry(delta_wheels.first, delta_wheels.second);
				error = atan2(sin(pos_dest.theta - pos.theta), cos(pos_dest.theta - pos.theta));
			}
			double distance = sqrt(pow((pos_dest.x - pos.x), 2) + pow((pos_dest.y, pos.y), 2));
			while (distance > eps) {
				v = Kp * distance;
				if (v > Vmax) {
					v = Vmax;
				}
				cmd_wheels = UnicycleToDifferential(v, omega);
				delta_wheels = GetWheelsRad();
				UpdateOdometry(delta_wheels.first, delta_wheels.second);
				distance = sqrt(pow((pos_dest.x - pos.x), 2) + pow((pos_dest.y, pos.y), 2));
			}
			if (error <= eps && distance <= eps) {
				state = State::GOAL_REACHED;
			}
		}
	}

private:
	double radius, wheelbase; // параметры робота
	Position pos; // текущее положение
	double v; // скорость движения
	pair<double, double> cmd_wheels; // задание на колеса
	double Kp; // коэффициент П-регулятора
	double eps; // погрешность
	State state; // состояние робота
};

json SetDestination(const double& x, const double& y, const double& theta, const double& Vmax) {
	json dest;

	dest["x"] = x;
	dest["y"] = y;
	dest["theta"] = theta;
	dest["Vmax"] = Vmax;

	return dest;
}

int main() {
	Robot b = Robot();
	b.ReadConfig("C:\\Dev\\workspace\\TRA\\test\\src\\config.json");

	json goal = SetDestination(5, 5, pi/4, 1.5);

	Server svr;

	svr.Post("/goto", [&](const Request& req, Response& res) {
		b.GoTo(goal);
	});

	svr.Get("/stop", [&](const Request& req, Response& res) {
		b.Stop();
		svr.stop();
	});

	svr.Get("/pause", [&](const Request& req, Response& res) {
		b.Pause();
	});

	svr.Get("/resume", [&](const Request& req, Response& res) {
		b.Resume();
	});

	svr.Get("/status", [&](const Request& req, Response& res) {
		json status = b.GetStatus();
		res.set_content(status, "application/json");
	});

	svr.listen("localhost", 8888);

	return 0;
}

