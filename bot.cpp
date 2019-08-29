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

class Robot {
public:
	Robot(double r, double wb) {
		radius = r;
		wheelbase = wb;
		x = 0.0;
		y = 0.0;
		theta = 0.0;
	}

	void ReadConfig(const string& fileName) {
		ifstream in(fileName);
		json j;
		in >> j;
		radius = j["bot"]["radius"];
		wheelbase = j["bot"]["wheelBase"];
	}

	json GetStatus() {
		/*
			Возвращает текущее положение, ориентацию
			и скорость робота в формате json
		*/
		json status;
		status["position"]["x"] = x;
		status["position"]["y"] = y;
		status["theta"] = theta;
		status["velocity"] = v;

		return status;
	}

	void Stop() {
		/*
			Остановка и сброс цели
		*/
	}

	void Pause() {
		/*
			Остановка без сброса цели
		*/
	}

	void Resume() {
		/*
			Возобновить движение к цели
		*/
	}

	void UpdateOdometry(const double& delta_right, const double& delta_left) {
		/*
			Вычисляем дистанцию, пройденную каждым колесом и центром.
			Обновляем положение и ориентацию робота.
		*/
		double distance_right = radius * delta_right;
		double distance_left = radius * delta_left;
		double distance_center = (distance_left + distance_right) / 2.0;

		double prev_x = x;
		double prev_y = y;
		double prev_theta = theta;

		x = prev_x + distance_center * cos(prev_theta);
		y = prev_y + distance_center * sin(prev_theta);
		theta = prev_theta + (distance_right - distance_left) / wheelbase;
	}
	
	pair<double, double> UnicycleToDifferential(const double& v, const double& omega) {
		/*
			Переход от unicycle (V, omega) модели к differential drive (Vl, Vr)
		*/
		double v_r = (2 * v - omega * wheelbase) / (2 * radius);
		double v_l = (2 * v + omega * wheelbase) / (2 * radius);

		return (make_pair(v_r, v_l));
	}

	pair<double, double> GetWheelsRad() {
		/*
			Считываем положение колес в радианах
		*/
		double delta_right = 0.0;
		double delta_left = 0.0;
		return make_pair(delta_right, delta_left);
	}

	void GoTo(const double& x_dest, const double& y_dest, const double& theta_dest) {
		double omega;
		pair<double, double> delta_wheels;

		// Сначала поворот на заданный угол
		double error = atan2(theta_dest, theta);
		while (error > eps) {
			omega = Kp * error;
			v = 0;
			cmd_wheels = UnicycleToDifferential(v, omega);
			delta_wheels = GetWheelsRad();
			UpdateOdometry(delta_wheels.first, delta_wheels.second);
			error = atan2(theta_dest, theta);
		}
		// Потом едем к цели по прямой
		double distance = sqrt( pow((x_dest - x), 2) + pow((y_dest, y), 2) );
		while (distance > eps) {
			omega = 0;
			v = Kp * distance;
			cmd_wheels = UnicycleToDifferential(v, omega);
			delta_wheels = GetWheelsRad();
			UpdateOdometry(delta_wheels.first, delta_wheels.second);
			distance = sqrt(pow((x_dest - x), 2) + pow((y_dest, y), 2));
		}
	}

private:
	double radius, wheelbase; // параметры робота
	double x, y, theta; // текущее положение
	double v; // скорость движения
	pair<double, double> cmd_wheels; // задание на колеса
	double Kp; // коэффициент п-регулятора
	double eps; // погрешность
};


int main() {
	Robot b = Robot(0.1, 0.2);
	b.ReadConfig("C:\\Dev\\workspace\\TRA\\test\\src\\config.json");

	Server svr;

	svr.Post("/goto", [&](const Request& req, Response& res) {
		b.GoTo(0.1, 0.1, 0.1);
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

