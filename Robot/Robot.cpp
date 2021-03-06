﻿// Robot.cpp: определяет точку входа для приложения.
//

#include "Robot.h"
using namespace std;
using json = nlohmann::json;
using namespace httplib;

Robot::Robot() {
		pos.x = 0;
		pos.y = 0;
		pos.theta = 0;
		v = 0;
		omega = 0;
		error_sum = 0;
		error_prev = 0;
		Kp = 10;
		Ki = 0.01;
		Kd = 3;
}

void Robot::ReadConfig(const string& fileName) {
	ifstream in;
	json j;

	cout << "Read config" << endl;

	try {
		in.open(fileName);
		in >> j;
		radius = j["bot"]["radius"];
		wheelbase = j["bot"]["wheelBase"];
		eps = j["epsilon"];
	}
	catch (const ifstream::failure& ex) {
		cout << "Exception while opening the config file: " << ex.what() << endl;
	}
}

	json Robot::GetStatus() const {
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

	void Robot::Stop() {
		/*
			Остановка и сброс цели
		*/
		cmd_wheels = UnicycleToDifferential(0, 0);
		state = State::STOPPED;
	}

	void Robot::Pause() {
		/*
			Остановка без сброса цели
		*/
		cmd_wheels = UnicycleToDifferential(0, 0);
		state = State::PAUSED;
	}

	void Robot::Resume() {
		/*
			Возобновить движение к цели
		*/
		if (state == State::PAUSED) {
			state = State::MOVING;
		}
	}

	pair<double, double> Robot::GetWheelsRad() {
		/*
			Считываем положение колес в радианах
			0.5 в качестве "заглушки"
		*/
		double delta_right = 0.5;
		double delta_left = 0.5;
		return make_pair(delta_right, delta_left);
	}

	void Robot::UpdateOdometry() {
		/*
			Вычисляем дистанцию, пройденную каждым колесом, на основании углов поворота.
			Обновляем положение и ориентацию робота.
		*/
		pair<double, double> delta_wheels;
		delta_wheels = GetWheelsRad();
		double distance_right = radius * delta_wheels.first;
		double distance_left = radius * delta_wheels.second;
		double distance_center = (distance_left + distance_right) / 2.0;

		double prev_x = pos.x;
		double prev_y = pos.y;
		double prev_theta = pos.theta;

		pos.x = prev_x + distance_center * cos(prev_theta);
		pos.y = prev_y + distance_center * sin(prev_theta);
		pos.theta = prev_theta + (distance_right - distance_left) / wheelbase;
	}

	pair<double, double> Robot::UnicycleToDifferential(const double& v, const double& omega) {
		/*
			Переход от unicycle (V, omega) модели к differential drive (Vl, Vr)
			V_r, V_l - задание скорости вращения колес
		*/
		double v_r = (2 * v - omega * wheelbase) / (2 * radius);
		double v_l = (2 * v + omega * wheelbase) / (2 * radius);

		return (make_pair(v_r, v_l));
	}

	void Robot::GoToGoal(const json& goal) {
		/*
			Контур управления движением робота: поворот на заданный угол с дальнейшим движением к цели по прямой.
			Входные параметры:
			 - заданные ориентация и координаты цели (x, y, theta);
			 - максимальная скорость движения.
		*/
		Position pos_dest;
		pos_dest.x = goal["x"];
		pos_dest.y = goal["y"];
		pos_dest.theta = goal["theta"];
		double Vmax = goal["Vmax"];

		state = State::MOVING;

		while (state == State::MOVING) {

			double error = atan2(sin(pos_dest.theta - pos.theta), cos(pos_dest.theta - pos.theta));
			// ПИД-регулятор
			omega = Kp * error + Ki * (error_sum + error) + Kd * (error - error_prev);
			v = Vmax;
			cmd_wheels = UnicycleToDifferential(v, omega);
			UpdateOdometry();

			double distance = sqrt(pow((pos_dest.x - pos.x), 2) + pow((pos_dest.y, pos.y), 2));
			if (error <= eps && distance <= eps) {
				// цель достигнута
				state = State::STOPPED;
			}

			error_sum += error;
			error_prev = error;
		}
	}


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
	b.ReadConfig("config.json");

	// тестовая точка назначения и ориентация
	json goal = SetDestination(5, 5, _Pi / 4, 1.5);

	Server svr;

	svr.Post("/goto", [&](const Request& req, Response& res) {
		b.GoToGoal(goal);
	});

	svr.Get("/stop", [&](const Request& req, Response& res) {
		b.Stop();
		goal = json({});
		svr.stop();
	});

	svr.Get("/pause", [&](const Request& req, Response& res) {
		b.Pause();
	});

	svr.Get("/resume", [&](const Request& req, Response& res) {
		b.Resume();
		b.GoToGoal(goal);
	});

	svr.Get("/status", [&](const Request& req, Response& res) {
		json status = b.GetStatus();
		res.set_content(status, "application/json");
	});

	svr.listen("localhost", 8888);

	return 0;
}