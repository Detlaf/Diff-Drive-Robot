// Robot.h : включаемый файл для стандартных системных включаемых файлов
// или включаемые файлы для конкретного проекта.

#pragma once

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
	STOPPED
};

struct Position {
	double x;
	double y;
	double theta;
};

class Robot {
public:
	Robot();
	void ReadConfig(const string&);
	json GetStatus() const;
	void Stop();
	void Pause();
	void Resume();
	pair<double, double> GetWheelsRad();
	void UpdateOdometry();
	pair<double, double> UnicycleToDifferential(const double&, const double&);
	void GoToGoal(const json&);
private:
	double radius, wheelbase;
	Position pos; // текущее положение
	double v, omega; // скорость движения и угловая скорость
	pair<double, double> cmd_wheels; // задание на колеса
	double Kp, Ki, Kd; // коэффициенты ПИД-регулятора
	double eps; // погрешность
	State state; // состояние робота
	double error_sum, error_prev;
};
