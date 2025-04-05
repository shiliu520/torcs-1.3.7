/***************************************************************************
 
    file                 : SimpleDriver.cpp
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#define BUFSIZE 1000
#include "SimpleDriver.h"
#include "SimpleLogger.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#define LOG(logger, level, message) logger.logMessage(level, __FILE__, __LINE__, message)

/* Gear Changing Constants*/
const int SimpleDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int SimpleDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };

/* Stuck constants*/
const int SimpleDriver::stuckTime = 25;
const float SimpleDriver::stuckAngle = .523598775; //PI/6

/* Accel and Brake Constants*/
const float SimpleDriver::maxSpeedDist=70;
const float SimpleDriver::maxSpeed=150;
const float SimpleDriver::sin5 = 0.08716;
const float SimpleDriver::cos5 = 0.99619;

/* Steering constants*/
const float SimpleDriver::steerLock=0.366519 ;
const float SimpleDriver::steerSensitivityOffset=80.0;
const float SimpleDriver::wheelSensitivityCoeff=1;

/* ABS Filter Constants */
const float SimpleDriver::wheelRadius[4]={0.3306,0.3306,0.3276,0.3276};
const float SimpleDriver::absSlip=2.0;
const float SimpleDriver::absRange=3.0;
const float SimpleDriver::absMinSpeed=3.0;

/* Clutch constants */
const float SimpleDriver::clutchMax=0.5;
const float SimpleDriver::clutchDelta=0.05;
const float SimpleDriver::clutchRange=0.82;
const float SimpleDriver::clutchDeltaTime=0.02;
const float SimpleDriver::clutchDeltaRaced=10;
const float SimpleDriver::clutchDec=0.01;
const float SimpleDriver::clutchMaxModifier=1.3;
const float SimpleDriver::clutchMaxTime=1.5;

LogRecorder logger("TorcsOutput.log");

int
SimpleDriver::getGear(CarState &cs)
{

    int gear = cs.getGear();
    int rpm  = cs.getRpm();

    // if gear is 0 (N) or -1 (R) just return 1
    if (gear<1)
        return 1;
    // check if the RPM value of car is greater than the one suggested
    // to shift up the gear from the current one
    if (gear <6 && rpm >= gearUp[gear-1])
        return gear + 1;
    else
    	// check if the RPM value of car is lower than the one suggested 
    	// to shift down the gear from the current one
        if (gear > 1 && rpm <= gearDown[gear-1])
            return gear - 1;
        else // otherwhise keep current gear
            return gear;
}

float
SimpleDriver::getSteer(CarState &cs)
{
	// steering angle is compute by correcting the actual car angle w.r.t. to track 
	// axis [cs.getAngle()] and to adjust car position w.r.t to middle of track [cs.getTrackPos()*0.5]
    float targetAngle=(cs.getAngle()-cs.getTrackPos()*0.5);
    // at high speed reduce the steering command to avoid loosing the control
    if (cs.getSpeedX() > steerSensitivityOffset)
        return targetAngle/(steerLock*(cs.getSpeedX()-steerSensitivityOffset)*wheelSensitivityCoeff);
    else
        return (targetAngle)/steerLock;

}
float
SimpleDriver::getAccel(CarState &cs)
{
    // checks if car is out of track
    if (cs.getTrackPos() < 1 && cs.getTrackPos() > -1)
    {
        // reading of sensor at +5 degree w.r.t. car axis
        float rxSensor=cs.getTrack(10);
        // reading of sensor parallel to car axis
        float cSensor=cs.getTrack(9);
        // reading of sensor at -5 degree w.r.t. car axis
        float sxSensor=cs.getTrack(8);

        float targetSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (cSensor>maxSpeedDist || (cSensor>=rxSensor && cSensor >= sxSensor))
            targetSpeed = maxSpeed;
        else
        {
            // approaching a turn on right
            if(rxSensor>sxSensor)
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = rxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }
            // approaching a turn on left
            else
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = sxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }

        }

        float acc_cmd = 2/(1+exp(cs.getSpeedX() - targetSpeed)) - 1;
        if (acc_cmd < 0 && acc_cmd >= -0.2) acc_cmd = 0.0;
        std::ostringstream oss;
        oss << "targetSpeed: " << targetSpeed << ", currentSpeed: " << cs.getSpeedX() << ", acc: " << acc_cmd;
        LOG(logger, LogLevel::INFO, oss.str().c_str());

        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        return acc_cmd;
    }
    else
        return 0.3; // when out of track returns a moderate acceleration command

}

// 函数用于找到向量中最大值对应的索引
template<typename T>
int findMaxIndex(const std::vector<T>& data) {
    if (data.empty()) {
        return -1; // 如果向量为空，返回 -1
    }
    auto maxIt = std::max_element(data.begin(), data.end());
    return std::distance(data.begin(), maxIt);
}

// 生成从 start 到 end-1 的索引序列
std::vector<int> arange(int start, int end) {
    std::vector<int> result;
    for (int i = start; i < end; ++i) {
        result.push_back(i);
    }
    return result;
}

// 多项式拟合函数
std::vector<float> fit_polynomial(const std::vector<float>& x, const std::vector<float>y, int degree) {
    int n = x.size();
    Eigen::MatrixXd A(n, degree + 1);
    Eigen::VectorXd b(n);

    // 构建矩阵 A 和向量 b
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = std::pow(x[i], j);
        }
        b(i) = y[i];
    }

    // 使用最小二乘法求解多项式系数
    Eigen::VectorXd coeffs = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    std::vector<float> result;
    for (int i = 0; i <= degree; ++i) {
        result.push_back(coeffs(i));
    }
    return result;
}

// 计算拟合多项式在指定点的值
std::vector<float> evaluate_polynomial(const std::vector<float>& coeffs, const std::vector<float>& x_fit_dense) {
    std::vector<float> result;
    for (float x : x_fit_dense) {
        float y = 0.0;
        int degree = coeffs.size() - 1;
        for (int i = degree; i >= 0; --i) {
            y = y * x + coeffs[i];
        }
        result.push_back(y);
    }
    return result;
}

// 计算两个向量对应元素的均值
std::vector<float> compute_average(const std::vector<float>& left, const std::vector<float>& right) {
    std::vector<float> average;
    size_t size = std::min(left.size(), right.size());
    for (size_t i = 0; i < size; ++i) {
        average.push_back((left[i] + right[i]) / 2.0);
    }
    return average;
}



CarControl
SimpleDriver::wDrive(CarState cs)
{
    float angle = cs.getAngle();
    float trackPos = cs.getTrackPos();
    char buffer[BUFSIZE];
    int offset = 0, max_index = 0;
    static float track_angle[] = {90, 75, 60, 45, 30, 20, 15, 10, 5, 0, -5, -10, -15, -20, -30, -45, -60, -75, -90};
    float bound_x[TRACK_SENSORS_NUM], bound_y[TRACK_SENSORS_NUM];
    std::vector<float> track_data;
    std::vector<int> left_indices;
    std::vector<int> right_indices;
    static float steer_prev = 0.0;

    offset += snprintf(buffer + offset, BUFSIZE - offset,
            "angle: %f, trackPos: %f, ", angle, trackPos);
    for (int i = 0; i < TRACK_SENSORS_NUM; i++)
    {
        offset += snprintf(buffer + + offset, BUFSIZE - offset,
                (i < TRACK_SENSORS_NUM -1)?"track_%d: %f, ":"track_%d: %f", i,
                cs.getTrack(i));
        bound_x[i] = cs.getTrack(i) * cos(track_angle[i] * PI / 180);
        bound_y[i] = cs.getTrack(i) * sin(track_angle[i] * PI / 180);
        track_data.push_back(cs.getTrack(i));
    }
    LOG(logger, LogLevel::INFO, buffer);

    max_index = findMaxIndex(track_data);
    if (max_index != -1) {
        // 生成左索引序列
        left_indices = arange(0, max_index);
        // 生成右索引序列
        right_indices = arange(max_index + 1, static_cast<int>(track_data.size()));
    }
    // 预先确定向量大小
    std::vector<float> x_bound_left(left_indices.size());
    std::vector<float> y_bound_left(left_indices.size());
    std::vector<float> x_bound_right(right_indices.size());
    std::vector<float> y_bound_right(right_indices.size());

    // 提取左边界数据
    for (size_t i = 0; i < left_indices.size(); ++i) {
        int index = left_indices[i];
        x_bound_left[i] = bound_x[index];
        y_bound_left[i] = bound_y[index];
    }

    // 提取右边界数据
    for (size_t i = 0; i < right_indices.size(); ++i) {
        int index = right_indices[i];
        x_bound_right[i] = bound_x[index];
        y_bound_right[i] = bound_y[index];
    }

    std::vector<float> coeffs_left = fit_polynomial(x_bound_left, y_bound_left, 3);
    std::vector<float> coeffs_right = fit_polynomial(x_bound_right, y_bound_right, 3);
    std::vector<float> x_fit_dense;
    for (int i = 0; i <= 350; i += 1) {
        float value = static_cast<float>(i) / 10.0;
        x_fit_dense.push_back(value);
    }

    std::vector<float> y_fit_dense_left = evaluate_polynomial(coeffs_left, x_fit_dense);
    std::vector<float> y_fit_dense_right = evaluate_polynomial(coeffs_right, x_fit_dense);
    std::vector<float> y_fit_dense_center = compute_average(y_fit_dense_left, y_fit_dense_right);

    std::ostringstream oss;
# define DEBUG_CENTER_LINE_FIT (0)
# if DEBUG_CENTER_LINE_FIT
    for (size_t i = 0; i < 7; i++)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        else
        {
            oss << "y_fit_dense_right: ";
        }

        oss << y_fit_dense_right[i];
    }
    std::strcpy(buffer, oss.str().c_str());
#endif
#undef DEBUG_CENTER_LINE_FIT
    oss << "vx: " << cs.getSpeedX() / 3.6 << ", delta: " << steer_prev;
    oss << ", vy: " << cs.getSpeedY() / 3.6  << ", yaw_rate: " << cs.getYawRate();
    oss << ", x: " << cs.getX() << ", y: " << cs.getY() << ", theta: ";
    oss << std::atan2(cs.getSpeedGlobalY(), cs.getSpeedGlobalX());
    oss << ", fuel: " << cs.getFuel();
    oss << ", yaw: " << cs.getYaw();
    std::strcpy(buffer, oss.str().c_str());
    LOG(logger, LogLevel::INFO, buffer);

	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > stuckAngle )
    {
		// update stuck counter
        stuck++;
    }
    else
    {
    	// if not stuck reset stuck counter
        stuck = 0;
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > stuckTime)
    {
    	/* set gear and sterring command assuming car is 
    	 * pointing in a direction out of track */
    	
    	// to bring car parallel to track axis
        float steer = - cs.getAngle() / steerLock; 
        int gear=-1; // gear R
        
        // if car is pointing in the correct direction revert gear and steer  
        if (cs.getAngle()*cs.getTrackPos()>0)
        {
            gear = 1;
            steer = -steer;
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc (1.0,0.0,gear,steer,clutch);
        return cc;
    }

    else // car is not stuck
    {
    	// compute accel/brake command
        float accel_and_brake = getAccel(cs);
        // compute gear 
        int gear = getGear(cs);
        // compute steering
        float steer = getSteer(cs);
        

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        
        // set accel and brake from the joint accel/brake command 
        float accel,brake;
        if (accel_and_brake>0)
        {
            accel = accel_and_brake;
            brake = 0;
        }
        else
        {
            accel = 0;
            // apply ABS to brake
            brake = filterABS(cs,-accel_and_brake);
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc(accel,brake,gear,steer,clutch);
        steer_prev = steer * steerLock;
        return cc;
    }
}

float
SimpleDriver::filterABS(CarState &cs,float brake)
{
	// convert speed to m/s
	float speed = cs.getSpeedX() / 3.6;
	// when spedd lower than min speed for abs do nothing
    if (speed < absMinSpeed)
        return brake;
    
    // compute the speed of wheels in m/s
    float slip = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        slip += cs.getWheelSpinVel(i) * wheelRadius[i];
    }
    // slip is the difference between actual speed of car and average speed of wheels
    slip = speed - slip/4.0f;
    // when slip too high applu ABS
    if (slip > absSlip)
    {
        brake = brake - (slip - absSlip)/absRange;
    }
    
    // check brake is not negative, otherwise set it to zero
    if (brake<0)
    	return 0;
    else
    	return brake;
}

void
SimpleDriver::onShutdown()
{
    cout << "Bye bye!" << endl;
}

void
SimpleDriver::onRestart()
{
    cout << "Restarting the race!" << endl;
}

void
SimpleDriver::clutching(CarState &cs, float &clutch)
{
  double maxClutch = clutchMax;

  // Check if the current situation is the race start
  if (cs.getCurLapTime()<clutchDeltaTime  && stage==RACE && cs.getDistRaced()<clutchDeltaRaced)
    clutch = maxClutch;

  // Adjust the current value of the clutch
  if(clutch > 0)
  {
    double delta = clutchDelta;
    if (cs.getGear() < 2)
	{
      // Apply a stronger clutch output when the gear is one and the race is just started
	  delta /= 2;
      maxClutch *= clutchMaxModifier;
      if (cs.getCurLapTime() < clutchMaxTime)
        clutch = maxClutch;
	}

    // check clutch is not bigger than maximum values
	clutch = min(maxClutch,double(clutch));

	// if clutch is not at max value decrease it quite quickly
	if (clutch!=maxClutch)
	{
	  clutch -= delta;
	  clutch = max(0.0,double(clutch));
	}
	// if clutch is at max value decrease it very slowly
	else
		clutch -= clutchDec;
  }
}

void
SimpleDriver::init(float *angles)
{

	// set angles as {-90,-75,-60,-45,-30,20,15,10,5,0,5,10,15,20,30,45,60,75,90}

	for (int i=0; i<5; i++)
	{
		angles[i]=-90+i*15;
		angles[18-i]=90-i*15;
	}

	for (int i=5; i<9; i++)
	{
			angles[i]=-20+(i-5)*5;
			angles[18-i]=20-(i-5)*5;
	}
	angles[9]=0;
}
