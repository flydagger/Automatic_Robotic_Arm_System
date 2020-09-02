/*
* Author: Yixiang Fan
* Company: Antikythera Robotics
* Data: 27/05/2018
* Modification: Multi-thread
*/

// auboi5-sdk-for-windows-x64.cpp : 定义控制台应用程序的入口点。

//#include <afxwin.h>
#include "stdafx.h"
#include "rsdef.h"
#include <string>
#include "example.h"
#include <tchar.h>
#include <stdio.h>
#include <thread>
#include <fstream>
#include <mutex>
#include "robot_control.h"
#include <thread>
//#include "server.h"
//#include <WinSock2.h>
#pragma comment(lib, "ws2_32.lib")

//#define LOCALHOST_IPADDRESS "192.168.1.13"
#define LOCALHOST_IPADDRESS "127.0.0.1"
#define ROBOT_ADDR "169.254.73.73"
#define ROBOT_PORT 8899
#define M_PI 3.14159265358979323846

RSHD g_rshd = -1; //机械臂控制上下文句柄
SOCKADDR_IN addrSrv;
SOCKET sockClient;
int port = 10000;
char message[128];
double offsetCom[2] = { 0 }; // offset values of target component
double offsetSlot[2] = { 0 }; // offset values of target slot

int client(RobotControl &, RSHD &);
int initialization(RobotControl &, RSHD &);
int move(RobotControl &, RSHD &);
int grab(RobotControl &, RSHD &);
int manual_control(RobotControl &, RSHD &);
int designating_position(RobotControl &, RSHD &);
int display(RSHD & rshd, aubo_robot_namespace::wayPoint_S *currentWayPoint);

double slot_distance = 0.3;
aubo_robot_namespace::wayPoint_S initialWayPoint;

int _tmain(int argc, _TCHAR* argv[])
{
	RobotControl rbctrl;
	WSADATA wsaData;
	memset(message, 0, sizeof(message));

	// load socket
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		printf("Fail to load socket : %d......\n", WSAGetLastError());
		return RS_FAILED;
	}

	// initialize IP and port
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(port);
	addrSrv.sin_addr.S_un.S_addr = inet_addr(LOCALHOST_IPADDRESS);

	// socket()
	sockClient = socket(AF_INET, SOCK_STREAM, 0);
	if (SOCKET_ERROR == sockClient) {
		printf("Fail to create socket : %d......\n", WSAGetLastError());
		return RS_FAILED;
	}

	// send request to server
	if (connect(sockClient, (struct  sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET)
	{
		printf("Fail to connect to server : %d......\n", WSAGetLastError());
		return RS_FAILED;
	}
	else
	{
		// After successfully initializing, RC module sends a message "2" to GUI module.
		message[0] = '2';
		send(sockClient, message, sizeof(message), 0);
		printf("Succeed to connect to GUI.\n");
	}

	//登录服务器 login robot
	if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
	{
		//启动机械臂(必须连接真实机械臂）Start robot (must connect with real robot)
		example_robotStartup(g_rshd);

		//机械臂轴动测试 Robot move joint example
		//example_moveJ(g_rshd);

		//机械臂保持当前姿态直线运动测试 Robot move line example (keeping current orientation)
		//example_moveL(g_rshd);

		//机械臂轨迹运动测试 Robot move track example
		//example_moveP(g_rshd);

		//test
		
		//move_test_1(g_rshd);
		//move_test_2(g_rshd);
		//server(g_rshd);
		client(rbctrl, g_rshd);

		//double jointAngle[aubo_robot_namespace::ARM_DOF];
		//bool IsBolck;
		//aubo_robot_namespace::robotServiceJointMove(jointAngle, IsBolck);)
		//move_test_3(g_rshd);
		//joint_test_1(g_rshd);
		//test end

		//机械臂正逆解测试 Robot forward/inverse kinematics example
		//example_ik_fk(g_rshd);

		//机械臂控制柜IO测试(必须连接真实机械臂）Control box IO example (must connect real robot)
		//example_boardIO(g_rshd);

		//机械臂工具端IO测试(必须连接真实机械臂） Control box Tool IO example (must connect real robot)
		//example_ToolIO(g_rshd);

		//实时路点信息回调函数测试 callback robot real time road point example
		//example_callbackRobotRoadPoint(g_rshd);

		//延时2两秒，观察回调函数 sleep for 2s, observe callback function
		//Sleep(2000);

		//关闭机械臂（必须连接真实机械臂） Robot shutdown (must connect real robot)
		example_robotShutdown(g_rshd);

		//退出登录 Log out
		example_logout(g_rshd);
	}

	//反初始化接口库 uninitialize 
	rs_uninitialize();

	// close socket
	closesocket(sockClient);
	WSACleanup();

	//std::system("pause");
	return 0;
}

int client(RobotControl & rbctrl, RSHD & rshd) {

	while (true) {
		memset(message, 0, sizeof(message));
		recv(sockClient, message, sizeof(message), 0);
		printf("Receive command : %s\n", message);

		double laser_distance = 0.0;
		int command = int(message[0] - 48);
		if (command == 0) {
			break;
		}

		// test
		aubo_robot_namespace::wayPoint_S currentWayPoint;
		rs_get_current_waypoint(rshd, &currentWayPoint);
		std::cout << "x = " << currentWayPoint.cartPos.position.x << ", y = " << currentWayPoint.cartPos.position.y << ", z = " << currentWayPoint.cartPos.position.z << std::endl;
		// test end

		switch (command) {
		case 1:  // grab the target, drop to the slot and return to initial position
			if (RS_SUCC != grab(rbctrl, rshd)) {
				cout << "Fail in function grab().\n" << endl;
				break;
			}
			memset(message, 0, sizeof(message));
			message[0] = '3';
			if (send(sockClient, message, sizeof(message), 0) == SOCKET_ERROR) {
				printf("Fail to send data in switch case 1.\n");
			}
			break;
		case 2:  // Reply the status of the robot.
			rbctrl.condition_monitor(rshd, sockClient);
			break;
		case 3:
			initialization(rbctrl, rshd);
			memset(message, 0, sizeof(message));
			message[0] = '3';
			if (send(sockClient, message, sizeof(message), 0) == SOCKET_ERROR) {
				printf("Fail to send data in switch case 3.\n");
			}
			break;
		case 5: 
			move(rbctrl, rshd);
			memset(message, 0, sizeof(message));
			message[0] = '4';
			if (send(sockClient, message, sizeof(message), 0) == SOCKET_ERROR) {
				printf("Fail to send data in switch case 5.\n");
			}
			break;
		case 6: // check whether the target is in the valid area. If in the valid working area, return 0, otherwise return 1.
			if (RS_SUCC == rbctrl.check_working_area(rshd, message)) {
				memset(message, 0, sizeof(message));
				message[0] = 'c';
				message[1] = '0';
				send(sockClient, message, sizeof(message), 0);
			}
			else {
				memset(message, 0, sizeof(message));
				message[0] = 'c';
				message[1] = '1';
				send(sockClient, message, sizeof(message), 0);
			}
			break;
		case 7:
			if (RS_SUCC != manual_control(rbctrl, rshd)) {
				printf("Error in manual control function.\n");
			}
			break;
		case 9:
			if (RS_SUCC != designating_position(rbctrl, rshd)) {
				printf("Error in saving position.\n");
			}
			break;
		default: break;
		}
		
	}

	return RS_SUCC;
}

int initialization(RobotControl & rbctrl, RSHD & rshd) {
	if (rbctrl.initialPosture(rshd) == false) {
		printf("Fail to initiate posture \n");
		return RS_FAILED;
	}

	// Obtain initial waypoint
	if (RS_SUCC != rs_get_current_waypoint(rshd, &initialWayPoint)) {
		std::cerr << " Fail to acquire initial waypoint. " << std::endl;
		return RS_FAILED;
	}

	return RS_SUCC;
}

int move(RobotControl & rbctrl, RSHD & rshd) {
	// move the robot to target position
	aubo_robot_namespace::wayPoint_S targetWayPoint;
	aubo_robot_namespace::wayPoint_S currentWayPoint;

	if (RS_SUCC != rs_get_current_waypoint(rshd, &currentWayPoint)) {
		// Acquiring current waypoint failed
		std::cerr << " Fail to acquire current waypoint. " << std::endl;
		return 0;
	}
	targetWayPoint = currentWayPoint;

	aubo_robot_namespace::Pos targetPosition = currentWayPoint.cartPos.position;
	aubo_robot_namespace::Ori targetOri = currentWayPoint.orientation;

	// Extract coordinates from message
	char coor_convert[10];
	strncpy_s(coor_convert, message + 3, 5);
	double h_coordinate = atof(coor_convert);
	if (int(message[2]) == int('-')) {
		h_coordinate = -h_coordinate;
	}
	memset(coor_convert, 0, sizeof(coor_convert));
	strncpy_s(coor_convert, message + 10, 5);
	double v_coordinate = atof(coor_convert);
	if (int(message[9]) == int('-')) {
		v_coordinate = -v_coordinate;
	}

	targetPosition.x += h_coordinate;
	targetPosition.y += v_coordinate;

	if (RS_SUCC != rs_inverse_kin(rshd, currentWayPoint.jointpos, &targetPosition, &targetOri, &targetWayPoint)) {
		printf("Fail to get inverse kin\n");
		return RS_FAILED;
	}
	if (RS_SUCC != rs_move_joint(rshd, targetWayPoint.jointpos)) {
		printf("Fail to move in capturePro1 \n");
		return RS_FAILED;
	}

	return RS_SUCC;
}

int grab(RobotControl & rbctrl, RSHD & rshd) {

	// Open the gripper
	if (RS_SUCC != rbctrl.release_component(rshd, sockClient)) {
		printf("Fail to open the gripper \n");
		return RS_FAILED;
	}
	else {
		printf("Succeed to open the gripper \n");
	}

	// Initialize waypoint
	aubo_robot_namespace::wayPoint_S targetWayPoint;
	aubo_robot_namespace::wayPoint_S currentWayPoint;

	// Obtain current waypoint
	if (RS_SUCC != rs_get_current_waypoint(rshd, &currentWayPoint)) {
		// Acquiring current waypoint failed
		std::cerr << " Fail to acquire current waypoint. " << std::endl;
		return RS_FAILED;
	}

	// Extract the gradient and direction of the target component from message
	char grad_convert[10];
	// Extract gradient
	strncpy_s(grad_convert, message + 3, 5);
	double gradient = atof(grad_convert);
	if (int(message[2]) == int('-')) {
		gradient = -gradient;
	}
	
	// Extract direction
	int direction = -1;
	if (int(message[9]) == int('1')) {
		direction = 1;
	}
	else if (int(message[9]) == int('0')) {
		direction = 0;
	}
	else {
		printf("Invalid direction.\n");
		return RS_FAILED;
	}

	// 60 == 1.0472
	// 120 == 2.0944
	// 180 == M_PI
	// 240 == 4.18879
	if (gradient >= -1.0472 && gradient <= 0) {  // -60 ~ 0
		gradient = gradient - 2.0944;  // g - 120
	}
	else if (gradient > 0 && gradient <= 2.0944) {  // 0 ~ 120
		gradient = gradient - 2.0944;  // -(120 - g) = g - 120
	}
	else if (gradient > 2.0944 && gradient <= M_PI) {  // 120 ~ 180
		gradient = gradient - 2.0944; // g - 120
	}
	else if (gradient >= -M_PI && gradient < -1.0472) {  // -180 ~ -60
		gradient = gradient + 4.18879;  // (180 + g) + 60 = 240 + g
	}

	// Rotate and descent robot gripper
	aubo_robot_namespace::Rpy tmp_rpy;
	if (RS_SUCC != rs_get_current_waypoint(rshd, &currentWayPoint)) {
		std::cerr << " Fail to acquire current waypoint " << std::endl;
		return RS_FAILED;
	}
	targetWayPoint = currentWayPoint;
	if (RS_SUCC != rs_quaternion_to_rpy(rshd, &currentWayPoint.orientation, &tmp_rpy)) {
		std::cerr << " Fail to convert currentWayPoint.orientation to tmp_rpy " << std::endl;
		return RS_FAILED;
	}
	tmp_rpy.rz = -gradient;
	if (RS_SUCC != rs_rpy_to_quaternion(rshd, &tmp_rpy, &targetWayPoint.orientation)) {
		std::cerr << " Fail to convert currentWayPoint.orientation to tmp_rpy " << std::endl;
		return RS_FAILED;
	}
	
	// Acquire the distance between the robot hand and target component
	double laser_distance;
	rbctrl.laserRangeFinder(laser_distance);
	if (laser_distance > 0.46) {
		printf("The dropping distance is overlength and dangerous. \n");
		return RS_FAILED;
	}
	targetWayPoint.cartPos.position.z -= (laser_distance - 0.03);  // pause at the position which is 3cm above the slot.
//	targetWayPoint.cartPos.positionVector
	if (RS_SUCC != rs_inverse_kin(rshd, currentWayPoint.jointpos, &targetWayPoint.cartPos.position, &targetWayPoint.orientation, &targetWayPoint))
	{
		printf("Fail to get inverse kin in descend to component \n");
		return RS_FAILED;
	}
	if (RS_SUCC != rs_move_joint(rshd, targetWayPoint.jointpos)) {
		printf("Fail to descend to component position\n");
		return RS_FAILED;
	}

	if (RS_SUCC != rs_get_current_waypoint(rshd, &currentWayPoint)) {
		std::cerr << " Fail to acquire current waypoint " << std::endl;
		return RS_FAILED;
	}
	targetWayPoint = currentWayPoint;
	targetWayPoint.cartPos.position.z -= 0.03;
	if (RS_SUCC != rs_inverse_kin(rshd, currentWayPoint.jointpos, &targetWayPoint.cartPos.position, &targetWayPoint.orientation, &targetWayPoint))
	{
		printf("Fail to get inverse kin in descend to component \n");
		return RS_FAILED;
	}
	if (RS_SUCC != rs_move_line(rshd, targetWayPoint.jointpos)) {
		printf("Fail to descend to component position\n");
		return RS_FAILED;
	}

	// Grab the component
	if (RS_SUCC != rbctrl.grasp_component(rshd, sockClient)) {
		printf("Fail to grab the component \n");
		return RS_FAILED;
	}

	//// test
	//rs_move_joint(rshd, rbctrl.initial_posotion.jointpos);
	//rbctrl.release_component(rshd, sockConn);
	//system("pause");
	//return RS_SUCC;
	//// test end

	// Ascend robot hand : 0.03m
	if (RS_SUCC != rs_get_current_waypoint(rshd, &currentWayPoint)) {
		std::cerr << " Fail to acquire current waypoint while rotating and descenting" << std::endl;
		return RS_FAILED;
	}
	targetWayPoint = currentWayPoint;
	targetWayPoint.cartPos.position.z += 0.03;
	if (RS_SUCC != rs_inverse_kin(rshd, currentWayPoint.jointpos, &targetWayPoint.cartPos.position, &targetWayPoint.orientation, &targetWayPoint))
	{
		printf("Fail to get inverse kin in ascend to component above \n");
		return RS_FAILED;
	}
	if (RS_SUCC != rs_move_joint(rshd, targetWayPoint.jointpos)) {
		printf("Fail to ascend to component above\n");
		return RS_FAILED;
	}

	//// Move to the ready position and keep the component one side upward
	//if (RS_SUCC != rs_get_current_waypoint(rshd, &currentWayPoint)) {
	//	std::cerr << " Fail to acquire current waypoint. " << std::endl;
	//	return RS_FAILED;
	//}

	//targetWayPoint = currentWayPoint;
	//targetWayPoint = rbctrl.default_slot_position_1;
	//targetWayPoint.orientation = rbctrl.default_slot_position_1.orientation;
	//targetWayPoint.cartPos.position = rbctrl.default_slot_position_1.cartPos.position;
	//targetWayPoint.cartPos.position.z += 0.05;  // the position is 5cm higher than the default slot

	//if (RS_SUCC != rs_inverse_kin(rshd, currentWayPoint.jointpos, &targetWayPoint.cartPos.position, &targetWayPoint.orientation, &targetWayPoint)) {
	//	printf("Fail to get inverse kinematics \n");
	//	return RS_FAILED;
	//}
	//else {
	//	printf("Succeed to get inverse kinematics before slot \n");
	//}

	//rs_move_joint(rshd, targetWayPoint.jointpos);
	if (RS_SUCC != rs_move_joint(rshd, rbctrl.default_slot_position_1.jointpos)) {
		printf("Fail to move joint before slot \n");
		return RS_FAILED;
	}

	// Release component
	if (RS_SUCC != rbctrl.release_component(rshd, sockClient)) {
		printf("Fail to release component to slot \n");
		return RS_FAILED;
	}

	// The gripper steps back to avoid the conflict.
	//currentWayPoint = targetWayPoint;
	//targetWayPoint.jointpos[0] -= 0.174533;  // test modified
	//if (RS_SUCC != rs_inverse_kin(rshd, currentWayPoint.jointpos, &targetWayPoint.cartPos.position, &targetWayPoint.orientation, &targetWayPoint))
	//{
	//	printf("Fail to get inverse kin in ascend to slot above \n");
	//	return RS_FAILED;
	//}
	//if (RS_SUCC != rs_move_joint(rshd, targetWayPoint.jointpos)) {
	//	printf("Fail to ascend to slot above\n");
	//	return RS_FAILED;
	//}

	// Clamp the gripper
	if (RS_SUCC != rbctrl.grasp_component(rshd, sockClient)) {
		printf("Fail to clamp the gripper after slot \n");
		return RS_FAILED;
	}

	// Return to preparation posture
	if (RS_SUCC != rs_move_joint(rshd, rbctrl.initial_posotion.jointpos)) {
		printf("Fail to return to preparation position \n");
		return RS_FAILED;
	}

	return RS_SUCC;
}

int display(RSHD & rshd, aubo_robot_namespace::wayPoint_S *currentWayPoint) {
	if (currentWayPoint == nullptr) {
		printf("Invalid parameter: current waypoint.\n");
		return -1;
	}

	aubo_robot_namespace::Rpy rpy;
	rs_quaternion_to_rpy(rshd, &currentWayPoint->orientation, &rpy);

	cout << "\nCurrent jointangles: " << endl
		<< "Joint 0: " << currentWayPoint->jointpos[0] * 180 / M_PI << " degree" << endl
		<< "Joint 1: " << currentWayPoint->jointpos[1] * 180 / M_PI << " degree" << endl
		<< "Joint 2: " << currentWayPoint->jointpos[2] * 180 / M_PI << " degree" << endl
		<< "Joint 3: " << currentWayPoint->jointpos[3] * 180 / M_PI << " degree" << endl
		<< "Joint 4: " << currentWayPoint->jointpos[4] * 180 / M_PI << " degree" << endl
		<< "Joint 5: " << currentWayPoint->jointpos[5] * 180 / M_PI << " degree" << endl
		<< "--------------------------------" << endl
		<< "RPY.x = " << rpy.rx * 180 / M_PI << " degree" << endl
		<< "RPY.y = " << rpy.ry * 180 / M_PI << " degree" << endl
		<< "RPY.z = " << rpy.rz * 180 / M_PI << " degree" << endl
		<< "--------------------------------" << endl
		<< "Orientation w = " << currentWayPoint->orientation.w << endl
		<< "Orientation x = " << currentWayPoint->orientation.x << endl
		<< "Orientation y = " << currentWayPoint->orientation.y << endl
		<< "Orientation z = " << currentWayPoint->orientation.z << endl
		<< "--------------------------------" << endl
		<< "Position X = " << currentWayPoint->cartPos.position.x << endl
		<< "Position Y = " << currentWayPoint->cartPos.position.y << endl
		<< "Position Z = " << currentWayPoint->cartPos.position.z << endl
		<< "--------------------------------" << endl;

	return RS_SUCC;
}

int manual_control(RobotControl & rbctrl, RSHD & rshd) {
	// move the robot to target position
	aubo_robot_namespace::wayPoint_S targetWayPoint;
	aubo_robot_namespace::wayPoint_S currentWayPoint;

	if (RS_SUCC != rs_get_current_waypoint(rshd, &currentWayPoint)) {
		// Acquiring current waypoint failed
		std::cerr << " Fail to acquire current waypoint. " << std::endl;
		return 0;
	}
	targetWayPoint = currentWayPoint;

	int flag = 1;
	if (message[3] == '-') {
		flag = -1;
	}

	char c_figure[6];
	memset(c_figure, 0, sizeof(c_figure));
	strncpy_s(c_figure, message + 4, 5);
	double d_figure = atof(c_figure);

	if (message[1] >= '0' && message[1] <= '5') {
		targetWayPoint.jointpos[int(message[1]) - 48] = d_figure * flag * M_PI / 180;
	}
	else {
		if (message[1] == 'x') {
			targetWayPoint.cartPos.position.x = d_figure * flag;
		}
		else if (message[1] == 'y') {
			targetWayPoint.cartPos.position.y = d_figure * flag;
		}
		else if (message[1] == 'z') {
			targetWayPoint.cartPos.position.z = d_figure * flag;
		}

		if (RS_SUCC != rs_inverse_kin(rshd, currentWayPoint.jointpos, &targetWayPoint.cartPos.position, &targetWayPoint.orientation, &targetWayPoint)) {
			printf("Fail to get inverse kinematics in manual control \n");
			return RS_FAILED;
		}
	}
	
	if (RS_SUCC != rs_move_joint(rshd, targetWayPoint.jointpos)) {
		printf("Fail to move joint in manual control \n");
		return RS_FAILED;
	}
	
	return RS_SUCC;
}

int designating_position(RobotControl & rbctrl, RSHD & rshd) {
	if (message[1] == '0') {
		rbctrl.user_designated_slot_position = rbctrl.default_slot_position;
		rbctrl.user_designated_initial_posotion = rbctrl.initial_posotion;
	}
	else if (message[1] == '1') {  // designate the position of the slot.
		if (RS_SUCC != rs_get_current_waypoint(rshd, &rbctrl.user_designated_slot_position)) {
			cerr << " Fail to get waypoint in function designating_position 1. " << endl;
			return RS_FAILED;
		}
	}
	else if (message[1] == '2') {  // designate the position of the valid working area.
		if (RS_SUCC != rs_get_current_waypoint(rshd, &rbctrl.working_area[message[2] - 48])) {
			cerr << " Fail to get waypoint in function designating_position 2. " << endl;
			return RS_FAILED;
		}
		if (message[2] == '2') {
			aubo_robot_namespace::Pos tmp_pos;
			tmp_pos.x = (rbctrl.working_area[0].cartPos.position.x + rbctrl.working_area[1].cartPos.position.x) / 2;
			tmp_pos.y = (rbctrl.working_area[0].cartPos.position.y + rbctrl.working_area[1].cartPos.position.y) / 2;
			tmp_pos.z = rbctrl.working_area[2].cartPos.position.z;
			if (RS_SUCC != rs_inverse_kin(rshd, rbctrl.working_area[2].jointpos, &tmp_pos, &rbctrl.working_area[2].orientation, &rbctrl.user_designated_initial_posotion)) {
				cerr << "Fail in inverse kin in function designating_position 2 2. " << endl;
				return RS_FAILED;
			}
		}
	}

	return RS_SUCC;
}

