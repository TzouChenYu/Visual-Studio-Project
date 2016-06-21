#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen\Dense>

#define _CRT_SECURE_NO_WARNINGS


#define pi 3.14159265359
#define Deg2Rad(theta) (theta*pi/180) 
#define Rad2Deg(theta) (theta*180/pi)

namespace Rbt
{
	/*******************************
	建立標準的 DH Model，參數說明：
	[   a   ] 為 Z(i-1)與Z(i)的共垂線距離
	[ alpha ] 為Z(i-1)沿著X(i)方向旋轉到Z(i)的夾角
	[   d   ] 為frame(i-1)的原點到共垂線與Z(i-1)交點的距離(沿著Z(i-1)方向為正)
	[ theta ] 為X(i-1)沿著Z(i-1)方向旋轉到X(i)的夾角

	T = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),   a*cos(theta),]
        [ sin(theta),  cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),  a*sin(theta),]
        [ 0,           sin(alpha),             cos(alpha),              d,			 ]
        [ 0,           0,                      0,                       1            ]
	*******************************/
	class MyClass
	{
	public:
		MyClass();
		~MyClass();

	private:

	};


	class Standard_DH
	{
	private:
		double a, alpha, d, theta, theta_offset;
		double joint_min, joint_max;
		Eigen::Matrix4d Front_T; // 用來存放 pseudo frame 到 主動frame 之間的 const T
		Eigen::Matrix4d back_T;  // 用來存放 pseudo frame 到 主動frame 之間的 const T

		int Parent_ID;
		int Self_ID;
		bool IsActiveFrame;

	public:
		// 建構子
		// 產生TransFormation Matrix，單位(m, rad, m, rad)
		Standard_DH(int Parent_ID, int Self_ID, double _a, double _alpha, double _d, double _ini_theta, double _theta_offset);

		// 解構子
		~Standard_DH();

		// 設定 Joint 旋轉範圍，單位rad
		void SetJointLimit(double joint_min, double joint_max);

		bool JointLimit();
		
		int Get_ParentID();
		int Get_SelfID();
	};
	
	
	class Robot
	{
	private:
		int Active_DOF;
		int Passive_DOF;

	public:
		// 建構子
		Robot();

		// 解構子
		~Robot();

		

		
	};
}

#pragma region Robot 相關數學運算函式

namespace Rbt
{
	// 從Standard DH Model 轉成 TransFormation Matrix，單位(m,rad,m,rad)
	inline Eigen::Matrix4d DH2TFMat(double a, double alpha, double d, double theta);
	
	inline void DH2TFMat(Eigen::Matrix4d& Dest_T, double a, double alpha, double d, double theta);


	// 從Rotation 轉成等效 Orientation = theta*k
	Eigen::Vector3d Get_Orientation(const Eigen::Matrix3d& Src_R);
	
	void Get_Orientation(Eigen::Vector3d& Dest_Ori, const Eigen::Matrix3d& Src_R);


	// 從Transformation Matrix 獲得 P = [Position, Orientation]
	void Get_PosOri(Eigen::VectorXd& Dest_P, const Eigen::Matrix4d& Src_T);
	

	// 從 P = [Position, Orientation] 轉換成 Transformation Matrix
	void Get_TFMat(Eigen::Matrix4d& Dest_T, const Eigen::VectorXd& Src_P);


}

#pragma endregion