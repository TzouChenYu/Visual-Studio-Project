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
	�إ߼зǪ� DH Model�A�Ѽƻ����G
	[   a   ] �� Z(i-1)�PZ(i)���@���u�Z��
	[ alpha ] ��Z(i-1)�u��X(i)��V�����Z(i)������
	[   d   ] ��frame(i-1)�����I��@���u�PZ(i-1)���I���Z��(�u��Z(i-1)��V����)
	[ theta ] ��X(i-1)�u��Z(i-1)��V�����X(i)������

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
		Eigen::Matrix4d Front_T; // �ΨӦs�� pseudo frame �� �D��frame ������ const T
		Eigen::Matrix4d back_T;  // �ΨӦs�� pseudo frame �� �D��frame ������ const T

		int Parent_ID;
		int Self_ID;
		bool IsActiveFrame;

	public:
		// �غc�l
		// ����TransFormation Matrix�A���(m, rad, m, rad)
		Standard_DH(int Parent_ID, int Self_ID, double _a, double _alpha, double _d, double _ini_theta, double _theta_offset);

		// �Ѻc�l
		~Standard_DH();

		// �]�w Joint ����d��A���rad
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
		// �غc�l
		Robot();

		// �Ѻc�l
		~Robot();

		

		
	};
}

#pragma region Robot �����ƾǹB��禡

namespace Rbt
{
	// �qStandard DH Model �ন TransFormation Matrix�A���(m,rad,m,rad)
	inline Eigen::Matrix4d DH2TFMat(double a, double alpha, double d, double theta);
	
	inline void DH2TFMat(Eigen::Matrix4d& Dest_T, double a, double alpha, double d, double theta);


	// �qRotation �ন���� Orientation = theta*k
	Eigen::Vector3d Get_Orientation(const Eigen::Matrix3d& Src_R);
	
	void Get_Orientation(Eigen::Vector3d& Dest_Ori, const Eigen::Matrix3d& Src_R);


	// �qTransformation Matrix ��o P = [Position, Orientation]
	void Get_PosOri(Eigen::VectorXd& Dest_P, const Eigen::Matrix4d& Src_T);
	

	// �q P = [Position, Orientation] �ഫ�� Transformation Matrix
	void Get_TFMat(Eigen::Matrix4d& Dest_T, const Eigen::VectorXd& Src_P);


}

#pragma endregion