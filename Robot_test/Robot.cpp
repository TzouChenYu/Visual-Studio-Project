#include "Robot.h"

using namespace Rbt;
















#pragma region Robot 相關數學運算函式

Eigen::Matrix4d Rbt::DH2TFMat(double a, double alpha, double d, double theta)
{
	Eigen::Matrix4d T;

	T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),   a*cos(theta),
		 sin(theta),  cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),  a*sin(theta),
		 0,           sin(alpha),             cos(alpha),              d,
		 0,           0,                      0,                       1;

	return T;
}

void Rbt::DH2TFMat(Eigen::Matrix4d& Dest_T, double a, double alpha, double d, double theta)
{
		
	Dest_T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),   a*cos(theta),
			  sin(theta),  cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),  a*sin(theta),
			  0,           sin(alpha),             cos(alpha),              d,
			  0,           0,                      0,                       1;

}

Eigen::Vector3d Rbt::Get_Orientation(const Eigen::Matrix3d& Src_R)
{
	Eigen::Vector3d k; // [kx,ky,kz]

	double theta = acos( (Src_R(0,0)+Src_R(1,1)+Src_R(2,2)-1)/2 );

	// theta = 0 (deg) ==> singular
	if(abs(theta) <= 0.0000001)
	{
		k << 0, 0, 0;
	}
	else if(abs(theta-pi) <= 0.0000001) // theta = 180 (deg) ==> singular
	{
		Eigen::Matrix3d temp = (Src_R + Eigen::Matrix3d::Identity()) / 2;
		
		// 令 kx >= 0
		k(0) = sqrt(temp(0,0));

		if(k(0) == 0)
		{
			if(temp(2,1) > 0) // ky*kz > 0
			{
				k(1) = sqrt(temp(1,1)); //ky
				k(2) = sqrt(temp(2,2)); //kZ
			}
			else
			{
				k(1) = sqrt(temp(1,1)); //ky
				k(2) = -sqrt(temp(2,2)); //kZ
			}
		}
		else  // kx > 0
		{
			if(temp(1,0) > 0) // kx*ky > 0
				k(1) = sqrt(temp(1,1)); //ky
			else
				k(1) = -sqrt(temp(1,1)); //ky


			if(temp(2,0) > 0) // kx*kz > 0
				k(2) = sqrt(temp(2,2)); //kz
			else
				k(2) = -sqrt(temp(2,2)); //kz
		}
	}
	else
	{
		k << Src_R(2,1)-Src_R(1,2), Src_R(0,2)-Src_R(2,0), Src_R(1,0)-Src_R(0,1);
		k = k / (2*sin(theta));
	}

	return theta*k;
};

void Rbt::Get_Orientation(Eigen::Vector3d& Dest_Ori, const Eigen::Matrix3d& Src_R)
{
	double theta = acos( (Src_R(0,0)+Src_R(1,1)+Src_R(2,2)-1)/2 );

	// theta = 0 (deg) ==> singular
	if(abs(theta) <= 0.0000001)
	{
		Dest_Ori << 0, 0, 0;
	}
	else if(abs(theta-pi) <= 0.0000001) // theta = 180 (deg) ==> singular
	{
		Eigen::Matrix3d temp = (Src_R + Eigen::Matrix3d::Identity()) / 2;
		
		// 令 kx >= 0
		Dest_Ori(0) = sqrt(temp(0,0));

		if(Dest_Ori(0) == 0)
		{
			if(temp(2,1) > 0) // ky*kz > 0
			{
				Dest_Ori(1) = sqrt(temp(1,1)); //ky
				Dest_Ori(2) = sqrt(temp(2,2)); //kZ
			}
			else
			{
				Dest_Ori(1) = sqrt(temp(1,1)); //ky
				Dest_Ori(2) = -sqrt(temp(2,2)); //kZ
			}
		}
		else  // kx > 0
		{
			if(temp(1,0) > 0) // kx*ky > 0
				Dest_Ori(1) = sqrt(temp(1,1)); //ky
			else
				Dest_Ori(1) = -sqrt(temp(1,1)); //ky


			if(temp(2,0) > 0) // kx*kz > 0
				Dest_Ori(2) = sqrt(temp(2,2)); //kz
			else
				Dest_Ori(2) = -sqrt(temp(2,2)); //kz
		}
	}
	else
	{
		Dest_Ori << Src_R(2,1)-Src_R(1,2), Src_R(0,2)-Src_R(2,0), Src_R(1,0)-Src_R(0,1);
		Dest_Ori = Dest_Ori / (2*sin(theta));
	}

	Dest_Ori = theta*Dest_Ori;
}

void Rbt::Get_PosOri(Eigen::VectorXd& Dest_P, const Eigen::Matrix4d& Src_T)
{
	if(Dest_P.rows() != 6)
		Dest_P.resize(6);

	Dest_P << Src_T(0,3), Src_T(1,3), Src_T(2,3), Get_Orientation(Src_T.topLeftCorner(3,3));
}

void Rbt::Get_TFMat(Eigen::Matrix4d& Dest_T, const Eigen::VectorXd& Src_P)
{
	Eigen::Vector3d k = Src_P.bottomLeftCorner(3,1);
		
	double theta = k.norm();
		
	k = k / theta; //變單位向量

	Eigen::Matrix3d temp;
	temp << 0, -k(2), k(1),
			k(2), 0, -k(0),
			-k(1), k(0), 0;

	Eigen::Matrix3d R = cos(theta)*Eigen::Matrix3d::Identity() + (1-cos(theta))*k*k.transpose() + sin(theta)*temp;


	Dest_T << R, Src_P.topLeftCorner(3,1),
			    0, 0, 0, 1;
}

#pragma endregion