#include "Robot.h"

using namespace Rbt;

#pragma region DHFrame 實作

DHFrame::DHFrame()
	   :Parent_ID(-1), Self_ID(-1), FrameMode(-1)
{
	printf("DHFrame Error：初始沒有指定參數\n");
	assert(false);
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, double _a, double _alpha, double _d, double _theta)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), FrameMode(ACTIVE_FRAME)
{
	// 錯誤檢查
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error：Parent_ID 與 Self_ID 相同\n");
		printf("Parent_ID=%d, Self_ID=%d, ACTIVE_FRAME\n", Parent_ID, Self_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f\n", _a, _alpha, _d, _theta);
		assert(false);
	}

	// 參數初始化
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = 1;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	Front_T = Eigen::Matrix4d::Zero();
	back_T = Eigen::Matrix4d::Zero();

	Praent_DHNode = NULL;
	DependActive_DHNode = this;
	DependActive_ID = Self_ID;

	qList_Index = -1;

	UpdateMat_ptr = &DHFrame::UpdateMat1;
	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, double _a, double _alpha, double _d, double _theta, Eigen::Matrix4d *_Front_T, Eigen::Matrix4d *_back_T)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), FrameMode(ACTIVE_FRAME)
{
	// 錯誤檢查
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error：Parent_ID 與 Self_ID 相同\n");
		printf("Parent_ID=%d, Self_ID=%d, ACTIVE_FRAME\n", Parent_ID, Self_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f\n", _a, _alpha, _d, _theta);
		if(_Front_T != NULL)
			std::cout << "Front_T = \n" << *_Front_T << std::endl;
		if(_back_T != NULL)
			std::cout << "back_T = \n" << *_back_T << std::endl;

		assert(false);
	}

	// 參數初始化
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = 1;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	
	if(_Front_T != NULL)
		Front_T = (*_Front_T);
	else
		Front_T = Eigen::Matrix4d::Zero();
	
	if(_back_T != NULL)
		back_T = (*_back_T);
	else
		back_T = Eigen::Matrix4d::Zero();

	
	Praent_DHNode = NULL;
	DependActive_DHNode = this;
	DependActive_ID = Self_ID;

	qList_Index = -1;

	if(_Front_T == NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat1;
	else if(_Front_T != NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat2;
	else if(_Front_T == NULL && _back_T != NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat3;
	else  // _Front_T != NULL && _back_T != NULL
		UpdateMat_ptr = &DHFrame::UpdateMat4;

	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, int _DependActive_ID, double _a, double _alpha, double _d, double _theta, double _ratio)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), DependActive_ID(_DependActive_ID), FrameMode(PASSIVE_FRAME)
{
	// 錯誤檢查
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error：Parent_ID 與 Self_ID 相同\n");
		printf("Parent_ID=%d, Self_ID=%d, DependActive_ID=%d, PASSIVE_FRAME\n", Parent_ID, Self_ID, DependActive_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, ratio=%f\n", _a, _alpha, _d, _theta, _ratio);
		assert(false);
	}

	// 參數初始化
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = _ratio;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	Front_T = Eigen::Matrix4d::Zero();
	back_T = Eigen::Matrix4d::Zero();

	Praent_DHNode = NULL;
	DependActive_DHNode = NULL;
	
	qList_Index = -1;

	UpdateMat_ptr = &DHFrame::UpdateMat1;
	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, int _DependActive_ID, double _a, double _alpha, double _d, double _theta, double _ratio, Eigen::Matrix4d *_Front_T, Eigen::Matrix4d *_back_T)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), DependActive_ID(_DependActive_ID), FrameMode(PASSIVE_FRAME)
{
	// 錯誤檢查
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error：Parent_ID 與 Self_ID 相同\n");
		printf("Parent_ID=%d, Self_ID=%d, DependActive_ID=%d, PASSIVE_FRAME\n", Parent_ID, Self_ID, DependActive_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, ratio=%f\n", _a, _alpha, _d, _theta, _ratio);
		if(_Front_T != NULL)
			std::cout << "Front_T = \n" << *_Front_T << std::endl;
		if(_back_T != NULL)
			std::cout << "back_T = \n" << *_back_T << std::endl;

		assert(false);
	}

	// 參數初始化
	a = _a;		alpha = _alpha;		d = _d;		theta = _theta;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = _ratio;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = Eigen::Matrix4d::Zero();
	
	if(_Front_T != NULL)
		Front_T = (*_Front_T);
	else
		Front_T = Eigen::Matrix4d::Zero();
	
	if(_back_T != NULL)
		back_T = (*_back_T);
	else
		back_T = Eigen::Matrix4d::Zero();

	
	Praent_DHNode = NULL;
	DependActive_DHNode = NULL;

	qList_Index = -1;

	if(_Front_T == NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat1;
	else if(_Front_T != NULL && _back_T == NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat2;
	else if(_Front_T == NULL && _back_T != NULL)
		UpdateMat_ptr = &DHFrame::UpdateMat3;
	else  // _Front_T != NULL && _back_T != NULL
		UpdateMat_ptr = &DHFrame::UpdateMat4;

	HaveJointLimit = false;
}

DHFrame::DHFrame(int _Parent_ID, int _Self_ID, const Eigen::Matrix4d& _ConstFrame)
	   :Parent_ID(_Parent_ID), Self_ID(_Self_ID), FrameMode(CONST_FRAME)
{
	// 錯誤檢查
	if(Parent_ID == Self_ID)
	{	
		printf("DHFrame Error：Parent_ID 與 Self_ID 相同\n");
		printf("Parent_ID=%d, Self_ID=%d, CONST_FRAME\n", Parent_ID, Self_ID);
		std::cout << "ConstFrame = \n" << _ConstFrame << std::endl;
		assert(false);
	}

	// 參數初始化
	a = 0;		alpha = 0;		d = 0;		theta = 0;
	joint_min = 0;		joint_max = 0;
	cmd = 0;
	ratio = 0;
	
	TFMat = Eigen::Matrix4d::Identity();
	ConstFrame = _ConstFrame;
	Front_T = Eigen::Matrix4d::Zero();
	back_T = Eigen::Matrix4d::Zero();

	Praent_DHNode = NULL;
	DependActive_DHNode = this;
	DependActive_ID = Self_ID;

	qList_Index = -1;

	UpdateMat_ptr = &DHFrame::UpdateMat5;
	HaveJointLimit = false;
}

DHFrame::~DHFrame()
{

}

void DHFrame::Set_JointLimit(double _joint_min, double _joint_max)
{
	if(FrameMode == ACTIVE_FRAME)
	{
		joint_min = _joint_min;
		joint_max = _joint_max;
		HaveJointLimit = true;
	}
}

void DHFrame::Set_Praent_DHNode(DHFrame *_Praent_DHNode)
{
	Praent_DHNode = _Praent_DHNode;
}

DHFrame* DHFrame::Get_Praent_DHNode() const
{
	return Praent_DHNode;
}

void DHFrame::Set_DependActive_DHNode(DHFrame *_DependActive_DHNode)
{
	if(FrameMode == PASSIVE_FRAME)
	{
		DependActive_DHNode = _DependActive_DHNode;
	}
}

DHFrame* DHFrame::Get_DependActive_DHNode() const
{
	return DependActive_DHNode;
}

// 根據ChildID由小排到大
bool SortChildID(const DHFrame* Child_A, const DHFrame* Child_B)
{
	return (Child_A->Self_ID < Child_B->Self_ID); 
}

void DHFrame::Add_Child_DHNode(DHFrame *_Child_DHNode)
{
	
	// Self_ID 不能重複
	for(int i=0; i < Child_DHNode.size(); i++)
	{
		if(Child_DHNode[i]->Self_ID == _Child_DHNode->Self_ID)
		{
			printf("DHFrame Error：Self_ID=%d 加入重複子Self_ID %d", Self_ID, _Child_DHNode->Self_ID);
			assert(false);
		}
	}

	Child_DHNode.push_back(_Child_DHNode);

	std::sort(Child_DHNode.begin(), Child_DHNode.end(), SortChildID);
}

const std::vector<DHFrame*>& DHFrame::Get_Child_DHNode()
{
	return Child_DHNode;
}

const Eigen::Matrix4d& DHFrame::Get_TFMat()
{
	return TFMat;
}

int DHFrame::Get_DependActive_ID() const
{
	return DependActive_ID;
}

void DHFrame::Set_qList_Index(int index)
{
	qList_Index = index;
}

int DHFrame::Get_qList_Index() const
{
	return qList_Index;
}

double DHFrame::Get_Ratio() const
{
	return ratio;
}

void DHFrame::Get_FrameInfo()
{
	printf("FrameInfo：\n");
	
	if(FrameMode == ACTIVE_FRAME)
	{	
		printf("Parent_ID=%d, Self_ID=%d, ACTIVE_FRAME\n", Parent_ID, Self_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, cmd=%f\n", a, alpha, d, theta, cmd);
		
		if(HaveJointLimit)
			printf("joint_min = %f, joint_max = %f\n", joint_min, joint_max);
	}
	else if(FrameMode == PASSIVE_FRAME)
	{	
		printf("Parent_ID=%d, Self_ID=%d, DependActive_ID=%d, PASSIVE_FRAME\n", Parent_ID, Self_ID, DependActive_ID);
		printf("a=%f, alpha=%f, d=%f, theta=%f, ratio=%f, cmd=%f\n", a, alpha, d, theta, ratio, cmd);
	}
	else // FrameMode == CONST_FRAME
	{	
		printf("Parent_ID=%d, Self_ID=%d, CONST_FRAME\n", Parent_ID, Self_ID);	
	}
	printf("qList_Index = %d\n", qList_Index);

	printf("Child_DHNode：\n");
	for(int i=0; i < Child_DHNode.size(); i++)
		printf("%d ", Child_DHNode[i]->Self_ID);
	printf("\n");

	std::cout << "TFMat = \n" << TFMat << std::endl;
}


void DHFrame::Set_Command(double New_Cmd)
{
	if(FrameMode == ACTIVE_FRAME)
		cmd = New_Cmd;
}

void DHFrame::Set_Command()
{
	if(FrameMode == PASSIVE_FRAME)
		cmd = ratio * DependActive_DHNode->Get_Command();
}

void DHFrame::Check_JointLimit(double& New_Cmd)
{
	if(HaveJointLimit)
	{
		if(New_Cmd > joint_max)
			New_Cmd = joint_max;
		else if(New_Cmd < joint_min)
			New_Cmd = joint_min;

		cmd = New_Cmd;
	}
}

double DHFrame::Get_Command() const
{
	return cmd;
}

// 以遞迴的方式去完成FK更新，並記錄在Frame_List
void DHFrame::Get_FK(const Eigen::Matrix4d& Parent_T)
{



}


inline void DHFrame::UpdateMat(double New_theta)  // 更新 TFMat (對前一軸)
{
	TFMat.block<3,4>(0,0) <<  cos(New_theta), -sin(New_theta)*cos(alpha),  sin(New_theta)*sin(alpha),   a*cos(New_theta),
							  sin(New_theta),  cos(New_theta)*cos(alpha),  -cos(New_theta)*sin(alpha),  a*sin(New_theta),
							  0,			   sin(alpha),                 cos(alpha),                  d;
}

void DHFrame::UpdateMat1(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * TFMat
{
	UpdateMat(theta + cmd);   // 更新 TFMat (相對前一軸)
	TFMat = Parent_T * TFMat; // 相對Root Frame
}

void DHFrame::UpdateMat2(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * Front_T * TFMat
{
	UpdateMat(theta + cmd);             // 更新 TFMat (對前一軸)
	TFMat = Parent_T * Front_T * TFMat; // 相對Root Frame
}

void DHFrame::UpdateMat3(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * TFMat * back_T
{
	UpdateMat(theta + cmd);            // 更新 TFMat (對前一軸)
	TFMat = Parent_T * TFMat * back_T; // 相對Root Frame
}

void DHFrame::UpdateMat4(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * Front_T * TFMat * back_T
{
	UpdateMat(theta + cmd);                      // 更新 TFMat (對前一軸)
	TFMat = Parent_T * Front_T * TFMat * back_T; // 相對Root Frame
}

void DHFrame::UpdateMat5(const Eigen::Matrix4d& Parent_T) // TFMat = Parent_T * Const Frame
{
	TFMat = Parent_T * ConstFrame;
}




#pragma endregion


#pragma region EndEffector 實作




#pragma endregion


#pragma region Robot 實作

Robot::Robot()
{
	Active_DOF = 0;
	Passive_DOF = 0;
	Root_DHFrame = NULL;
}

Robot::~Robot()
{
	// 釋放空間
	for(int i=0, n=ActiveFrame_Store.size(); i < n ; i++)
		delete[] ActiveFrame_Store[i];
	
	for(int i=0, n=PassiveFrame_Store.size(); i < n ; i++)
		delete[] PassiveFrame_Store[i];

	for(int i=0, n=ConstFrame_Store.size(); i < n ; i++)
		delete[] ConstFrame_Store[i];
}

void Robot::AddActiveFrame(DHFrame* ActiveFrame)
{
	ActiveFrame_Store.push_back(ActiveFrame);
	Active_DOF++;
}

void Robot::AddPassiveFrame(DHFrame* PassiveFrame)
{
	PassiveFrame_Store.push_back(PassiveFrame);
	Passive_DOF++;
}

void Robot::AddConstFrame(DHFrame* ConstFrame)
{
	ConstFrame_Store.push_back(ConstFrame);
}

bool Build_Robot()
{
	assert(true);
	return true;
}

#pragma endregion



#pragma region Robot 相關數學運算函式

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
inline Eigen::Matrix4d Rbt::DH2TFMat(double a, double alpha, double d, double theta)
{
	Eigen::Matrix4d T;

	T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),   a*cos(theta),
		 sin(theta),  cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),  a*sin(theta),
		 0,           sin(alpha),             cos(alpha),              d,
		 0,           0,                      0,                       1;

	return T;
}

inline void Rbt::DH2TFMat(Eigen::Matrix4d& Dest_T, double a, double alpha, double d, double theta)
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