#include<iostream>
#include"fuzzycontrol.h"
#include"fuzzypid.h"
using namespace std;

fuzzy a;
fuzzypid b;

void main()
{
	b.Fuzzy_Control_Matrix_Total();
	b.Fuzzy_Input(250,100,20,34);
	b.U_Calculate();
	b.U_Defuzzy();
	b.Input_Kp_Interval(0.120, 0.180);
	b.Input_Ki_Interval(0.0120, 0.0180);
	b.Input_Kd_Interval(0.00120, 0.00180);
	b.Cal_Kp_Ki_Kd(0.150, 0.0150, 0.00150);



	cout << "角度差值：" <<"      " <<b.error<< endl;
	cout << "角速度差值：    " << b.dderror<< endl;
	cout << endl;
	cout << "E对应的模糊集合：" << endl;
	for (int i = 0; i < 13; i++)
		cout << b.e1[i] << "   ";
	cout << endl;
	cout << endl;
	cout << "EC对应的模糊集合：" << endl;
	for (int j = 0; j < 13; j++)
		cout << b.ec1[j] << "   ";
	cout << endl;
	cout << endl;
	cout << "依次输出Kp、Ki、Kd输出量的模糊集合：" << endl;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 13; j++)
		{
			cout << b.U_Final[i][j] << "   ";
			if (j == 12)
				cout << endl;
		}
		cout << endl;
	}
	cout << endl;
	cout << "输出模糊矩阵解模糊化后的精确输出量：" << endl;
	cout << "U(Kp):" << b.u[0] << "       " << "U(Ki):" << b.u[1] << "       " << "U(Kd):" << b.u[2] << endl;
	cout << endl;
	cout << "Kp、Ki、Kd的变化量:" << endl;
	cout << "ΔKp：" << b.Kp_ << "    " << "ΔKi：" << b.Ki_ << "    " << "ΔKd" << b.Kd_ << endl;
	cout << endl;
	cout << "Kp、Ki、Kd的初始值:" << endl;
	cout << "Kp：" << "0.150" << "            " << "Ki：" << "0.150" << "           " << "Kd：" << "0.00150" << endl;
	cout << "计算出Kp、Ki、Kd的值:" << endl;
	cout << "Kp：" << b.kp << "         " << "Ki：" << b.ki << "        " << "Kd：" << b.kd << endl;
	


	 
}

