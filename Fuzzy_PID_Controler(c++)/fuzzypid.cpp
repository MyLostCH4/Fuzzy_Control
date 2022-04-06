#include "fuzzypid.h"

fuzzypid::fuzzypid() {}

fuzzypid::~fuzzypid() {}


//取两个数之间数值较小的数
float fuzzypid::min(float a, float b)
{
	if (a < b)
		return a;
	else
		return b;
}

//取两个数之间数值较大的数
float fuzzypid::max(float a, float b)
{
	if (a > b)
		return a;
	else
		return b;
}
//四舍五入取整
int fuzzypid::round_float(float number)
{
	return (number > 0.0) ? (number + 0.5) : (number - 0.5);
}


//计算单模糊规则下总模糊矩阵
void fuzzypid::Fuzzy_Control_Matrix_cal(int a, int b, int c)
{
	int k = 0;
	//E和EC模糊关系计算
	for (int j = 0; j < 13; j++)
	{
		for (int i = 0; i < 13; i++)
		{
			FC_Matrix[j][i] = min(E[a][j], EC[b][i]);
			FC_Matrix_Use[k] = FC_Matrix[j][i];
			k++;
		}
	}

	//E、EC和U模糊关系计算
	for (int i = 0; i < 13 * 13; i++)
	{
		for (int j = 0; j < 13; j++)
		{
			FC_Matrix_Final[i][j] = min(FC_Matrix_Use[i], U[c][j]);
		}
	}
}


//测试函数，用于验证算法
void fuzzypid::Fuzzy_Control_Matrix_cal_1()
{
	int h = 0;
	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < 3; i++)
		{
		   v[j][i] = min(y[j],k[i]);
		   vv[h] = v[j][i];
		   h++;
		}
	}
	for(int i=0;i<3*3;i++)
		for (int j = 0; j < 3; j++)
		{
			vvv[i][j] = min(vv[i],p[j]);
		}
}


//计算所有模糊规则下Kp、Ki、Kd关于E、EC、U的总模糊关系矩阵
void fuzzypid::Fuzzy_Control_Matrix_Total()
{
	//分别计算Kp Ki Kd的总模糊关系矩阵
	for (int m = 0; m < 3; m++)
	{

		for (int i = 0; i < 7; i++)
		{
			for (int j = 0; j < 8; j++)
			{
				if (rules[m][i][j] != 666)
					Fuzzy_Control_Matrix_cal(i, j, rules[m][i][j]);
				if (rules[m][i][j] == 666)
					continue;
				for (int a = 0; a < 13 * 13; a++)
					for (int b = 0; b < 13; b++)
					{
						Final[m][a][b] = max(Final[m][a][b], FC_Matrix_Final[a][b]);
					}
			}
		}
	}
}


//对输入量进行模糊化处理
void fuzzypid::Fuzzy_Input(float target_e, float current_value_e, float target_ec, float current_value_ec)
{
	error = current_value_e - target_e;
	dderror = current_value_ec - target_ec;
	e = error / 10;
	ec = dderror / 20;

	//找到e对应的模糊集合的状态
	if (e <= -5)
		for (int i = 0; i < 13; i++)
			e1[i] = E[ENL][i];
	if (e > -5 && e <= -3)
		for (int i = 0; i < 13; i++)
			e1[i] = E[ENM][i];
	if (e > -3 && e <= -1)
		for (int i = 0; i < 13; i++)
			e1[i] = E[ENS][i];
	if (e > -1 && e <= 1)
		for (int i = 0; i < 13; i++)
			e1[i] = E[EZO][i];
	if (e > 1 && e <= 3)
		for (int i = 0; i < 13; i++)
			e1[i] = E[EPS][i];
	if (e > 3 && e <= 5)
		for (int i = 0; i < 13; i++)
			e1[i] = E[EPM][i];
	if (e > 5)
		for (int i = 0; i < 13; i++)
			e1[i] = E[EPL][i];

	//找到ec对应的模糊集合的状态
	if (ec <= -5)
		for (int i = 0; i < 13; i++)
			ec1[i] = EC[ECNL][i];
	if (ec > -5 && ec <= -3)
		for (int i = 0; i < 13; i++)
			ec1[i] = EC[ECNM][i];
	if (ec > -3 && ec <= -1)
		for (int i = 0; i < 13; i++)
			ec1[i] = EC[ECNS][i];
	if (ec > -1 && ec <= 1)
		for (int i = 0; i < 13; i++)
			ec1[i] = EC[ECZO][i];
	if (ec > 1 && ec <= 3)
		for (int i = 0; i < 13; i++)
			ec1[i] = EC[ECPS][i];
	if (ec > 3 && ec <= 5)
		for (int i = 0; i < 13; i++)
			ec1[i] = EC[ECPM][i];
	if (ec > 5)
		for (int i = 0; i < 13; i++)
			ec1[i] = EC[ECPL][i];
}


//计算输出量U的模糊关系矩阵
void fuzzypid::U_Calculate()
{
	//计算E和EC的模糊关系矩阵
	int k = 0;
	for (int j = 0; j < 13; j++)
	{
		for (int i = 0; i < 13; i++)
		{
			fc_matrix[j][i] = min(e1[j],ec1[i]);
			fc_matrix_use[k] = fc_matrix[j][i];
			k++;
		}
	}

	//由 E和EC确定的模糊关系矩阵 和 总模糊关系矩阵 反解出输出量U的模糊关系矩阵
	for (int m = 0; m < 3; m++)
	{
		float zuida;
		float use[13*13];
		for (int w = 0; w < 13; w++)
		{
			for (int f = 0; f < 13 * 13; f++)
				use[f] = min(fc_matrix_use[f], Final[m][f][w]);
			zuida = use[0];
			for (int f = 0; f < 13 * 13; f++)
			{
				if (use[f] >= zuida)
					zuida = use[f];
			}
			U_Final[m][w] = zuida;
	    }
	}
}


//解模糊化处理——加权平均法
void fuzzypid::U_Defuzzy()
{
	int a = 0;
	float u_total[3] = {0,0,0};

	for (int m = 0; m < 3; m++)
	{
		for (int i = -6; i <= 6; i++)
		{
			u[m] = u[m] + i * U_Final[m][a];
			a++;
		}
		for (int i = 0; i < 13; i++)
		{
			u_total[m] = u_total[m] + U_Final[m][i];
		}
		u[m] = u[m] / u_total[m];
	}
}


//根据E和EC计算Kp、Ki、Kd的值
void fuzzypid::Cal_Kp_Ki_Kd(float Kp, float Ki, float Kd)
{
	Kp_ = (u[0] / u_max) * (Kp_max / 2);
	Ki_ = (u[1] / u_max) * (Ki_max / 2);
	Kd_ = (u[2] / u_max) * (Kd_max / 2);

	kp = Kp + Kp_;
	ki = Ki + Ki_;
	kd = Kd + Kd_;
}
