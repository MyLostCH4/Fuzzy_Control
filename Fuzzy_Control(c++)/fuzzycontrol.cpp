#include"fuzzycontrol.h"

fuzzy::fuzzy(){}
 

fuzzy::~fuzzy() {}


//取两个数之间数值较小的数
float fuzzy::min(float a, float b)
{
	if (a < b)
		return a;
	else
		return b;
}

//取两个数之间数值较大的数
float fuzzy::max(float a, float b)
{
	if (a > b)
		return a;
	else
		return b;
}
//四舍五入取整
int fuzzy::round_float(float number)
{
	return (number > 0.0) ? (number + 0.5) : (number - 0.5);
}


//输入量的模糊化处理
//将输入的角度偏差和角速度偏差转化为对应的模糊集合
void fuzzy::Fuzzy_Input(float target_e, float current_value_e,float target_ec,float current_value_ec)
{
	error = current_value_e - target_e;
	dderror = current_value_ec - target_ec;
	e = error/10;
	ec = dderror/20;
	//找到e对应的模糊集合的状态
	if (e <= -5)
		for (int i = 0; i < 14; i++)
			e1[i] = E[ENL][i];
	if (e>-5&&e<=-3 )
		for (int i = 0; i < 14; i++)
			e1[i] = E[ENM][i];
	if (e > -3 && e <=-1)
		for (int i = 0; i < 14; i++)
			e1[i] = E[ENS][i];
	if (e > -1 && e <= 0)
		for (int i = 0; i < 14; i++)
			e1[i] = E[ENO][i];
	if (e > 0 && e <= 1)
		for (int i = 0; i < 14; i++)
			e1[i] = E[EPO][i];
	if (e > 1 && e <= 3)
		for (int i = 0; i < 14; i++)
			e1[i] = E[EPS][i];
	if (e > 3 && e <= 5)
		for (int i = 0; i < 14; i++)
			e1[i] = E[EPM][i];
	if (e > 5)
		for (int i = 0; i < 14; i++)
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

//根据输入模糊关系矩阵计算输出模糊关系矩阵
void fuzzy::U_Calculate()
{
	//计算E和EC的模糊关系矩阵
	int q = 0;
	for (int j = 0; j < 14; j++)
	{
		for (int i = 0; i < 13; i++)
		{

			FC_Matrix_1[j][i] = min(e1[j], ec1[i]);
			FC_Matrix_Use_1[q] = FC_Matrix_1[j][i];
			q++;
		}
	}

	float zuida;
	for (int w = 0; w < 13; w++)
	{
		for (int f = 0; f < 14 * 13; f++)
			use[f] = min(FC_Matrix_Use_1[f], FC_Matrix_Total[f][w]);
		zuida = use[0];
		for (int f = 0; f < 14 * 13; f++)
		{
			if (use[f] >= zuida)
				zuida = use[f];
		}
		U_Final[w] = zuida;
	}
	//测试程序
	/*
	for (int f = 0; f < 14 * 13; f++)
		use[f] = min(FC_Matrix_Use_1[f], FC_Matrix_Total[f][1]);
	zuida = use[0];
	for (int f = 0; f < 14 * 13; f++)
	{
		if (use[f] >= zuida)
			zuida = use[f];
	}
	U_Final[1] = zuida;
	*/
}
void fuzzy::U_Defuzzy()
{
	int a = 0;
	for (int i = -6; i <= 6; i++)
	{
		u = u + i * U_Final[a];
		a++;
	}
	for (int i = 0; i < 13; i++)
	{
		u_total = u_total + U_Final[i];
	}
	u = u / u_total;
}
//模糊矩阵的计算
//a E模糊集合（行数）
//b EC模糊集合（行数）
//c U模糊集合（行数）
//对每条模糊规则Ri = （Ai x Bi） x Ci
//首先计算E 和 EC的模糊关系矩阵,并将E和EC的模糊关系矩阵转化为可参与下一次计算的形式
//E可近似看作14x1的列向量
//EC可看作1x13的行向量
void fuzzy::Fuzzy_Control_Matrix_cal(int a,int b,int c)
{
	int k = 0;
	for (int j=0; j<14; j++)
	{
		for (int i = 0; i < 13; i++)
		{

			FC_Matrix[j][i] = min(E[a][j],EC[b][i]);
			FC_Matrix_Use[k] = FC_Matrix[j][i];
			k++;
		}
	}
	for (int i = 0; i < 14 * 13; i++)
	{
		for (int j = 0; j < 13; j++)
		{
			FC_Matrix_Final[i][j] = min(FC_Matrix_Use[i],U[c][j]);
		}
	}
}

void fuzzy::Fuzzy_Control_Matrix_Total()
{
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			if(rules[i][j]!=666)
				Fuzzy_Control_Matrix_cal(i, j, rules[i][j]);
			if (rules[i][j] == 666)
				continue;
			for(int a=0; a<14*13; a++)
				for (int b = 0; b < 13; b++)
				{
					FC_Matrix_Total[a][b] = max(FC_Matrix_Total[a][b],FC_Matrix_Final[a][b]);
				}


			/*
			for (int a = 0; a < 14 * 13; a++)
			{
				for (int b = 0; b < 13; b++)
				{
					FC_Matrix_Final_Last[a][b] = FC_Matrix_Final[a][b];
				}
			}
			if (rules[i][j] == 666)
				continue;
			if(rules[i][j]!=666)
				Fuzzy_Control_Matrix_cal(i,j,rules[i][j]);
			for (int c = 0; c < 14 * 13; c++)
			{
				for (int d = 0; d < 13; d++)
				{
					FC_Matrix_Total[c][d] = max(FC_Matrix_Final_Last[c][d], FC_Matrix_Final[c][d]);
				}
			}
			*/
		}
	}
}
 
//模糊规则
void fuzzy::Set_Rules()
{

}