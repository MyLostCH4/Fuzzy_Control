#include"fuzzycontrol.h"

fuzzy::fuzzy(){}
 

fuzzy::~fuzzy() {}



float fuzzy::min(float a, float b)
{
	if (a < b)
		return a;
	else
		return b;
}

float fuzzy::max(float a, float b)
{
	if (a > b)
		return a;
	else
		return b;
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