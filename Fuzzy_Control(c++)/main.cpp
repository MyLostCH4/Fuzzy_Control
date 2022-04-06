#include<iostream>
#include"fuzzycontrol.h"
using namespace std;

fuzzy a;

void main()
{

	a.Fuzzy_Control_Matrix_Total();
	
	a.Fuzzy_Input(0, 1, 0, 0);

	a.U_Calculate();

	a.U_Defuzzy();

	

	cout << "E对应的模糊集合：" << endl;
	for (int i = 0; i < 14; i++)
		cout << a.e1[i] << " ";
	cout << endl;
	cout << "EC对应的模糊集合：" << endl;
	for (int i = 0; i < 13; i++)
		cout << a.ec1[i] << " ";
	cout << endl;
	cout << "E和EC所确定的模糊关系矩阵:" << endl;
	for (int i = 0; i < 14; i++)
		for (int j = 0; j < 13; j++)
		{
			cout << a.FC_Matrix_1[i][j] << "  ";
			if (j == 12)
				cout << endl;
		}

	cout << "输出量的模糊集合：" << endl;
	for (int i = 0; i < 13; i++)
		cout << a.U_Final[i] << " ";
	cout << endl;
	
	cout << "将输出模糊矩阵解模糊化后的精确输出量：" << endl;
	cout << a.u << endl;
	/*
	cout << "总的模糊关系矩阵:" << endl;
	for (int i = 0; i < 14 * 13; i++)
	{
		for (int j = 0; j < 13; j++)
		{
			if (j < 12)
				cout << a.FC_Matrix_Total[i][j] << "    ";
			if (j == 12)
				cout << a.FC_Matrix_Total[i][j] << endl;
		}
	}
	*/
	 
}

