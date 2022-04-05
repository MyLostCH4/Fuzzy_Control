#include<iostream>
#include"fuzzycontrol.h"
using namespace std;

fuzzy a;

void main()
{

	a.Fuzzy_Control_Matrix_Total();
	
	a.Fuzzy_Input(60,30,150,10);

	a.U_Calculate();

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
	 
}

