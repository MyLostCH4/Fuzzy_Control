#include<iostream>
using namespace std;
#include"fuzzycontrol.h"
void main()
{
	fuzzy fuzzy;
	fuzzy.Fuzzy_Control_Matrix_cal(1,2,3);

	for (int i = 0; i < 14 * 13; i++)
	{
		for (int j = 0; j < 13; j++)
		{
			if (j < 12)
					cout << fuzzy.FC_Matrix_Final[i][j] << "  ";
			if (j == 12)
				cout << fuzzy.FC_Matrix_Final[i][j] << endl;
		}
	}
}