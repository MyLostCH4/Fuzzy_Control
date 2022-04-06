#pragma once
#ifndef FUZZYPID_H
#define FUZZYPID_H
//为论域与模糊等级之间建立对应关系

#define ENL 0
#define ENM 1
#define ENS 2
#define EZO 3
#define EPS 4
#define EPM 5
#define EPL 6

#define ECNL 0
#define ECNM 1
#define ECNS 2
#define ECZO 3
#define ECPS 4
#define ECPM 5
#define ECPL 6

#define UNL 0
#define UNM 1
#define UNS 2
#define UZO 3
#define UPS 4
#define UPM 5
#define UPL 6
class fuzzypid
{
public:

	fuzzypid();
	~fuzzypid();


	//返回a和b之间数值小的那个
	float min(float a, float b);


	//返回a和b之间数值大的那个
	float max(float a, float b);


	//四舍五入将小数化为整数
	int round_float(float number);


	//fuzzy matrix模糊关系矩阵计算
	void Fuzzy_Control_Matrix_cal(int a, int b, int c);


	//计算所有模糊矩阵的并集（把所有模糊矩阵叠到一起）
	void Fuzzy_Control_Matrix_Total();


	//输入量的模糊化处理
	void Fuzzy_Input(float target_e, float current_value_e, float target_ec, float current_value_ec);


	//计算输出的模糊矩阵
	void U_Calculate();


	//对输出的模糊矩阵解模糊化
	void U_Defuzzy();


	//计算添加了ΔKp 、 ΔKi 、 ΔKd之后的Kp、Ki、Kd
	void Cal_Kp_Ki_Kd(float Kp,float Ki,float Kd);

	//输入Kp、Ki、Kd的变化区间
	void Input_Kp_Interval(float a, float b) { Kp_max =  b - a; }

	void Input_Ki_Interval(float a, float b) { Ki_max = b - a; }
	
	void Input_Kd_Interval(float a, float b) { Kd_max = b - a; }



	//E和EC组成的模糊关系矩阵(计算总模糊矩阵时使用)
	float FC_Matrix[13][13];
	float FC_Matrix_Use[13 * 13];
	float FC_Matrix_Final[13*13][13];
	

	//输入的精确角度误差和角速度误差
	float error;
	float dderror;


	//将误差映射到规定论域
	float e;
	float ec;


	//精确误差模糊化处理后得到的模糊集合
	float e1[13];
	float ec1[13];
	float u1[13];

	//e1和ec1组成的模糊关系矩阵，计算输出矩阵时使用
	float fc_matrix[13][13];
	float fc_matrix_use[13 * 13];


	//Kp 、Ki、Kd的输出模糊矩阵
	float U_Final[3][13];


	//输出Kp、Ki、Kd模糊矩阵的精确解
	float u[3];


	//根据输出量的模糊矩阵计算输出量的最大值（绝对值）
	float u_max = (1 * 6 + 0.8 * 5 + 0.4 * 4 + 0.1 * 3) / (1 + 0.8 + 0.4 + 0.1);

	//Kp Ki Kd所对应的总模糊关系矩阵
	float Final[3][13 * 13][13];


	//Kp、Ki、Kd的变化量
	float Kp_;
	float Ki_;
	float Kd_;

	//Kp、Ki、Kd的变化范围
	float Kp_max;
	float Ki_max;
	float Kd_max;

	//处理后Kp、Ki、Kd的值
	float kp;
	float ki;
	float kd;



	//测试变量，用于验证算法
	float y[3] = { 1,2,3 };
	float k[3] = { 4,3,1 };
	float p[3] = { 4,6,7 };
	float v[3][3];
	float vv[9];
	float vvv[3 * 3][3];

	//测试函数，用于验证算法
	void Fuzzy_Control_Matrix_cal_1();
 
private:
	//角度误差论域
	float E[7][13] =
	{
		//-6    -5    -4    -3     -2    -1     0     1     2     3     4     5     6
		{ 1.0,  0.8,  0.4,  0.1,    0,    0,    0,    0,    0,    0,    0,    0,    0},//ENL

		{ 0.2,  0.7,  1.0,  0.7,  0.2,    0,    0,    0,    0,    0,    0,    0,    0},//ENM

		{   0,    0,  0.2,  0.6,  1.0,  0.9,    0,    0,    0,    0,    0,    0,    0},//ENS

		{   0,    0,    0,    0,    0,  0.5,  1.0,  0.5,    0,    0,    0,    0,    0},//EZO

		{   0,    0,    0,    0,    0,    0,    0,  0.9,  1.0,  0.7,  0.2,    0,    0},//EPS

		{   0,    0,    0,    0,    0,    0,    0,    0,  0.2,  0.7,  1.0,  0.7,  0.2},//EPM

		{   0,    0,    0,    0,    0,    0,    0,    0,    0,  0.1,  0.4,  0.8,  1.0} //EPL
	};
	//角速度误差论域
	float EC[7][13] =
	{
		//-6    -5    -4    -3     -2    -1     0     1     2     3     4     5     6
		{ 1.0,  0.8,  0.4,  0.1,    0,    0,    0,    0,    0,    0,    0,    0,    0},//ECNL

		{ 0.2,  0.7,  1.0,  0.7,  0.2,    0,    0,    0,    0,    0,    0,    0,    0},//ECNM

		{   0,    0,  0.2,  0.6,  1.0,  0.9,    0,    0,    0,    0,    0,    0,    0},//ECNS

		{   0,    0,    0,    0,    0,  0.5,  1.0,  0.5,    0,    0,    0,    0,    0},//ECZO

		{   0,    0,    0,    0,    0,    0,    0,  0.9,  1.0,  0.7,  0.2,    0,    0},//ECPS

		{   0,    0,    0,    0,    0,    0,    0,    0,  0.2,  0.7,  1.0,  0.7,  0.2},//ECPM

		{   0,    0,    0,    0,    0,    0,    0,    0,    0,  0.1,  0.4,  0.8,  1.0} //ECPL
	};
	//输出量论域
	float U[7][13] =
	{
		//-6    -5    -4    -3     -2    -1     0     1     2     3     4     5     6
		{ 1.0,  0.8,  0.4,  0.1,    0,    0,    0,    0,    0,    0,    0,    0,    0},//UNL

		{ 0.2,  0.7,  1.0,  0.7,  0.2,    0,    0,    0,    0,    0,    0,    0,    0},//UNM

		{   0,    0,  0.2,  0.6,  1.0,  0.9,    0,    0,    0,    0,    0,    0,    0},//UNS

		{   0,    0,    0,    0,    0,  0.5,  1.0,  0.5,    0,    0,    0,    0,    0},//UZO

		{   0,    0,    0,    0,    0,    0,    0,  0.9,  1.0,  0.7,  0.2,    0,    0},//UPO

		{   0,    0,    0,    0,    0,    0,    0,    0,  0.2,  0.7,  1.0,  0.7,  0.2},//UPS

		{   0,    0,    0,    0,    0,    0,    0,    0,    0,  0.1,  0.4,  0.8,  1.0} //UPM
	};

	//模糊规则，储存在矩阵中，若该位置数值为666，则表示不可能出现的情况（死区）
	float rules[3][7][7] =
	{
		//Kp对应的模糊规则
		{
			//ENL   ENM   ENS   EZO   EPS   EPM   EPL
			{ 666,  666,  UPL,  UPL,  UPL,  UNM,  UNL},//ECNL

			{ UPM,  UPS,  UPS,  UPM,  UPM,  UNM,  UNL},//ECNM

			{ UPL,  UPM,  UPS,  UPS,  UPS,  UNM,  UNL},//ECNS

			{ UPL,  UPM,  UPS,  UZO,  UNS,  UNM,  UNL},//ECZO

			{ UPL,  UPM,  UNS,  UNS,  UNS,  UNM,  UNL},//ECPS

			{ UPL,  UPM,  UNM,  UNS,  UNS,  UNS,  UNM},//ECPM

			{ UPL,  UPM,  UNL,  UNL,  UNL,  666,  666} //ECPL
		},
		//Ki对应的模糊规则
		{
			//ENL   ENM   ENS   EZO   EPS   EPM   EPL
			{ 666,  666,  UPL,  UPL,  UPL,  UNM,  UNL},//ECNL

			{ UPM,  UPS,  UPS,  UPM,  UPM,  UNM,  UNL},//ECNM

			{ UPL,  UPM,  UPS,  UPS,  UPS,  UNM,  UNL},//ECNS

			{ UPL,  UPM,  UPS,  UZO,  UNS,  UNM,  UNL},//ECZO

			{ UPL,  UPM,  UNS,  UNS,  UNS,  UNM,  UNL},//ECPS

			{ UPL,  UPM,  UNM,  UNS,  UNS,  UNS,  UNM},//ECPM

			{ UPL,  UPM,  UNL,  UNL,  UNL,  666,  666} //ECPL
		},
		//Kd对应的模糊规则
		{
			//ENL   ENM   ENS   EZO   EPS   EPM   EPL
			{ 666,  666,  UPL,  UPL,  UPL,  UNM,  UNL},//ECNL

			{ UPM,  UPS,  UPS,  UPM,  UPM,  UNM,  UNL},//ECNM

			{ UPL,  UPM,  UPS,  UPS,  UPS,  UNM,  UNL},//ECNS

			{ UPL,  UPM,  UPS,  UZO,  UNS,  UNM,  UNL},//ECZO

			{ UPL,  UPM,  UNS,  UNS,  UNS,  UNM,  UNL},//ECPS

			{ UPL,  UPM,  UNM,  UNS,  UNS,  UNS,  UNM},//ECPM

			{ UPL,  UPM,  UNL,  UNL,  UNL,  666,  666} //ECPL
		}

	};
	//测试矩阵
	float ru[7][7] =
	{
		//ENL   ENM   ENS   EZO   EPS   EPM   EPL
		{ 666,  666,  UPL,  UPL,  UPL,  UNM,  UNL},//ECNL

		{ UPM,  UPS,  UPS,  UPM,  UPM,  UNM,  UNL},//ECNM

		{ UPL,  UPM,  UPS,  UPS,  UPS,  UNM,  UNL},//ECNS

		{ UPL,  UPM,  UPS,  UZO,  UNS,  UNM,  UNL},//ECZO

		{ UPL,  UPM,  UNS,  UNS,  UNS,  UNM,  UNL},//ECPS

		{ UPL,  UPM,  UNM,  UNS,  UNS,  UNS,  UNM},//ECPM

		{ UPL,  UPM,  UNL,  UNL,  UNL,  666,  666} //ECPL
	};
};



#endif // !FUZZYPID_H
