//目前只支持7个模糊等级，论域为[-6，6]
//                       模糊等级为：PB PM PS ZO NS NM NP
// 
//****************** 注意：模糊等级和论域目前不支持自定义***********************
// 
//双输入：角度偏差E和角速度偏差EC
//单输出：ΔKp 或ΔKi 或ΔKd
#pragma once
#ifndef FUZZYCONTROL_H
#define FUZZYCONTROL_H
//为论域与模糊等级之间建立对应关系

#define ENL 0
#define ENM 1
#define ENS 2
#define ENO 3
#define EPO 4
#define EPS 5
#define EPM 6
#define EPL 7

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

 
class fuzzy
{
 public:
	 fuzzy();
	 ~fuzzy();
	 //fuzzy matrix模糊关系矩阵计算
	 void fm_calculate();
	 void Set_Rules();
 
private:
	//角度误差论域[-6,6]共计14个数(分+0和-0，所以14个)

	float E[8][14] = 
	{ 
//论域-6    -5    -4    -3     -2    -1    -0    +0     1     2     3     4     5    6
	{ 1.0,  0.8,  0.4,  0.1,    0,    0,    0,    0,    0,    0,    0,    0,    0,   0},//NL
 
	{ 0.2,  0.7,  1.0,  0.7,  0.2,    0,    0,    0,    0,    0,    0,    0,    0,   0},//NM
 
	{   0,    0,  0.1,  0.5,    1,  0.8,  0.3,    0,    0,    0,    0,    0,    0,   0},//NS
 
	{   0,    0,    0,    0,  0.1,  0.6,  1.0,    0,    0,    0,    0,    0,    0,   0},//NO
 
	{   0,    0,    0,    0,    0,    0,    0,  1.0,  0.6,  0.1,    0,    0,    0,   0},//PO
 
	{   0,    0,    0,    0,    0,    0,    0,  0.3,  0.8,  1.0,  0.5,  0.1,    0,   0},//PS
 
	{   0,    0,    0,    0,    0,    0,    0,    0,    0,  0.2,  0.7,  1.0,  0.7, 0.2},//PM
 
	{   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  0.1,  0.4,  0.8, 1.0}	//PL
	};


	//角速度误差论域，同
	float EC[7][13] = 
	{
//论域-6    -5    -4    -3     -2    -1     0     1     2     3     4     5     6
	{ 1.0,  0.8,  0.4,  0.1,    0,    0,    0,    0,    0,    0,    0,    0,    0},//NL

	{ 0.2,  0.7,  1.0,  0.7,  0.2,    0,    0,    0,    0,    0,    0,    0,    0},//NM

	{   0,    0,  0.2,  0.6,  1.0,  0.9,    0,    0,    0,    0,    0,    0,    0},//NS

	{   0,    0,    0,    0,    0,  0.5,  1.0,  0.5,    0,    0,    0,    0,    0},//ZO

	{   0,    0,    0,    0,    0,    0,    0,  0.9,  1.0,  0.7,  0.2,    0,    0},//PO

	{   0,    0,    0,    0,    0,    0,    0,    0,  0.2,  0.7,  1.0,  0.7,  0.2},//PS

	{   0,    0,    0,    0,    0,    0,    0,    0,    0,  0.1,  0.4,  0.8,  1.0} //PM
	};


	//输出量论域，同
	float U[7][13] =  
	{
//论域-6    -5    -4    -3     -2    -1     0     1     2     3     4     5     6
	{ 1.0,  0.8,  0.4,  0.1,    0,    0,    0,    0,    0,    0,    0,    0,    0},//NL

	{ 0.2,  0.7,  1.0,  0.7,  0.2,    0,    0,    0,    0,    0,    0,    0,    0},//NM

	{   0,    0,  0.2,  0.6,  1.0,  0.9,    0,    0,    0,    0,    0,    0,    0},//NS

	{   0,    0,    0,    0,    0,  0.5,  1.0,  0.5,    0,    0,    0,    0,    0},//ZO

	{   0,    0,    0,    0,    0,    0,    0,  0.9,  1.0,  0.7,  0.2,    0,    0},//PO

	{   0,    0,    0,    0,    0,    0,    0,    0,  0.2,  0.7,  1.0,  0.7,  0.2},//PS

	{   0,    0,    0,    0,    0,    0,    0,    0,    0,  0.1,  0.4,  0.8,  1.0} //PM
	};


	//模糊规则，储存在矩阵中，若该位置数值为666，则表示不可能出现的情况（死区）
	float rules[7][8] =
	{

	{ 666,  666,  UPL,  UPL,  UPL,  UPL,  UNM,  UNL},//NL

	{ UPM,  UPS,  UPS,  UPM,  UPM,  UPM,  UNM,  UNL},//NM

	{ UPL,  UPM,  UPS,  UPS,  UPS,  UPS,  UNM,  UNL},//NS

	{ UPL,  UPM,  UPS,  UZO,  UZO,  UNS,  UNM,  UNL},//ZO

	{ UPL,  UPM,  UNS,  UNS,  UNS,  UNS,  UNM,  UNL},//PO

	{ UPL,  UPM,  UNM,  UNM,  UNS,  UNS,  UNS,  UNM},//PS

	{ UPL,  UPM,  UNL,  UNL,  UNL,  UNL,  666,  666} //PM
	};
};


#endif // !FUZZYCONTROL_H

