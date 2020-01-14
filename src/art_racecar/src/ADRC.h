/*
Copyright (c) 2019,ZunYi Zhou
All rights reserved.(CQU)

This file is used for Active Disturbance Rejection Control

It's distributed in the hope that it will be useful,but 
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with ADRC.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef	_ADRC_H
#define _ADRC_H

class ADRC
{
	public:
		ADRC();
		double Fhan(double _e, double _v2, double _r, double _h);
		double Fal(double _e, double _alpha, double _delta);
		int Sign(double _x);
		int Fsg(double _x, double _d);
		double Calculate(double _y, double _v0); //返回补偿后控制量  _y为系统输出 
		void TD(double _v0);
		void ESO(double _y);
		double getV1(){return v1;}//获取v1
		double getZ1(){return z1;}
	private:
		/*--------安排过渡过程--------*/ 
		double v0; 	//设定值
		double v1;	//跟踪的状态变量
		double v2;	//跟踪的状态变量微分
		double fh;	//fh=fhan(e,v2,r,h0) fhan最速控制综合函数 
		double e;	//过渡过程误差	 e=v1-v0
		double r0;	//1/T,T为时间常数
		double h;	//积分步长（采样时间）
		double n0;	//h0=n0*h 
		double h0; 	//h0为滤波因子 防止超调，抑制噪声放大
		
		/*--------跟踪估计系统状态和扰动--------*/
		double z1;	//跟踪估计的状态变量 
		double z2;	//跟踪估计的状态变量微分 
		double z3;	//扰动补偿 
		double err;	//系统误差  err=z1-y
		double y;	//系统输出
		double b0;	//补偿系数 
 		double beta01;
		double beta02;
		double beta03;	//可调系数 
		double fe;	//fe=fal(err,0.5,delta)
		double fe1; //fe1=fal(err,0.25,delta)
		double delta;	//线性段的区间长度 
		
		/*--------误差反馈律--------*/
		double u0;	//补偿前控制量（非线性组合后的控制量）
		double e1;	//状态误差项 e1=v1-z1 
		double e2; 	//微分误差项 e2=v2-z2 
		int combine_mode; //选择的组合方式 
			/*第一种组合方式*/ //u0=beta1*e1+beta2*e2 
		double beta1;	//比例系数 
		double beta2; 	//微分系数 
			/*第二种组合方式*/ //u0=beta1*fal(e1,alpha1,delta)+beta2*fal(e2,alpha2,delta) 0 < alpha1 < 1 < alpha2
		double alpha1;
		double alpha2;
			/*第三种组合方式*/ //u0=-fhan(e1,e2,r,h1)
		double r;
		double n1;	//h1=n1*h 
		double h1;	//h1为滤波因子 防止超调，抑制噪声放大
			/*第四种组合方式*/ //u0=-fhan(e1,c*e2,r,h1)
		double c;
		
		/*--------扰动补偿过程--------*/
		double u;	//补偿后控制量	u=u0-z3/b0或u=(u0-z3)/b0		
};

#endif

