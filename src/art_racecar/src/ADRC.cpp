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

#include <iostream>
#include "ADRC.h" 
#include "ros/ros.h"
#include <cmath>

using namespace std;

ADRC::ADRC()
{
	/*初始化变量*/ 
	ros::NodeHandle pn("~");
	pn.param("v0", v0, 2.0);
	pn.param("h", h, 0.05);
	pn.param("r0", r0, 100.0);
	pn.param("r", r, 50.0);
	pn.param("n0", n0, 1.0);
	pn.param("n1", n1, 1.0);
	pn.param("b0", b0, 1.0);
	pn.param("beta01", beta01, 1.0);
	pn.param("beta02", beta02, 1.0);
	pn.param("beta03", beta03, 1.0);
	pn.param("delta", delta, 0.01);
	pn.param("combine_mode", combine_mode, 1);
	pn.param("beta1", beta1, 1.0);
	pn.param("beta2", beta2, 1.0);
	pn.param("alpha1", alpha1, 0.25);
	pn.param("alpha2", alpha2, 1.5);
	pn.param("c", c, 1.0);	
	v1 = v2 = 0;
	z1 = z2 = z3 = 0.0;
	u = 0;
	/*打印信息*/ 
	cout << "h = " << h << endl;
	cout << "r = " << r << endl; 
	cout << "r0 = " << r0 << endl; 
	cout << "n0 = " << n0 << endl;
	cout << "n1 = " << n1 << endl;
	cout << "b0 = " << b0 << endl;
	cout << "beta01 = " << beta01 << endl;
	cout << "beta02 = " << beta02 << endl;
	cout << "beta03 = " << beta03 << endl;
	cout << "delta = " << delta << endl;
	cout << "combine_mode = " << combine_mode << endl;
	cout << "beta1 = " << beta1 << endl;
	cout << "beta2 = " << beta2 << endl;
	cout << "alpha1 = " << alpha1 << endl;
	cout << "alpha2 = " << alpha2 << endl;
	cout << "c = " << c << endl;
}

int ADRC::Sign(double _x) //符号函数 
{
	int out = 0;
	if(_x > 1E-6)
		out = 1;
	else if(_x < -1E-6)
		out = -1;
	else 
		out = 0;
	return out;		
}

int ADRC::Fsg(double _x, double _d)
{
	return (Sign(_x + _d) - Sign(_x - _d)) / 2;
}

double ADRC::Fhan(double _x1, double _x2, double _r, double _h) // 最速控制综合函数 
{
	double d = 0, a0 = 0, _y = 0, a1 = 0, a2 = 0, a = 0, _fh = 0;
	d = _r * powf(_h, 2);
	a0 = _h * _x2;
	_y = _x1 + a0;
	a1 = sqrt(d * (d + 8 * fabs(_y)));
	a2 = a0 + Sign(_y) * (a1 - d) / 2;
	a = (a0 + _y) * Fsg(_y, d) + a2 * (1 - Fsg(_y, d));
	_fh = -_r * (a / d) * Fsg(a, d) - _r * Sign(a) * (1 - Fsg(a, d));
	return _fh;
}

double ADRC::Fal(double _e, double _alpha, double _delta) //连续的幂次函数 
{
	double _fe = 0;
	int s = Fsg(_e, _delta);
	_fe = _e * s / powf(_delta, 1 - _alpha) + powf(fabs(_e), _alpha) * Sign(_e) * (1 - s);
	return _fe;
}

void ADRC::TD(double _v0)
{
	e = v1- _v0;
	h0 = n0 * h; //h0为滤波因子 防止超调，抑制噪声放大
	fh = Fhan(e, v2, r0, h0);
	v1 += h * v2;
	v2 += h * fh;
}

void ADRC::ESO(double _y)
{
	err = z1 - _y;
	fe = Fal(err, 0.5, h);
	fe1 = Fal(err, 0.25, h);
	z1 =z1+ h * (z2 - beta01 * err);
	z2 += h * (z3 - beta02 * fe + b0 * u);
	z3 += h * (-beta03 * fe1);
}
double ADRC::Calculate(double _y,double _v0)
{
	/*第一步---跟踪微分器初始化及安排过渡过程*/ 
	TD(_v0);
	
	/*第二步---估计系统状态和扰动*/
		/*扩张状态观测器*/ 
	ESO(_y);
	
	/*第三步---状态误差反馈律*/
	e1 = v1 - z1;//状态偏差项
	e2 = v2 - z2;//状态微分项
	e2 = e2 > 1000 ? 1000:e2;
	e2 = e2 < -1000 ? -1000:e2;
	h1 = n1 * h;
	if(combine_mode == 1)
		u0 = beta1 * e1 + beta2 * e2; 
	else if(combine_mode == 2)
		u0 = beta1 * Fal(e1, alpha1, delta) + beta2 * Fal(e2, alpha2, delta);
	else if(combine_mode == 3)
		u0 = -Fhan(e1, e2, r, h1);
	else
		u0 = -Fhan(e1, c * e2, r, h1);
		
	/*第四步---扰动补偿*/
	u = u0 - z3 / b0; //或者 u = (u0 - z3) / b0;
	// cout<<"u0 = "<<u0<<" e1 = "<<e1<<" e2 = "<<e2<<endl;
	u = u > 200 ? 200:u;
	u = u < -200 ? -200:u;
	return u;
}
