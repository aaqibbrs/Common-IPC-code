#include <iostream>
#include <sstream>

using namespace std;

string converter(string str)
{
	stringstream hr_time, mi_time, hr_new_time, mi_new_time;
	string m_time = "00", h_time = "00";
	
	int minu_time, hrs_time;

	m_time[0] = str.at(3); m_time[1]=str.at(4);
	h_time[0] = str.at(0); h_time[1]= str.at(1);

	hr_time<<h_time;
	hr_time>>hrs_time;
	mi_time<<m_time;
	mi_time>>minu_time;

	if(minu_time<30)
	{
	//j
		minu_time = minu_time + 30;
		hrs_time = hrs_time + 5;
	}
	else
	{
	//u
		minu_time = minu_time - 30;
		hrs_time = hrs_time + 6;
	}
	
	if(minu_time < 10)
	{
	//g
		mi_new_time<<"0"<<minu_time;
	}
	else
	{
	//a
		mi_new_time<<minu_time;
	}
	if(hrs_time<10)
	{
	//a
		hr_new_time<<"0"<<hrs_time;
	}
	else
	{
	//d
		hr_new_time<<hrs_time;
	}
	
	m_time.clear(); h_time.clear();

	mi_new_time>>m_time;
	hr_new_time>>h_time;

	str[0] = h_time[0];str[1] = h_time[1]; str[3] = m_time[0]; str[4] = m_time[1];

	return str;
}