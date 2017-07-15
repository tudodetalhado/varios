#pragma once
#include "stdafx.h"
#include <windows.h>
#include <Mmsystem.h>//��Ҫ Winmm.lib���֧�� ----timeGetTime()
#include <string>
#include <Kinect.h>
using namespace std;
class CTools
{
public:
	static string DigitToString(int val)
	{
		char char_str[50];
		sprintf_s(char_str, "%d", val);
		return string(char_str);
	}

	//��64bit����ת��Ϊ�ַ���
	static string DigitToString(UINT64 val)
	{
		char char_str[50];
		sprintf_s(char_str, "%ld", val);
		return string(char_str);
	}

	//��32bit������ת��Ϊ�ַ���
	static string DigitToString(float val)
	{
		char char_str[50];
		sprintf_s(char_str, "%.2f", val);
		return string(char_str);
	}

	//��leap����ת��Ϊ�ַ���
	static string DigitToString(Vector4 vec)
	{
		char char_str[100];
		sprintf_s(char_str, "%.2f\t%.2f\t%.2f\t%.2f", vec.x, vec.y,vec.z,vec.w);
		return string(char_str);
	}

	//����־ת��Ϊ�ַ���
	static string DigitToString(bool flag)
	{
		return (flag == true) ? "true" : "false";
	}

	//��ϵͳʱ��ת��Ϊ�ַ���
	static string GetTimeString()
	{
		SYSTEMTIME sys;
		GetLocalTime(&sys);
		char char_str[100];
		sprintf_s(char_str, "%02d:%02d:%02d:%03d", sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);
		return string(char_str);
	}
};

