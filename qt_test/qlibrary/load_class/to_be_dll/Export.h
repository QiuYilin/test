#pragma once

#include "VenderAddExport.h"

class Interface;

// extern "C" _declspec(dllexport) Interface* GetInstance();//使用 extern "C" 来确保函数名不会被改变。这样其他编程语言就可以按照C语言的规则来调用该函数。
// extern "C" _declspec(dllexport) void ReleaseInstance();

extern "C" VENDERADD_EXPORT Interface* GetInstance();
extern "C" VENDERADD_EXPORT void ReleaseInstance();