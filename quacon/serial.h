#pragma once
#include <Windows.h>

void connect(TCHAR * CurrentPort);
void sendData(const void * Data, size_t Count);
