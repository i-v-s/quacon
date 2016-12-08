#include "stdafx.h"
#include <Windows.h>
#include <iostream>

HANDLE hPort = 0;

HANDLE UPLHandle = 0, hPortScan = 0, hFileSend = 0, hReceive = 0;

CRITICAL_SECTION UPLSection, FSSection;

OVERLAPPED Overlap = {};
COMSTAT ComState = {};

DWORD WINAPI ReceiveThread(void * Param)
{
	char Buffer[1025];
	COMSTAT State;
	if (!hPort) return 0;
	DWORD n, t = 0;
	for (;;)
	{
		DWORD e;
		ClearCommError(hPort, &e, &State);
		if (!e && State.cbInQue)
		{
			if (!ReadFile(hPort, Buffer, State.cbInQue, &n, &Overlap))
			{
				int e = GetLastError();
				if (e != 997)
				{
					char buf[64];
					sprintf_s(buf, sizeof(buf), "Ошибка ReadFile(): LastError = %d, принято %d из %d.", e, n, 1024);
					std::cout << buf;
					return 0;
				}
			}
			//RMemo->AppendText(Buffer, State.cbInQue);
		}
		Sleep(100);
	}
	return 0;
}

void connect(TCHAR * CurrentPort)
{
	using namespace std;

	InitializeCriticalSection(&FSSection);

	cout << "Try to open port...";

	HANDLE handle = CreateFile(CurrentPort, GENERIC_READ | GENERIC_WRITE, NULL, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
	//free(CurrentPort);
	if (handle == INVALID_HANDLE_VALUE)
	{
		cout << "Port opening error.";
		//CBUpdate->State = BST_UNCHECKED;
		//OnUpdateBox(CBUpdate);
		return;
	}
	DCB	dcb;
	cout << "Retrieving port state...";
	if (!GetCommState(handle, &dcb))
	{
		CloseHandle(handle);
		cout << "Port state retrieving error.";
		//CBUpdate->State = BST_UNCHECKED;
		return;
	}
	dcb.BaudRate = 115200;// 56000;
	dcb.ByteSize = 8;
	dcb.Parity = 0;
	dcb.StopBits = 0;
	cout << "Setting port state...";
	if (!SetCommState(handle, &dcb))
	{
		CloseHandle(handle);
		cout << "State saving error.";
		//CBUpdate->State = BST_UNCHECKED;
		return;
	}
	COMMTIMEOUTS tm;
	tm.ReadIntervalTimeout = 10;
	tm.ReadTotalTimeoutMultiplier = 1;
	tm.ReadTotalTimeoutConstant = 100;
	tm.WriteTotalTimeoutMultiplier = 0;
	tm.WriteTotalTimeoutConstant = 0;
	cout << "Устанавливаем таймауты...";
	SetCommTimeouts(handle, &tm);
	SetupComm(handle, 1200, 1200);

	PurgeComm(handle, PURGE_RXCLEAR);
	PurgeComm(handle, PURGE_TXCLEAR);

	cout << "Connection successfull.";
	hPort = handle;

	//DWORD Id;
	//hReceive = CreateThread(0, 0, &ReceiveThread, 0, 0, &Id);
}

void sendData(const void * Data, size_t Count)
{
	using namespace std;
	if (!hPort) return;
	DWORD e;
	//cout << "Сбрасываем ошибки порта...";
	ClearCommError(hPort, &e, &ComState);
	if (e)
	{
		cout << "Ошибка ClearCommError().";
		return;
	}
	//cout << "Производим отправку...";
	DWORD W;

	SetCommMask(hPort, EV_TXEMPTY);
	size_t MaxSize = 500;
	size_t c = 0;
	for (const char * d = (const char *)Data; c < Count; d += MaxSize)
	{
		size_t s = MaxSize;
		if (s > Count - c) s = Count - c;
		c += s;
		EnterCriticalSection(&FSSection);
		if (!WriteFile(hPort, d, (DWORD)s, &W, &Overlap))
		{
			int e = GetLastError();
			if (e != 997)
			{
				printf("Ошибка WriteFile(): %d, отправлено %d из %d.", e, W, (int)Count);
				LeaveCriticalSection(&FSSection);
				return;
			}
		}
		LeaveCriticalSection(&FSSection);
		if (!GetOverlappedResult(hPort, &Overlap, &W, true))
		{
			DWORD e = GetLastError();
			if (e == ERROR_OPERATION_ABORTED)
			{
				printf("Отправка прервана %d/%d", (int)c, (int)Count);
				return;
			}
			if (e != ERROR_IO_INCOMPLETE)
			{
				printf("Ошибка GetOverlappedResult: %d", e);
				return;
			}
		}
		ClearCommError(hPort, &e, &ComState);
		if (e)
		{
			printf("Unable to ClearCommError: %d", e);
			return;
		}
		if (ComState.cbOutQue)
		{
			printf("Не отправлено: %d", e);
			return;
		}
		//printf("Отправлено %d/%d", c, Count);
	}
	return;
}
