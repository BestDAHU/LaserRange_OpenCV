
#include<opencv.hpp>
#include<windows.h>
#include<iostream>
#include "stdlib.h"
#include <setupapi.h>
#include<algorithm>
#include<fstream>
#include<string.h>
#pragma comment(lib,"setupapi.lib")

#define RAD_NUM 380
using namespace std;
using namespace cv;

float w_mm = 100;
float offset = 0.00583948;
float pixcel = 0.001859768;


wchar_t** GetSerialNumber(int& len) {//ͨ����ȡ�豸���������õ����õĴ��ں�
									 //��ͨ����ȡ�豸���������õ�����Ϊ��Ports�����豸��������Щ�豸���Ѻ����ͼ�¼����
	HDEVINFO hDevInfo;
	SP_DEVINFO_DATA DeviceInfoData;
	DWORD t;
	int n = 0;
	wchar_t szClassBuf[MAX_PATH] = { 0 };
	wchar_t** szNameBuf = new wchar_t*[20];
	for (int t = 0; t <= 19; t++) szNameBuf[t] = new wchar_t[MAX_PATH];
	hDevInfo = SetupDiGetClassDevs(NULL, 0, 0, DIGCF_PRESENT | DIGCF_ALLCLASSES);
	if (hDevInfo == INVALID_HANDLE_VALUE) return NULL;
	DeviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	for (t = 0; SetupDiEnumDeviceInfo(hDevInfo, t, &DeviceInfoData); t++) {
		if (!SetupDiGetDeviceRegistryProperty(hDevInfo, &DeviceInfoData, SPDRP_CLASS, NULL, (PBYTE)szClassBuf, MAX_PATH - 1, NULL)) continue;//���ҵ��豸����
		if (szClassBuf[0] == 'P'&&szClassBuf[1] == 'o'&&szClassBuf[2] == 'r'&&szClassBuf[3] == 't'&&szClassBuf[4] == 's') {//�������Ϊ'Ports'
			if (!SetupDiGetDeviceRegistryProperty(hDevInfo, &DeviceInfoData, SPDRP_FRIENDLYNAME, NULL, (PBYTE)szNameBuf[n++], MAX_PATH - 1, NULL)) continue;//�ҵ��豸���Ѻ�����
		}
	}
	//ͨ��Ports�豸���Ѻ����ͣ��ҵ����õ�COM��
	//Ҳ������ȡ����COMx���������ַ���
	wchar_t** serialnumber = new wchar_t*[n];
	SetupDiDestroyDeviceInfoList(hDevInfo);
	for (int m = 0; m <= n - 1; m++) {
		int atleft = 0;
		int atright = 0;
		for (int t1 = 0; t1 <= MAX_PATH - 1; t1++) {
			if (szNameBuf[m][t1] == '(') atleft = t1;
			if (szNameBuf[m][t1] == ')') atright = t1;
		}
		if (atleft == 0 || atright == 0) continue;
		serialnumber[m] = new wchar_t[6];
		for (int t2 = 0; t2 <= 5; t2++) serialnumber[m][t2] = 0;
		for (int t3 = 0; t3<atright - atleft - 1; t3++) {
			serialnumber[m][t3] = szNameBuf[m][t3 + atleft + 1];
		}
	}
	len = n;
	return serialnumber;
}


/*�������Ӻ��������Ժ͵�Ƭ��ͨ���˺�������*/
int serial_connect(HANDLE& hCom, wchar_t* serialport, int baudrate) {
	hCom = CreateFile(serialport, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);//�򿪴��ڣ��������д���첽��ʽ
	if (hCom == (HANDLE)-1) return -1;
	SetupComm(hCom, 1024, 1024); //���뻺����������������Ĵ�С����1024 
	COMMTIMEOUTS TimeOuts; //�趨����ʱ
	TimeOuts.ReadIntervalTimeout = 1000;
	TimeOuts.ReadTotalTimeoutMultiplier = 500;
	TimeOuts.ReadTotalTimeoutConstant = 5000; //�趨д��ʱ
	TimeOuts.WriteTotalTimeoutMultiplier = 500;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(hCom, &TimeOuts); //���ó�ʱ
	DCB dcb;
	GetCommState(hCom, &dcb);   //�õ�com�ڵ�״̬  ����bcd�Ĳ����϶࣬һ�㽫��ǰ�������ö���������bcd�ϣ�Ȼ�����޸Ķ�Ӧ����
	dcb.BaudRate = baudrate;//���ò�����
	dcb.fParity = 0; // ָ����żУ��ʹ�ܡ����˳�ԱΪ1��������żУ���� ��
	dcb.ByteSize = 8; 
	dcb.Parity = NOPARITY; //ָ����żУ�鷽�����˳�Ա����������ֵ�� EVENPARITY żУ�� NOPARITY ��У�� MARKPARITY ���У�� ODDPARITY ��У��
	dcb.StopBits = ONESTOPBIT; //ָ��ֹͣλ��λ�����˳�Ա����������ֵ�� ONESTOPBIT 1λֹͣλ TWOSTOPBITS 2λֹͣλ
	SetCommState(hCom, &dcb);//����com��״̬
	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);  //
	return 0;
}

/*���ڷ��ͺ������򴮿ڻ�������д������*/
int Write(HANDLE hCom, char* lpOutBuffer, int length) {//���ڷ���
	DWORD dwBytesWrite = (DWORD)length;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	OVERLAPPED m_osWrite;//����Overlapped�ṹ 
	m_osWrite.InternalHigh = 0;
	m_osWrite.Offset = 0;
	m_osWrite.OffsetHigh = 0;
	m_osWrite.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);//��ʼ��Overlapped�ṹ
	ClearCommError(hCom, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(hCom, lpOutBuffer, dwBytesWrite, &dwBytesWrite, &m_osWrite);//�첽д����
	if (!bWriteStat) {
		return -1;
	}
	PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	return 0;
}

/*��ȡ���ڻ������е�����*/
int Read(HANDLE hCom) {//���ڽ���
	DWORD dwBytesRead = 1000;
	char* str = new char[(int)dwBytesRead];
	for (int t = 0; t <= (int)dwBytesRead - 1; t++) str[t] = 0;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	OVERLAPPED m_osRead; //����Overlapped�ṹ
	memset(&m_osRead, 0, sizeof(OVERLAPPED));    //����һ�����ƺ���
	m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);    //��ʼ��overlapped�ṹ
	ClearCommError(hCom, &dwErrorFlags, &ComStat);
	dwBytesRead = min(dwBytesRead, (DWORD)ComStat.cbInQue);
	if (!dwBytesRead) {//���������Ϊ�գ���ô����ͻ᷵��
		return -1;
	}
	BOOL bReadStatus;
	bReadStatus = ReadFile(hCom, str, dwBytesRead, &dwBytesRead, &m_osRead);//�첽�����ڣ���ȡ������������
	if (!bReadStatus) {
		return -1;
	}
	if (dwBytesRead) printf("���ڽ��ܵ�������Ϊ:%d\r\n%s\r\n", (int)dwBytesRead, str);//�����ȡ�����Ļ����������ݲ�Ϊ�գ���ô�Ѷ�ȡ���������ݷ��ͳ���
	return 0;
}

int main()
{
	HANDLE hCom;//���ھ��  �����ṹ��
	int len;
	int key_value = 0;
	int connectstate ;
	Mat frame;
	Mat range;
	Mat range_canny;
	bool stop = false;
	float fps = 0;
	//char fps_char[10];
	double fps_t = 0;
	int begin_flag = 0;
	int MaxPixel=0;
	int MaxHeight=0;
	int MaxWidth=0;
	int i = 0;     //width
	int j = 0;      //height
	int photo_num = 0;
	int send_num = 0;
	float av_pixy=0;
	int av_pixy_num = 0;
	int av_point_num=0;
	float av_pixy_last = 0;
	int move_flag = 0;
	double dis_arange[10];
	double distance = 0;
	float PixToCenter = 0;
	float dis_av = 0;
	float dis_x;
	float dis_y;
	string fps_char;
	string fpsString;
	string DisString;
	char send[20];
	VideoCapture cap(1);

	connectstate = serial_connect(hCom, L"\\\\.\\COM13", CBR_115200);   //��10���ϵĴ�����Ҫ����\\\\.\\
	 /*�򿪴���*/
	if (connectstate == -1)
	{
		cout << "�޷��򿪴���";
		system("pause");
	}
	/*������ͷ*/
	if (!cap.isOpened())
	{
		cout << "�޷�������ͷ";
		system("pause");
	}
	/*����������ļ�*/
	ofstream outf;   //�ļ�������
	outf.open("LaserRangeData.txt");

	while (!stop)        //USB����ͷ640*480��col*row��
	{
		fps_t = (double)cv::getTickCount();      //��¼��ʼʱ��
		cap >> frame;                            //����ͷ�����������Ƶ�frame  MAT������
		cvtColor(frame, range, CV_BGR2GRAY);     //��frame�е�ͼ��תΪ�Ҷȣ�������range��
		GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT); //��˹�˲�
		blur(range, range, Size(3, 3));                               //�˲�
		                                                             //
		MaxPixel = 0;
		MaxHeight = 0;
		MaxWidth = 0;         //�����㸳��ֵ

		av_pixy = 0;            
		av_pixy_num = 0;      //����������

		/*���ｫͼ�����Ȳ��ߵĵ�ȫ������Ϊ0�������ֵ��*/
		for(int i=0;i<range.cols;i++)
			for (int j = 0; j < range.rows; j++)
			{
				if ((range.data[j*range.step + i]) > 150)
					range.data[j*range.step + i] = 255;
				else range.data[j*range.step + i] = 0;
			}


		vector<Vec4i> hierarchy;   //ÿ��Ԫ�ذ���4������
		Canny(range, range_canny, 0, 200, 3); //����canny�㷨����Ե  �������Ϊ�Ҷ�ͼ    
		vector<vector<Point> > contours;  //�����ά��������������������߽�����꣬���ű߽��ʱ���Զ����ɣ�����������ٿռ�
		findContours(range_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //�������� ��ȷ�������Ľ�������ʾ����

		/*ֱ���ж�ȡƽ��*/
		for (int i = range_canny.cols / 2; i < range_canny.cols; i++)
		{
			if (range_canny.data[220 * range_canny.step + i] > 220)
			{
				av_pixy = av_pixy + i;
				av_pixy_num++;
			}
		}
		if (av_pixy_num == 0)
			av_pixy = av_pixy_last;
		else
		av_pixy = av_pixy / av_pixy_num;

		av_pixy_last = av_pixy;
		/*�������ü��������ĵ�������ƽ��ֵ��������룬���ȱ���һ���㷨��*/
		if (begin_flag == 1)
		{
			dis_arange[av_point_num] = av_pixy;
			av_point_num++;
			if (av_point_num == 10)
			{
			sort(dis_arange,dis_arange+10);
			strcpy_s(send, "ABCDEF\r\n");
			Write(hCom, send, 8);//���͡�ABCDE��
			Sleep(5);//��ʱ0.5s
			av_point_num = 0;
			dis_av = ( dis_arange[3] + dis_arange[4] + dis_arange[5]+ dis_arange[6]) / 4.0;
			PixToCenter = abs(range.cols / 2 - dis_av);
			distance = w_mm / tan(PixToCenter * pixcel + offset);
			if (distance < 0)
				distance = -distance;
			send_num++;
			dis_x = cos(send_num*3.14 / RAD_NUM)*distance;
			dis_y = sin(send_num*3.14 / RAD_NUM)*distance;
			outf << dis_x << "  " << dis_y << "  " << 0 << "\r\n";
			cout <<"distance:"<< distance <<"mm"<<"\r\n";
			}
		}

		/*�������*/
		PixToCenter = abs(range.cols / 2 - MaxWidth);
		PixToCenter = abs(range.cols / 2 - av_pixy);
		distance=w_mm/ tan(PixToCenter * pixcel + offset);
		if (distance < 0)
			distance = -distance;

		key_value = waitKey(30);
       if (key_value == 32)   //�ո��ֵΪ32
		{   
		   if (begin_flag == 0)
		   {
			   begin_flag = 1;            
		   }
		   else if (begin_flag == 1)
		   {
			   strcpy_s(send, "GHGHGH\r\n");
			   Write(hCom, send, 8);
			   begin_flag = 0;    
		   }
		} 
	   else if (key_value == 27)    //����Esc��ֵΪ27
	   {
		   stop = true;
		   CloseHandle(hCom);
	   }
	  
	   /*����֡��*/
	   fps_t = ((double)cv::getTickCount() - fps_t) / cv::getTickFrequency();
	   fps = 1.0 / fps_t;
	   DisString = to_string(dis_av);
	   fpsString = "fps:" + to_string(fps) + "  cols:" + to_string(MaxWidth) + "  rows:" + to_string(MaxHeight);

	   /*��ͼ������ʾ�����Ϣ*/
	   cv::putText(frame, fpsString, cv::Point(5, 20), cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 255));
	   cv::putText(range, DisString, cv::Point(MaxWidth, MaxHeight), cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 255));

	   imshow("�Ҷ�ͼ", range);
	   imshow("ԭͼ", frame);
	   imshow("canny", range_canny);
	   if (move_flag == 0)
	   {
		   cvMoveWindow("ԭͼ", 10, 10);
		   cvMoveWindow("�Ҷ�ͼ", 200, 10);
		   cvMoveWindow("canny", 10, 50);
		   move_flag = 1;
	   }

 	}
	frame.release();
	range.release();
	range_canny.release();
	outf.close();
	return 0;
}

/*ԭ������ͷ������*/
//������⡢���������������꣬ɸѡ���ķ���Ҫ���������ɾ�������С������
void yuandian(Mat range)
{
	Mat range_canny;
	vector<Vec4i> hierarchy;   //ÿ��Ԫ�ذ���4������
	Canny(range, range_canny, 0, 200, 3); //����canny�㷨����Ե  �������Ϊ�Ҷ�ͼ    
	vector<vector<Point> > contours;  //�����ά��������������������߽�����꣬���ű߽��ʱ���Զ����ɣ�����������ٿռ�
	findContours(range_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //�������� ��ȷ�������Ľ�������ʾ����
	vector<Moments> mu(contours.size());   //moments ������ ֱ�Ӵ�����mu��
	for (int i = 0; i < contours.size(); i++)  //contouts.size()��ʾ�ж���������
	{
		mu[i] = moments(contours[i], false);
	}

	/*����ÿ���������ĵ�����*/
	vector<Point2f> mc(contours.size());  //�������ݴ���ÿ������������
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	Mat drawing1 = Mat::zeros(range_canny.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)   //contours��ÿ��Ԫ�ش���һ������
	{
		/*���ﻭ������*/
		Scalar color = Scalar(255, 255, 255);
		drawContours(drawing1, contours, i, color, 2, 8, hierarchy, 0, Point());
		circle(drawing1, mc[i], 5, Scalar(0, 0, 255), -1, 8, 0);
		rectangle(drawing1, boundingRect(contours.at(i)), cvScalar(0, 255, 0));
		imshow("����ͼ",drawing1);
		/*����������ļ�ΪPointToCenter*/
	}

	/*ɾ�����̫С������*/
	vector <vector<Point>>::iterator iter = contours.begin();
	for (; iter != contours.end();)
	{
		double g_dConArea = contourArea(*iter);
		if (g_dConArea < 10)
		{
			iter = contours.erase(iter);
		}
		else
		{
			++iter;
		}
	}
	Mat drawing = Mat::zeros(range_canny.size(), CV_8UC3);
	drawContours(drawing, contours, -1, Scalar(255), 1);   // -1 ��ʾ��������
	imshow("ɾ�����ͼ��", drawing);
}

