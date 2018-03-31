
#include<opencv.hpp>
#include<windows.h>
#include<iostream>
#include "stdlib.h"
#include <setupapi.h>
#include<algorithm>
#include<fstream>
#include<string.h>
#pragma comment(lib,"setupapi.lib")

#define RAD_NUM 190
using namespace std;
using namespace cv;

float w_mm = 100;                     //摄像头中心到激光发射器中心的距离10cm
float offset = 0.00583948;            //每个像素点对应的弧度值的误差补偿
float pixcel = 0.001859768;           //像素点对应的弧度制
float offset_z = 0;                   //误差补偿在z轴上的对应值
float pixcel_z = 0;                   //像素点在z轴上对应的弧度值

wchar_t** GetSerialNumber(int& len) {//通过读取设备管理器，得到可用的串口号
									 //先通过读取设备管理器，得到类型为“Ports”的设备，并把这些设备的友好类型记录下来
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
		if (!SetupDiGetDeviceRegistryProperty(hDevInfo, &DeviceInfoData, SPDRP_CLASS, NULL, (PBYTE)szClassBuf, MAX_PATH - 1, NULL)) continue;//先找到设备的类
		if (szClassBuf[0] == 'P'&&szClassBuf[1] == 'o'&&szClassBuf[2] == 'r'&&szClassBuf[3] == 't'&&szClassBuf[4] == 's') {//如果类型为'Ports'
			if (!SetupDiGetDeviceRegistryProperty(hDevInfo, &DeviceInfoData, SPDRP_FRIENDLYNAME, NULL, (PBYTE)szNameBuf[n++], MAX_PATH - 1, NULL)) continue;//找到设备的友好名称
		}
	}
	//通过Ports设备的友好类型，找到可用的COM口
	//也就是提取到“COMx”这样的字符串
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


/*串口连接函数，电脑和单片机通过此函数连接*/
int serial_connect(HANDLE& hCom, wchar_t* serialport, int baudrate) {
	hCom = CreateFile(serialport, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);//打开串口，允许读和写，异步方式
	if (hCom == (HANDLE)-1) return -1;
	SetupComm(hCom, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024 
	COMMTIMEOUTS TimeOuts; //设定读超时
	TimeOuts.ReadIntervalTimeout = 1000;
	TimeOuts.ReadTotalTimeoutMultiplier = 500;
	TimeOuts.ReadTotalTimeoutConstant = 5000; //设定写超时
	TimeOuts.WriteTotalTimeoutMultiplier = 500;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(hCom, &TimeOuts); //设置超时
	DCB dcb;
	GetCommState(hCom, &dcb);   //得到com口的状态  由于bcd的参数较多，一般将当前串口设置读出，存在bcd上，然后再修改对应参数
	dcb.BaudRate = baudrate;//设置波特率
	dcb.fParity = 0; // 指定奇偶校验使能。若此成员为1，允许奇偶校验检查 …
	dcb.ByteSize = 8; 
	dcb.Parity = NOPARITY; //指定奇偶校验方法。此成员可以有下列值： EVENPARITY 偶校验 NOPARITY 无校验 MARKPARITY 标记校验 ODDPARITY 奇校验
	dcb.StopBits = ONESTOPBIT; //指定停止位的位数。此成员可以有下列值： ONESTOPBIT 1位停止位 TWOSTOPBITS 2位停止位
	SetCommState(hCom, &dcb);//设置com口状态
	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);  //
	return 0;
}

/*串口发送函数，向串口缓存器中写入数据*/
int Write(HANDLE hCom, char* lpOutBuffer, int length) {//串口发送
	DWORD dwBytesWrite = (DWORD)length;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	OVERLAPPED m_osWrite;//建立Overlapped结构 
	m_osWrite.InternalHigh = 0;
	m_osWrite.Offset = 0;
	m_osWrite.OffsetHigh = 0;
	m_osWrite.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);//初始化Overlapped结构
	ClearCommError(hCom, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(hCom, lpOutBuffer, dwBytesWrite, &dwBytesWrite, &m_osWrite);//异步写串口
	if (!bWriteStat) {
		return -1;
	}
	PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	return 0;
}

/*读取串口缓存器中的数据*/
int Read(HANDLE hCom) {//串口接收
	DWORD dwBytesRead = 1000;
	char* str = new char[(int)dwBytesRead];
	for (int t = 0; t <= (int)dwBytesRead - 1; t++) str[t] = 0;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	OVERLAPPED m_osRead; //建立Overlapped结构
	memset(&m_osRead, 0, sizeof(OVERLAPPED));    //这是一个复制函数
	m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);    //初始化overlapped结构
	ClearCommError(hCom, &dwErrorFlags, &ComStat);
	dwBytesRead = min(dwBytesRead, (DWORD)ComStat.cbInQue);
	if (!dwBytesRead) {//如果缓冲区为空，那么这里就会返回
		return -1;
	}
	BOOL bReadStatus;
	bReadStatus = ReadFile(hCom, str, dwBytesRead, &dwBytesRead, &m_osRead);//异步读串口，读取缓冲区的内容
	if (!bReadStatus) {
		return -1;
	}
	if (dwBytesRead) printf("串口接受到的内容为:%d\r\n%s\r\n", (int)dwBytesRead, str);//如果读取出来的缓冲区的内容不为空，那么把读取出来的内容发送出来
	return 0;
}

/*获得系统时间函数，以方便对测试数据进行查看和管理*/
/*时间的输出格式为：  星期 月份 日 时 分 秒 年份    */
string GetTime()
{
	time_t timep;
	//struct tm t;
	char tmp[64];
	string timp;
	time(&timep);
	//localtime_s(&t, &timep);   //将time_t的时间转化为struct tm 类型
	ctime_s(tmp,64,&timep);     //将time_t类型的时间结构体转化为字符串并赋值到tmp字符数组中
	return tmp;
}

int main()
{
	HANDLE hCom;//串口句柄  创建结构体
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
	float av_pixy_last[480];
	int move_flag = 0;
	double dis_arange[480][6];
	double distance = 0;
	float PixToCenter = 0;
	float PizToCenter_z = 0;
	float dis_av = 0;
	float dis_x;
	float dis_y;
	float dis_z;
	string fps_char;
	string fpsString;
	string DisString;
	string Time;
	char send[20];

	VideoCapture cap(1);     //

	connectstate = serial_connect(hCom, L"\\\\.\\COM13", CBR_115200);   //打开10以上的串口需要加上\\\\.\\
	 /*打开串口*/
	if (connectstate == -1)
	{
		cout << "无法打开串口...";
		system("pause");
	}
	/*打开摄像头*/
	if (!cap.isOpened())
	{
		cout << "无法打开摄像头...";
		system("pause");
	}
	/*打开坐标输出文件*/
	ofstream outf;   //文件输入流
	outf.open("LaserRangeData.txt",ios::app);   //ios::app表示在未见末尾追加而不覆盖
	Time = GetTime();
	if (!outf) 
	{
		cout << "无法打开数据文件...\n";
		system("pause");
	}
	else outf << Time;
    

	while (!stop)        //USB摄像头640*480（col*row）
	{
		fps_t = (double)cv::getTickCount();      //记录开始时间
		cap >> frame;                            //摄像头的数据流复制到frame  MAT矩阵中
		cvtColor(frame, range, CV_BGR2GRAY);     //将frame中的图像转为灰度，储存在range中
		GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT); //高斯滤波
		blur(range, range, Size(3, 3));                               //滤波
		                                                             
		MaxPixel = 0;
		MaxHeight = 0;
		MaxWidth = 0;         //最亮点赋初值

		av_pixy = 0;            
		av_pixy_num = 0;      //最亮点坐标

		/*这里将图像亮度不高的点全部设置为0，软件二值化                     */
		/*由于图像的上半部分光线常常比较暗，图像的上半部分与下半部分分开处理*/
		/*按行遍历，行数小于200行与大于200行的单独处理                      */
		for(int i=0;i<range.rows;i++)
			if(i<250)
			for (int j = 0; j < range.cols; j++)
			{
				if ((range.data[i*range.step + j]) > 50)
					range.data[i*range.step + j] = 255;
				else range.data[i*range.step + j] = 0;
			}
		    else
				  for (int j = 0; j < range.cols; j++)
				  {
					  if ((range.data[i*range.step + j]) > 220)
						  range.data[i*range.step + j] = 255;
					  else range.data[i*range.step + j] = 0;
				  }
		vector<Vec4i> hierarchy;   //每个元素包含4个整数
		Canny(range, range_canny, 0, 200, 3); //利用canny算法检测边缘  输入必须为灰度图 新的图片存放在range_canny中   
		vector<vector<Point> > contours;  //定义二维浮点型向量，用来储存边界的坐标，在着边界的时候自动生成，这里给他开辟空间
		findContours(range_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //查找轮廓 并确定轮廓的建立与显示方法

		/*这里是用激光束中心点的坐标的平均值来计算距离，精度比另一种算法高*/
		if (begin_flag == 1)
		{
			av_point_num++;
			/*按行遍历每张图片，将每行中的最亮点的坐标存在dis_arange[480][6]二维数组里*/
			for (int i_raw = 0; i_raw < 480; i_raw++)    //按行遍历，查找最亮点
			{
				/*直接判断取平均*/
				for (int i = 0; i < range_canny.cols/2; i++)   //摄像头接收的光只可能在感光元件的右半部分，直接从中点向后寻找即可
				{
					if (range_canny.data[i_raw * range_canny.step + i] > 220)     //取轮廓图中每行上亮度大于220的点，表示边缘的点的坐标，左边缘或者右边缘，所有表示边缘的点的绝对坐标（从图像边沿算起）相加
					{
						av_pixy = av_pixy + i;
						av_pixy_num++;          //记录边缘点的个数，以求中心点的平均坐标
					}
				}
				/*计算一张图片中，最亮点的平均绝对坐标，用av_pixy表示*/
				if (av_pixy_num == 0)           //如果某张图片中的某行不存在边缘点，则直接算作上一次的值，以消除噪声干扰和计算错误
					av_pixy = av_pixy_last[i_raw];
				else
					av_pixy = av_pixy / av_pixy_num;


				av_pixy_last[i_raw] = av_pixy;
				dis_arange[i_raw][av_point_num-1] = av_pixy; //储存6张图片中的平均最亮点绝对坐标
			}
			/*收集完成6张图片，这时处理dis_arange[][]数组的数据                    */
			/*先进行冒泡排序，也是按行遍历所有的距离信息，挨个输出每个点的x,y,z坐标，输出到文件LaserRangeData.txt*/
			/*利用matlab进行进一步的重现 */
			if (av_point_num == 6)             //舵机的每个位置都取6张图片来就距离的平均值，以减少误差
			{
				for (int i_raw = 0; i_raw < 480; i_raw++)
				{
					//sort(dis_arange, dis_arange + 6);     //冒泡排序，将距离值从低到高排序，以便去掉一些误差特别大的点
					dis_av = (dis_arange[i_raw][1] + dis_arange[i_raw][2] + dis_arange[i_raw][3] + dis_arange[i_raw][4]) / 4.0;    //平均亮度中心的像素点坐标
					PixToCenter = abs(range.cols / 2 - dis_av);    //以摄像头感光元件正中心为坐标原点的绝对坐标
					PizToCenter_z = range.rows / 2 - i_raw;
					distance = w_mm / tan(PixToCenter * pixcel + offset);   //计算实际距离的公式  注意：以mm为单位，计算用的角度以rad为单位，并不是°为单位
					if (distance < 0)
						distance = -distance;
					/*这里计算以摄像头为中心的所测量的点的二维坐标，以便得到平面的坐标点云数据*/
					dis_x = cos(send_num*3.14 / RAD_NUM)*distance;
					dis_y = sin(send_num*3.14 / RAD_NUM)*distance;
					dis_z = tan(PizToCenter_z*pixcel+offset)*distance;
					outf << dis_x << "  " << dis_y << "  " << dis_z << "\r\n";				
				}
				cout << "distance:" << distance << "mm" << "\r\n";
				send_num++;                         //舵机旋转的次数，用来计算舵机旋转的弧度值，计算坐标时用 
				av_point_num = 0;                   //计数清零
				strcpy_s(send, "ABCDEF\r\n");       //使用串口发送字符串，控制舵机旋转一定角度
				Write(hCom, send, 8);//发送“ABCDE”
				Sleep(0.05);//延时0.5s
			}
		}

		///*计算距离*/
		//PixToCenter = abs(range.cols / 2 - MaxWidth);
		//PixToCenter = abs(range.cols / 2 - av_pixy);
		//distance=w_mm/ tan(PixToCenter * pixcel + offset);
		//if (distance < 0)
		//	distance = -distance;

		/*按键扫描，执行相应操作，开始或退出  ESC退出，空格开始，再次空格可恢复原始状态*/
		key_value = waitKey(30);
       if (key_value == 32)   //空格键值为32
		{   
		   if (begin_flag == 0)      //第一次按空格开始
		   {
			   begin_flag = 1;            
		   }
		   else if (begin_flag == 1)
		   {
			   strcpy_s(send, "GHGHGH\r\n");      //通过串口发送给单片机开始命令，以执行重新开始操作操作，舵机恢复原来角度，再次按空格可重新开始
			   Write(hCom, send, 8);              //发送8个字节的数据
			   begin_flag = 0;                    //改变标志位，不执行测距主循环中的内容                  
		   }
		} 
	   else if (key_value == 27)    //键盘Esc键值为27
	   {
		   stop = true;             //结束总循环流程
		   CloseHandle(hCom);       //关闭串口文件，保护系统内存
	   }
	  
	   /*计算帧率*/
	   fps_t = ((double)cv::getTickCount() - fps_t) / cv::getTickFrequency();   //读取系统时间，计算从上次读取到运行到此的时间
	   fps = 1.0 / fps_t;                 //计算帧率
	   DisString = to_string(dis_av);     
	   fpsString = "fps:" + to_string(fps) + "  cols:" + to_string(MaxWidth) + "  rows:" + to_string(MaxHeight);

	   /*在不同图像上显示相关信息，并设置字体颜色等内容*/
	   cv::putText(frame, fpsString, cv::Point(5, 20), cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 255));      
	   cv::putText(range, DisString, cv::Point(MaxWidth, MaxHeight), cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 255));

	   /*给每个图像窗命名*/
	   imshow("灰度图", range);
	   imshow("原图", frame);
	   imshow("canny", range_canny);

	   /*在第一次显示图像时固定图像出现的位置，防止图像出现时窗体在屏幕的外面而无法操作*/
	   if (move_flag == 0)
	   {
		   cvMoveWindow("原图", 10, 10);
		   cvMoveWindow("灰度图", 200, 10);
		   cvMoveWindow("canny", 10, 50);
		   move_flag = 1;
	   }

 	}
	/*释放图像空间，保护系统储存空间*/
	frame.release();
	range.release();
	range_canny.release();
	outf.close();
	return 0;
}

/*原点摄像头激光测距*/
//轮廓检测、计算轮廓质心坐标，筛选质心符合要求的轮廓，删除面积过小的轮廓
void yuandian(Mat range)
{
	Mat range_canny;
	vector<Vec4i> hierarchy;   //每个元素包含4个整数
	Canny(range, range_canny, 0, 200, 3); //利用canny算法检测边缘  输入必须为灰度图    
	vector<vector<Point> > contours;  //定义二维浮点型向量，用来储存边界的坐标，在着边界的时候自动生成，这里给他开辟空间
	findContours(range_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //查找轮廓 并确定轮廓的建立与显示方法
	vector<Moments> mu(contours.size());   //moments 轮廓距 直接储存在mu里
	for (int i = 0; i < contours.size(); i++)  //contouts.size()表示有多少条轮廓
	{
		mu[i] = moments(contours[i], false);
	}

	/*计算每条轮廓质心的坐标*/
	vector<Point2f> mc(contours.size());  //定义数据储存每条轮廓的中心
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	Mat drawing1 = Mat::zeros(range_canny.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)   //contours的每个元素代表一个轮廓
	{
		/*这里画出轮廓*/
		Scalar color = Scalar(255, 255, 255);
		drawContours(drawing1, contours, i, color, 2, 8, hierarchy, 0, Point());
		circle(drawing1, mc[i], 5, Scalar(0, 0, 255), -1, 8, 0);
		rectangle(drawing1, boundingRect(contours.at(i)), cvScalar(0, 255, 0));
		imshow("轮廓图",drawing1);
		/*满足此条件的既为PointToCenter*/
	}

	/*删除面积太小的轮廓*/
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
	drawContours(drawing, contours, -1, Scalar(255), 1);   // -1 表示所有轮廓
	imshow("删除后的图像", drawing);
}

