//#define CV_WIN

#include <wpb_local_planner/wl_helper.h>
#include <string.h>
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>

static int y_offset = 4;

#define MAP_WIDTH 100
#define MAP_HEIGHT 100

#ifdef CV_WIN
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
cv::Mat loc_map( MAP_WIDTH, MAP_HEIGHT, CV_8UC3, Scalar(0,255,0) );
cv::Mat loc_map_fliped( MAP_WIDTH, MAP_HEIGHT, CV_8UC3, Scalar(0,255,0) );
#endif

static unsigned char map_obstacle[MAP_WIDTH*MAP_HEIGHT];
static unsigned char map_target[MAP_WIDTH*MAP_HEIGHT];
static unsigned char map_path[MAP_WIDTH*MAP_HEIGHT];
static unsigned char map_back[MAP_WIDTH*MAP_HEIGHT];
static int loc_path_x[MAP_WIDTH*MAP_HEIGHT];
static int loc_path_y[MAP_WIDTH*MAP_HEIGHT];
static int nLocPathLenght;
static unsigned char TempSinglePoint[21 * 21];
static unsigned char TempSinglePath[255 * 255];
static int nNumber = 720;
static double x_sin[720];
static double y_cos[720];
static int pnt_x[720];
static int pnt_y[720];
static float move_x = 0;
static float move_y = 0;

void InitHelper()
{
	//���ݻ���
	memset(map_obstacle, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_target, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_path, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_back, 0, MAP_WIDTH*MAP_HEIGHT);
	//�����״��ģ��
    SetBorder(0.45);

	//·����ģ��
	memset(TempSinglePath, 0, 255 * 255);
	for (int y = 0; y < 255; y++)
	{
		for (int x = 0; x < 255; x++)
		{
			double fx = x;
			double fy = y;
			int val = sqrt(double(fx - 127)*(fx - 127) + double(fy - 127)*(fy - 127));
			if (val > 127)
			{
				val = 127;
			}
			TempSinglePath[y * 255 + x] = (127 - val) * 1;
		}
	}

	double kStep = (M_PI * 2) / nNumber;
	for (int i = 0; i < nNumber; i++)
	{
		x_sin[i] = 20*sin(M_PI*1.0 - kStep*i);
		y_cos[i] = -20*cos(M_PI*1.0 - kStep*i);
	}
	
#ifdef CV_WIN
	cv::namedWindow("map",CV_WINDOW_NORMAL);
	cvResizeWindow("map", 400, 400); 
#endif
}

int ResetNum(int inNum)
{
	double kStep = (M_PI * 2) / inNum;
	for (int i = 0; i < inNum; i++)
	{
		x_sin[i] = 20*sin(M_PI*1.0 - kStep*i);
		y_cos[i] = -20*cos(M_PI*1.0 - kStep*i);
	}
	nNumber = inNum;
}

void SetObst(int inX, int inY)
{
	int nFixedY = inY + y_offset;
	if(inX < 0 || inX >= MAP_WIDTH || nFixedY < 0 || nFixedY>=MAP_HEIGHT)
		return;
	//map_obstacle[nFixedY*MAP_WIDTH + inX] = 0xff;

	int nDepthR = 10;
	for (int y = nFixedY - nDepthR; y <= nFixedY + nDepthR; y++)
	{
		for (int x = inX - nDepthR; x <= inX + nDepthR; x++)
		{
			if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
			{
				unsigned char* pMapPoint = map_obstacle + y*MAP_WIDTH + x;
				int tx = x - inX + nDepthR;
				int ty = y - nFixedY + nDepthR;

				unsigned char* pTmpMark = TempSinglePoint + ty * 21 + tx;
				if (*pMapPoint < *pTmpMark)
				{
					*pMapPoint = *pTmpMark;
				}
			}
		}
	}
}

void SetRanges(float* inData)
{
	ClearObst();
	int i =0;
	for(i=0;i<nNumber;i++)
	{
    	pnt_x[i] = MAP_WIDTH/2 + inData[i] * x_sin[i];
		pnt_y[i] = MAP_HEIGHT/2 + inData[i] * y_cos[i];
		//printf("[%d] (%d , %d)\n",i,pnt_x[i],pnt_y[i]);
		SetObst(pnt_x[i],pnt_y[i]);
	}
}

void SetBorder(float inBorder)
{
    int nBorder = inBorder*20;
    int x_range = nBorder/2;
    int y_range = x_range*1.5;
    for (int y = 0; y < 21; y++)
    {
        for (int x = 0; x < 21; x++)
        {
            int x_offset = abs(x - 10);
            int y_offset = abs(y - 10);

            if (x_offset <= x_range && x_offset <= y_range)
            {
                TempSinglePoint[y * 21 + x] = 0xff;
            }
            else
            {
                TempSinglePoint[y * 21 + x] = 0;
            }
        }
    }
}

void ClearObst()
{
	memset(map_obstacle, 0, MAP_WIDTH*MAP_HEIGHT);
}

void SetTarget(int inX, int inY, bool inForce)
{
	int nFixedY = inY + y_offset;
	if( (inX >=0 && inX< MAP_WIDTH && nFixedY >= 0 && nFixedY<MAP_HEIGHT) || (inForce == true) )
	{
		map_target[nFixedY*MAP_WIDTH + inX] = 0xff;

		int nDepthR = 126;
		for (int y = nFixedY - nDepthR; y <= nFixedY + nDepthR; y++)
		{
			for (int x = inX - nDepthR; x <= inX + nDepthR; x++)
			{
				if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
				{
					unsigned char* pMapPoint = map_target + y*MAP_WIDTH + x;
					int tx = x - inX + nDepthR;
					int ty = y - nFixedY + nDepthR;

					unsigned char* pTmpMark = TempSinglePath + ty * 255 + tx;
					if (*pMapPoint < *pTmpMark)
					{
						*pMapPoint = *pTmpMark;
					}
				}
			}
		}
	}
}

void ClearTarget()
{
	memset(map_target, 0, MAP_WIDTH*MAP_HEIGHT);
}

bool ChkTarget(int inX, int inY)
{
	bool res = true;
	if(inX >=0 && inX< MAP_WIDTH && inY >= 0 && inY < MAP_HEIGHT)
	{
		if(map_obstacle[inY*MAP_WIDTH+inX] == 0xff)
		{	
			res = false;
		}
	}
	return res;
}

void SetBack(int inX, int inY)
{
	if(inX < 0 || inX >= MAP_WIDTH || inY < 0 || inY>=MAP_HEIGHT)
	return;

	map_back[inY*MAP_WIDTH + inX] = 0xff;

	int nDepthR = 126;
	for (int y = inY - nDepthR; y <= inY + nDepthR; y++)
	{
		for (int x = inX - nDepthR; x <= inX + nDepthR; x++)
		{
			if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
			{
				unsigned char* pMapPoint = map_back + y*MAP_WIDTH + x;
				int tx = x - inX + nDepthR;
				int ty = y - inY + nDepthR;

				unsigned char* pTmpMark = TempSinglePath + ty * 255 + tx;
				if (*pMapPoint < *pTmpMark)
				{
					*pMapPoint = *pTmpMark;
				}
			}
		}
	}
}

int calPointCost(int inX, int inY)
{
	if(inX < 0 || inX >= MAP_WIDTH || inY < 0 || inY>=MAP_HEIGHT)
		return 999;

	int retVal;
	if (map_obstacle[inY*MAP_WIDTH + inX] == 0xff || map_path[inY*MAP_WIDTH + inX] > 0)
	{
		retVal = 999;
	}
	else
	{
		retVal = (int)map_obstacle[inY*MAP_WIDTH + inX] - map_target[inY*MAP_WIDTH + inX];
	}
	return retVal;
}

static int cur_x = 0;
static int cur_y = 0;
bool PathGrow()
{
	bool res = false;
	int tx = cur_x;
	int ty = cur_y;
	int minCost = 256;
	int nx, ny,tCost;
		
	//上
	nx = cur_x - 0;
	ny = cur_y + 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}

	//左上
	nx = cur_x - 1;
	ny = cur_y + 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	
	//右上
	nx = cur_x + 1;
	ny = cur_y + 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}

	//下
	nx = cur_x + 0;
	ny = cur_y - 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//左下
	nx = cur_x - 1;
	ny = cur_y - 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//右下
	nx = cur_x + 1;
	ny = cur_y - 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//左
	nx = cur_x - 1;
	ny = cur_y - 0;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//右
	nx = cur_x + 1;
	ny = cur_y + 0;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}

	cur_x = tx;
	cur_y = ty;
	map_path[cur_y*MAP_WIDTH + cur_x] = 150;
	return res;
}

void ShowMap()
{
#ifdef CV_WIN
	unsigned char* pObstacleMap = map_obstacle;
	unsigned char* pTargetMap = map_target;
	unsigned char* pPathMap = map_path;

	unsigned char *uc_pixel = loc_map.data;
	int height = loc_map.rows;
    int width = loc_map.cols;
    for(int row=0; row < height; row++)
	{
        for(int col=0; col < width; col++)
		{
            uc_pixel[0] = (*pTargetMap);
            uc_pixel[1] = (*pObstacleMap) ;
            uc_pixel[2] = (*pPathMap);

			pObstacleMap++;
			pTargetMap++;
			pPathMap++;
			uc_pixel += 3;
        }
    }

	uc_pixel = loc_map.data;
	uc_pixel += (MAP_HEIGHT*MAP_WIDTH*3)/2 + MAP_WIDTH*3/2;
	uc_pixel[0] = 0xff;
	uc_pixel[1] = 0xff ;
	uc_pixel[2] = 0xff;
		
	cv::flip(loc_map, loc_map_fliped, -1);
	cv::imshow("map", loc_map_fliped);
    cv::waitKey(1);
#endif
}

void StartPos()
{
	cur_x = MAP_WIDTH / 2;
	cur_y = MAP_HEIGHT / 2;
	if(map_obstacle[cur_y*MAP_WIDTH + cur_x] == 0xff)
	{
		for(int tr=1;tr<MAP_HEIGHT/2;tr++)
		{
			for(int ty = MAP_HEIGHT/2-tr; ty < MAP_HEIGHT/2+tr; ty++)
			{
				for(int tx = MAP_WIDTH/2-tr; tx < MAP_WIDTH/2+tr; tx++)
				{
					if(tx >=0 && tx< MAP_WIDTH && ty >= 0 && ty < MAP_HEIGHT)
						if(map_obstacle[ty*MAP_WIDTH + tx] != 0xff)
						{
							cur_x = tx;
							cur_y = ty;
							return;
						}
				}
			}
		}
	}
}

bool OutLine()
{
	bool res = false;
	memset(map_path, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_back, 0, MAP_WIDTH*MAP_HEIGHT);
	nLocPathLenght = 0;
	StartPos();
	SetBack(cur_x, cur_y);

	// ������
	//map_path[cur_y*MAP_WIDTH + cur_x] = 0xff;
	while (PathGrow() == true)
	{
		loc_path_x[nLocPathLenght] = cur_x;
		loc_path_y[nLocPathLenght] = cur_y;
		nLocPathLenght++;
		if (map_target[cur_y*MAP_WIDTH + cur_x] == 0xff)
		{
			res = true;
			break;
		}
	}

	// 寻找移动方向
	if(nLocPathLenght > 0)
	{
		int path_index = 0;
		while(path_index < nLocPathLenght)
		{
			int nDist = sqrt((loc_path_x[path_index] - MAP_WIDTH/2)*(loc_path_x[path_index] - MAP_WIDTH/2) + (loc_path_y[path_index] - MAP_HEIGHT/2)*(loc_path_y[path_index] - MAP_HEIGHT/2));
			if(nDist > 2)
			{
				break;
			}
			else
			{
				path_index ++;
			}
			
		}
		move_x = (loc_path_y[path_index] - MAP_HEIGHT/2) * 0.05;
		move_y = (loc_path_x[path_index] - MAP_WIDTH/2) * 0.05;
	}
	else
	{
		move_x = 0;
		move_y = 0;
		
		res = false;
	}

	ShowMap();

	return res;
}

float CalAngle(float inX, float inY)
{
	float tan = inY / inX;
	float angle = atan(tan);
	if (inX < 0)
	{
		angle = M_PI + angle;
	}
	if (angle > M_PI)
	{
		angle -= 2 * M_PI;
	}
	return angle;
}

int GetHelperNum()
{
	return nLocPathLenght;
}

float GetFixX()
{
	return move_x;
}

float GetFixY()
{
	return move_y;
}

float GetFaceX()
{
	float retVal = 0.0f;
	if(nLocPathLenght > 0)
		retVal = (loc_path_y[nLocPathLenght-1] - MAP_HEIGHT/2) * 0.05;
	return retVal;
}

float GetFaceY()
{
	float retVal = 0.0f;
	if(nLocPathLenght > 0)
		retVal = (loc_path_x[nLocPathLenght-1] - MAP_WIDTH/2) * 0.05;
	return retVal;
}
