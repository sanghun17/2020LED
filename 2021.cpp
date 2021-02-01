#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <iostream>
#include <string>
#include <time.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#define PI 3.141592


using namespace cv;
using namespace std;



//if LED square is detected and they are far, return 1
bool checkfar(bool mark_flag[], double maxamonggr1gr2)
{
	return (mark_flag[0] && mark_flag[1] && mark_flag[2] && mark_flag[3] && maxamonggr1gr2 < 350);
}
Mat func_medianblur(Mat src, int i)
{
	Mat dst;
	for (int j = 0; j < i - 1; j++)
	{
		medianBlur(src, src, 3);
	}
	medianBlur(src, dst, 3);
	return dst;
}

Mat func_inrange_r(Mat src_rgb, Mat src_hsv, int RESIZE_LED)
{
	Mat dst, dst_temp;
	inRange(src_hsv, Scalar(0, 20, 80), Scalar(35, 255, 255), dst);
	inRange(src_hsv, Scalar(10, 40, 220), Scalar(60, 255, 255), dst_temp);

	uchar* r2_data = (uchar*)dst_temp.data;
	uchar* r_data = (uchar*)dst.data;
	int length = (src_rgb.cols / RESIZE_LED) * (src_rgb.rows / RESIZE_LED);
	bitwise_or(dst, dst_temp, dst);
	uchar *image_data = src_rgb.data;
	for (int y = 0; y < src_rgb.rows; y++)
	{
		uchar* image_data = src_rgb.ptr<uchar>(y);
		for (int x = 0; x < src_rgb.cols; x++)
		{
			uchar b = image_data[x * 3];
			uchar g = image_data[x * 3 + 1];
			uchar r = image_data[x * 3 + 2];

			if (r < (g + b) / 2)
				r_data[y*src_rgb.cols + x] = 0;
		}

	}
	return dst;
}

Mat func_inrange_g(Mat src_rgb, Mat src_hsv, int RESIZE_LED)
{
	Mat dst;
	inRange(src_hsv, Scalar(61, 100, 120), Scalar(100, 255, 255), dst);
	uchar* g_data = (uchar*)dst.data;
	int length = (src_rgb.cols / RESIZE_LED) * (src_rgb.rows / RESIZE_LED);
	uchar *image_data = src_rgb.data;
	for (int y = 0; y < src_rgb.rows; y++)
	{
		uchar* image_data = src_rgb.ptr<uchar>(y);
		for (int x = 0; x < src_rgb.cols; x++)
		{
			uchar b = image_data[x * 3];
			uchar g = image_data[x * 3 + 1];
			uchar r = image_data[x * 3 + 2];

			if (g < (r + b) / 2)
				g_data[y*src_rgb.cols + x] = 0;
		}

	}
	return dst;
}

int signextract(int a)
{
	if (a >= 0)
		return 1;
	if (a < 0)
		return -1;
}

void func_findcandidate(vector<vector<Point>>contours, int(*mark_temp)[2], int(*mark)[2], int candidate[], int RESIZE_LED, int& cnt, int roi_dis, int lednum, int far);


void axialsymmetry(int mark1[], int mark2[], int srcmark[], int result[])
{
	int x0 = mark1[0];
	int y0 = mark1[1];

	int x1 = mark2[0];
	int y1 = mark2[1];

	int x_src = srcmark[0];
	int y_src = srcmark[1];

	float a = (float)(y1 - y0) / (x1 - x0);
	float b = (float)y1 - a * x1;

	result[0] = (int)(((1 - a * a)*x_src + 2 * a*y_src - 2 * a*b) / (1 + a * a));
	result[1] = (int)((2 * a*x_src - (1 - a * a)*y_src + 2 * b) / (1 + a * a));
}




int main(int argc, char** argv) {



	double error_aspectratio = 0.3;
	double error_arearatio = 0.2;
	double error_radian = 4.5;
	double error_distance = 0.1;

	//int mark[4][2] = { 0 };
	int mark[4][2] = { {1126, 390}, {878, 382}, {874 , 621}, {1124, 625} };
	bool far = 0;
	int RESIZE_LED;
	if (far == 1)
		RESIZE_LED = 1;
	else
		RESIZE_LED = 2;

	int mark_temp_r[100][2] = { 0 };
	int mark_temp_g[100][2] = { 0 };
	int g_candidate[100] = { 0 };
	int r_candidate[100] = { 0 };
	bool mark_flag[4] = { false, false, false,false };
	int detected = 0;


	vector<vector<Point> > contours_g, contours_r, contours_r1, contours_r2, contours_r3;
	vector<Vec4i> hierarchy_g, hierarchy_r, hierarchy_r1, hierarchy_r2, hierarchy_r3;
	Mat image_resized;

	int g_cnt, r_cnt;
	double mark_center[2];
	int vertical_cnt = 0;
	int diagonal_cnt1 = 0;
	int diagonal_cnt2 = 0;
	int r_vertical = 0;
	int r_diagonal_1 = 0;
	int r_diagonal_2[2] = { 0 };
	int r_diagoanl_2_tri = 0;
	double gr1_size, gr2_size;

	int roi_dis = 80;

	Mat drawing_g, drawing_r;

	clock_t start1, start2, start3, start4, start5, start6, start7, end1, end2, end3, end4, end5, end6, end7;
	string imageName = "0910 (89).bmp";

	if (argc > 1) {
		imageName = argv[1];
	}

	Mat image_input = imread(imageName.c_str(), IMREAD_COLOR); // Read the file

	if (image_input.empty()) {
		std::cout << "Could not open or find the image" << std::endl;
		return -1;
	}




	if (far == 1)
	{
		start1 = clock();
		std::cout << "!!!!!!i'm in far state!!!!!!!!!!!!!!!" << endl;
		Rect roi_g((int)mark[0][0] - roi_dis, (int)mark[0][1] - roi_dis, 2 * roi_dis, 2 * roi_dis);
		Mat roi_g_image = image_input(roi_g);
		Mat roi_g_image_hsv;
		cv::cvtColor(roi_g_image, roi_g_image_hsv, COLOR_BGR2HSV);
		Mat image_binary_g = func_inrange_g(roi_g_image, roi_g_image_hsv, RESIZE_LED);
		image_binary_g = func_medianblur(image_binary_g, 6);
		cv::findContours(image_binary_g, contours_g, hierarchy_g, 0, 1, Point());


		Rect roi_r1((int)mark[1][0] - roi_dis, (int)mark[1][1] - roi_dis, 2 * roi_dis, 2 * roi_dis);
		Mat roi_r1_image = image_input(roi_r1);
		Mat roi_r1_image_hsv;
		cv::cvtColor(roi_r1_image, roi_r1_image_hsv, COLOR_BGR2HSV);
		Mat image_binary_r1 = func_inrange_r(roi_r1_image, roi_r1_image_hsv, RESIZE_LED);
		image_binary_r1 = func_medianblur(image_binary_r1, 6);
		cv::findContours(image_binary_r1, contours_r1, hierarchy_r1, 0, 1, Point());

		Rect roi_r2((int)mark[2][0] - roi_dis, (int)mark[2][1] - roi_dis, 2 * roi_dis, 2 * roi_dis);
		Mat roi_r2_image = image_input(roi_r2);
		Mat roi_r2_image_hsv;
		cv::cvtColor(roi_r2_image, roi_r2_image_hsv, COLOR_BGR2HSV);
		Mat image_binary_r2 = func_inrange_r(roi_r2_image, roi_r2_image_hsv, RESIZE_LED);
		image_binary_r2 = func_medianblur(image_binary_r2, 6);
		cv::findContours(image_binary_r2, contours_r2, hierarchy_r2, 0, 1, Point());

		Rect roi_r3((int)mark[3][0] - roi_dis, (int)mark[3][1] - roi_dis, 2 * roi_dis, 2 * roi_dis);
		Mat roi_r3_image = image_input(roi_r3);
		Mat roi_r3_image_hsv;
		cv::cvtColor(roi_r3_image, roi_r3_image_hsv, COLOR_BGR2HSV);
		Mat image_binary_r3 = func_inrange_r(roi_r3_image, roi_r3_image_hsv, RESIZE_LED);
		image_binary_r3 = func_medianblur(image_binary_r3, 6);
		cv::findContours(image_binary_r3, contours_r3, hierarchy_r3, 0, 1, Point());
		contours_r.insert(std::end(contours_r), std::begin(contours_r1), std::end(contours_r1));
		contours_r.insert(std::end(contours_r), std::begin(contours_r2), std::end(contours_r2));
		contours_r.insert(std::end(contours_r), std::begin(contours_r3), std::end(contours_r3));
		hierarchy_r.insert(std::end(hierarchy_r), std::begin(hierarchy_r1), std::end(hierarchy_r1));
		hierarchy_r.insert(std::end(hierarchy_r), std::begin(hierarchy_r2), std::end(hierarchy_r2));
		hierarchy_r.insert(std::end(hierarchy_r), std::begin(hierarchy_r3), std::end(hierarchy_r3));

		end1 = clock();
		start2 = clock();
		end2 = clock();
		start3 = clock();
		end3 = clock();
		start4 = clock();
		end4 = clock();
		start6 = clock();
		end6 = clock();
		Scalar color = Scalar(0, 255, 0);
		Mat drawing_g = Mat::zeros(image_binary_g.size(), CV_8UC3);
		Mat drawing_r1 = Mat::zeros(image_binary_r1.size(), CV_8UC3);
		Mat drawing_r2 = Mat::zeros(image_binary_r2.size(), CV_8UC3);
		Mat drawing_r3 = Mat::zeros(image_binary_r3.size(), CV_8UC3);
		for (int i = 0; i < contours_g.size(); i++)
		{
			drawContours(drawing_g, contours_g, i, color, 1, 8, hierarchy_g, 0, Point());
		}
		for (int i = 0; i < contours_r1.size(); i++)
		{
			drawContours(drawing_r1, contours_r1, i, color, 1, 8, hierarchy_r1, 0, Point());
		}
		for (int i = 0; i < contours_r2.size(); i++)
		{
			drawContours(drawing_r2, contours_r2, i, color, 1, 8, hierarchy_r2, 0, Point());
		}
		for (int i = 0; i < contours_r3.size(); i++)
		{
			drawContours(drawing_r3, contours_r3, i, color, 1, 8, hierarchy_r3, 0, Point());
		}
		circle(drawing_g, Point((int)mark[0][0], (int)mark[0][1]), 2, Scalar(255, 255, 255), 5);
		cv::namedWindow("binary_g", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("binary_g", image_binary_g);
		cv::namedWindow("drawing_g", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("drawing_g", drawing_g);
		cv::namedWindow("image_binary_r1", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("image_binary_r1", image_binary_r1);                // Show our image inside it.
		cv::namedWindow("drawing_r1", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("drawing_r1", drawing_r1);                // Show our image inside it.
		cv::namedWindow("binary_r2", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("binary_r2", image_binary_r2);                // Show our image inside it.
		cv::namedWindow("drawing_r2", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("drawing_r2", drawing_r2);                // Show our image inside it.
		cv::namedWindow("binary_r3", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("binary_r3", image_binary_r3);                // Show our image inside it.
		cv::namedWindow("drawing_r3", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("drawing_r3", drawing_r3);                // Show our image inside it.
	}


	else //far == 0
	{
		//resize
		start1 = clock();
		resize(image_input, image_resized, Size(2048 / RESIZE_LED, 1536 / RESIZE_LED), INTER_NEAREST);
		end1 = clock();
		cout << "reszie time: " << (double)(end1 - start1) / CLOCKS_PER_SEC << endl;
		//medianBlur(image_resized, image_resized, 3);
		//cvtcolor
		start2 = clock();
		Mat image_hsv;
		cvtColor(image_resized, image_hsv, COLOR_BGR2HSV);
		end2 = clock();
		cout << "cvtColor time: " << (double)(end2 - start2) / CLOCKS_PER_SEC << endl;


		//image_hsv = func_medianblur(image_hsv, 6);


		//inrange
		start3 = clock();
		Mat image_binary_r = func_inrange_r(image_resized, image_hsv, RESIZE_LED);
		Mat image_binary_g = func_inrange_g(image_resized, image_hsv, RESIZE_LED);
		end3 = clock();
		cout << "inrange time: " << (double)(end3 - start3) / CLOCKS_PER_SEC << endl;


		//To remove dots and To fill up dots from binary images
		start4 = clock();
		image_binary_r = func_medianblur(image_binary_r, 6);
		image_binary_g = func_medianblur(image_binary_g, 6);
		end4 = clock();
		cout << "medianbluar time: " << (double)(end4 - start4) / CLOCKS_PER_SEC << endl;


		// finde contours at bianry image
		start6 = clock();
		findContours(image_binary_g, contours_g, hierarchy_g, 0, 1, Point());
		findContours(image_binary_r, contours_r, hierarchy_r, 0, 1, Point());

		end6 = clock();
		cout << "findcontours time: " << (double)(end6 - start6) / CLOCKS_PER_SEC << endl;

		Scalar color = Scalar(0, 255, 0);
		drawing_g = Mat::zeros(image_binary_g.size(), CV_8UC3);
		drawing_r = Mat::zeros(image_binary_r.size(), CV_8UC3);
		for (int i = 0; i < contours_g.size(); i++)
		{
			drawContours(drawing_g, contours_g, i, color, 1, 8, hierarchy_g, 0, Point());
		}
		for (int i = 0; i < contours_r.size(); i++)
		{
			drawContours(drawing_r, contours_r, i, color, 1, 8, hierarchy_r, 0, Point());
		}

		cv::namedWindow("binary_g", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("binary_g", image_binary_g);
		cv::namedWindow("drawing_g", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("drawing_g", drawing_g);
		cv::namedWindow("image_binary_r", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("image_binary_r", image_binary_r);                // Show our image inside it.
		cv::namedWindow("drawing_r", WINDOW_NORMAL); // Create a window for display.
		cv::imshow("drawing_r", drawing_r);                // Show our image inside it.




	}






	// check Aspect Ratio, Area Ratio from bianry image 
	start5 = clock();
	if (far == 1)
	{
		g_cnt = 0;
		cout << "GREEN" << endl;
		func_findcandidate(contours_g, mark_temp_g, mark, g_candidate, RESIZE_LED, g_cnt, roi_dis, 0, far);
		r_cnt = 0;
		cout << "RED1" << endl;
		func_findcandidate(contours_r1, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 1, far);
		cout << "RED2" << endl;
		func_findcandidate(contours_r2, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 2, far);
		cout << "RED3" << endl;
		func_findcandidate(contours_r3, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 3, far);
	}

	else
	{
		cout << "GREEN" << endl;
		g_cnt = 0;
		func_findcandidate(contours_g, mark_temp_g, mark, g_candidate, RESIZE_LED, g_cnt, roi_dis, 0, far);
		r_cnt = 0;
		cout << "RED" << endl;
		func_findcandidate(contours_r, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 0, far);
		for (int i = 0; i < g_cnt; i++)
		{
			putText(image_input, to_string(i), Point(mark_temp_g[i][0], mark_temp_g[i][1]), FONT_HERSHEY_SIMPLEX, 1.8, Scalar(0, 0, 255), 1.8);
			circle(image_input, Point(mark_temp_g[i][0], mark_temp_g[i][1]), 4, Scalar(0, 0, 255), 3);

		}
		for (int i = 0; i < r_cnt; i++)
		{
			putText(image_input, to_string(i), Point(mark_temp_r[i][0], mark_temp_r[i][1]), FONT_HERSHEY_SIMPLEX, 1.8, Scalar(0, 255, 0), 1.8);
			circle(image_input, Point(mark_temp_r[i][0], mark_temp_r[i][1]), 4, Scalar(0, 255, 0), 3);

		}
	}

	end5 = clock();
	cout << "find cantidate: " << (double)(end5 - start5) / CLOCKS_PER_SEC << endl;
	start7 = clock();



	//FInd square from LED candidates.
	int g1, r1, r2, r3;

	for (int i = 0; i < g_cnt; i++)
	{
		for (int j = 0; j < r_cnt; j++)
		{
			vertical_cnt = 0;
			diagonal_cnt1 = 0;
			diagonal_cnt2 = 0;
			//check LED Area.  green led should between 0.5~ 2 times of RED led.
			if ((0.3* contourArea(contours_g[g_candidate[i]]) > contourArea(contours_r[r_candidate[j]]) || (2.5 * contourArea(contours_g[g_candidate[i]])) < contourArea(contours_r[r_candidate[j]])))
			{
				cout << "SIZE ERROR AT: G" << i << "R" << j << endl;
				continue;
			}
			int gr1[2] = { mark_temp_r[j][0] - mark_temp_g[i][0], mark_temp_r[j][1] - mark_temp_g[i][1] };
			gr1_size = sqrt(gr1[0] * gr1[0] + gr1[1] * gr1[1]);
			cout << "gr1_size" << "G" << i << "R" << j << ": " << gr1_size << endl;
			for (int k = j + 1; k < r_cnt; k++)
			{
				//check LED Area.  green led should between 0.5~ 2 times of RED led.
				if ((0.3* contourArea(contours_g[g_candidate[i]]) >= contourArea(contours_r[r_candidate[k]]) || (2.5 * contourArea(contours_g[g_candidate[i]])) <= contourArea(contours_r[r_candidate[j]])))
				{
					cout << "SIZE ERROR AT: G" << i << "R" << k << endl;
					continue;
				}
				int gr2[2] = { mark_temp_r[k][0] - mark_temp_g[i][0], mark_temp_r[k][1] - mark_temp_g[i][1] };
				gr2_size = sqrt(gr2[0] * gr2[0] + gr2[1] * gr2[1]);
				cout << "gr2_size" << "G" << i << "R" << k << ": " << gr2_size << endl;
				double radian = acos((gr1[0] * gr2[0] + gr1[1] * gr2[1]) / (gr2_size*gr1_size));
				radian = radian * (180 / PI);
				cout << "radian" << "G" << i << "R" << j << "R" << k << ": " << radian << endl;
				// check angle of vector gr1 and vector gr2. 
				// check length of vetor gr1 and vector gr2. 
				// determine they are LED of Squre or NOT.
				if (90 - error_radian < radian && radian < 90 + error_radian)
				{
					if ((1 - error_distance)*gr1_size < gr2_size && gr2_size < (1 + error_distance)*gr1_size)
					{
						r_vertical = k;
						vertical_cnt = vertical_cnt + 1;
						cout << "vertical" << endl;
					}
				}
				else if (45 - error_radian < radian && radian < 45 + error_radian)
				{
					if (sqrt(2)*(1 - error_distance)*gr1_size < gr2_size && gr2_size < sqrt(2)*(1 + error_distance)*gr1_size)
					{
						r_diagonal_1 = k;
						diagonal_cnt1 = diagonal_cnt1 + 1;
						cout << "diagonal1" << endl;

					}

					else if ((1 - error_distance)*gr1_size / sqrt(2) < gr2_size && gr2_size < (1 + error_distance)*gr1_size / sqrt(2))
					{
						r_diagonal_2[diagonal_cnt2] = k;
						diagonal_cnt2 = diagonal_cnt2 + 1;
						cout << "diagonal2" << endl;
					}
				}

				if (diagonal_cnt2 == 2)
				{
					cout << "square found2" << endl << endl;
					break;
				}

				if ((vertical_cnt >= 1 && diagonal_cnt1 >= 1))
				{
					//check r_vertial and r_diagonal1 is same direction about gr1.
					if (signextract(((mark_temp_r[j][0] - mark_temp_g[i][0])*(mark_temp_r[r_diagonal_1][1] - mark_temp_g[i][1]) - (mark_temp_r[j][1] - mark_temp_g[i][1])*(mark_temp_r[r_diagonal_1][0] - mark_temp_g[i][0]))) * signextract(((mark_temp_r[j][0] - mark_temp_g[i][0])*(mark_temp_r[r_vertical][1] - mark_temp_g[i][1]) - (mark_temp_r[j][1] - mark_temp_g[i][1])*(mark_temp_r[r_vertical][0] - mark_temp_g[i][0]))) > 0)
					{
						cout << "square found1" << endl << endl;
						break;
					}
				}
			}

			if (vertical_cnt >= 1 && diagonal_cnt1 >= 1)
			{
				g1 = i;
				r1 = r_vertical;
				r2 = r_diagonal_1;
				r3 = j;
				mark[0][0] = mark_temp_g[g1][0];
				mark[0][1] = mark_temp_g[g1][1];
				mark[1][0] = mark_temp_r[r1][0];
				mark[1][1] = mark_temp_r[r1][1];
				mark[2][0] = mark_temp_r[r2][0];
				mark[2][1] = mark_temp_r[r2][1];
				mark[3][0] = mark_temp_r[r3][0];
				mark[3][1] = mark_temp_r[r3][1];
				detected = 1;
			}

			else if (diagonal_cnt2 == 2)
			{
				g1 = i;
				r1 = r_diagonal_2[0];
				r2 = j;
				r3 = r_diagonal_2[1];
				mark[0][0] = mark_temp_g[g1][0];
				mark[0][1] = mark_temp_g[g1][1];
				mark[1][0] = mark_temp_r[r1][0];
				mark[1][1] = mark_temp_r[r1][1];
				mark[2][0] = mark_temp_r[r2][0];
				mark[2][1] = mark_temp_r[r2][1];
				mark[3][0] = mark_temp_r[r3][0];
				mark[3][1] = mark_temp_r[r3][1];
				detected = 1;

				// to prevent mark1 and mark3 locate same direction about mark2.
				// if product of cross products is positive, they are same direction.  
				mark_center[0] = (mark[0][0] + mark[1][0] + mark[2][0] + mark[3][0]) / 4;
				mark_center[1] = (mark[0][1] + mark[1][1] + mark[2][1] + mark[3][1]) / 4;
				if (signextract(((mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]))) * signextract(((mark[2][0] - mark[0][0])*(mark[1][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[1][0] - mark[0][0]))) > 0)
				{
					cout << "MARK1 AND MARK3 ERROR: " << "They are same direction now." << endl;
					detected = 0;
				}

			}


			//check length of side of squre and average LED diameter. (using non-change distance 3m(between led) and 0.2m(led diameter))
			if (detected == 1)
			{
				double AREA = contourArea(contours_g[g_candidate[g1]]) + contourArea(contours_r[r_candidate[r1]]) + contourArea(contours_r[r_candidate[r2]]) + contourArea(contours_r[r_candidate[r3]]);
				//cout << g1 << " " << r1 << " " << r2 << " " << r3 << endl;
				//cout << contourArea(contours_g[g_candidate[g1]]) << " " << contourArea(contours_r[r_candidate[r1]]) << " " << contourArea(contours_r[r_candidate[r2]]) << " " << contourArea(contours_r[r_candidate[r3]]) << endl;

				double DISTANCE_RATIO_CAL = 88 * pow(AREA, -0.72);
				double DISTANCE_RATIO_REAL = max(gr1_size, gr2_size) / AREA;
				cout << "DISTANCE_RATIO_CAL: " << DISTANCE_RATIO_CAL << endl;
				cout << "DISTANCE RATIO_REAL: " << DISTANCE_RATIO_REAL << endl;
				cout << "AREA: " << (contourArea(contours_g[g1]) + contourArea(contours_r[r1]) + contourArea(contours_r[r2]) + contourArea(contours_r[r3])) << endl;

				// just test. delete it//
				cout << "real/cal: " << max(DISTANCE_RATIO_CAL, DISTANCE_RATIO_REAL) / min(DISTANCE_RATIO_CAL, DISTANCE_RATIO_REAL) << endl;
				// from here
				if (DISTANCE_RATIO_REAL > DISTANCE_RATIO_CAL * 2 || DISTANCE_RATIO_REAL < DISTANCE_RATIO_CAL*0.4)
				{
					cout << "DISTANCE_RATIO ERROR: rejected." << endl;
					detected = 0;
				}
			}

			// check numbering is counterclockwise. if not, change 1 and 3.

			if ((mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]) > 0)
			{
				cout << (mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) << endl;
				cout << (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]) << endl;
				cout << (mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]) << endl;
				int temp[2] = { mark[1][0],mark[1][1] };
				mark[1][0] = mark[3][0];
				mark[1][1] = mark[3][1];
				mark[3][0] = temp[0];
				mark[3][1] = temp[1];
			}

			if (detected == 1)
			{
				for (int i = 0; i < 4; i++)
				{
					std::cout << "LED" << i << "x,y :" << mark[i][0] << ", " << mark[i][1] << endl;
					mark_flag[i] = true;
					putText(image_input, to_string(i), Point(mark[i][0], mark[i][1]), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 0), 2);
					circle(image_input, Point(mark[i][0], mark[i][1]), 4, Scalar(255, 0, 0), 3);
				}
				break;
			}
		}
		if (detected == 1)
		{
			//far = checkfar(mark_flag, max(gr1_size, gr2_size));
			break;
		}
	}
	if (detected != 1)
		std::cout << "square detect fail. start finding triangle..." << endl;
	end7 = clock();
	cout << "findsquare time: " << (double)(end7 - start7) / CLOCKS_PER_SEC << endl;
	////
	//// triangle start
	////
	if (detected != 1)
	{
		r_vertical = 0;
		r_diagonal_1 = 0;
		r_diagoanl_2_tri = 0;
		for (int i = 0; i < g_cnt; i++)
		{
			for (int j = 0; j < r_cnt; j++)
			{
				vertical_cnt = 0;
				diagonal_cnt1 = 0;
				diagonal_cnt2 = 0;
				//check LED Area.  green led should between 0.5~ 2 times of RED led.
				if ((0.3* contourArea(contours_g[g_candidate[i]]) > contourArea(contours_r[r_candidate[j]]) || (2.5 * contourArea(contours_g[g_candidate[i]])) < contourArea(contours_r[r_candidate[j]])))
				{
					cout << "SIZE ERROR AT: G" << i << "R" << j << endl;
					continue;
				}
				int gr1[2] = { mark_temp_r[j][0] - mark_temp_g[i][0], mark_temp_r[j][1] - mark_temp_g[i][1] };
				gr1_size = sqrt(gr1[0] * gr1[0] + gr1[1] * gr1[1]);
				cout << "gr1_size" << "G" << i << "R" << j << ": " << gr1_size << endl;
				for (int k = j + 1; k < r_cnt; k++)
				{
					//check LED Area.  green led should between 0.5~ 2 times of RED led.
					if ((0.3* contourArea(contours_g[g_candidate[i]]) >= contourArea(contours_r[r_candidate[k]]) || (2.5 * contourArea(contours_g[g_candidate[i]])) <= contourArea(contours_r[r_candidate[j]])))
					{
						cout << "SIZE ERROR AT: G" << i << "R" << k << endl;
						continue;
					}
					int gr2[2] = { mark_temp_r[k][0] - mark_temp_g[i][0], mark_temp_r[k][1] - mark_temp_g[i][1] };
					gr2_size = sqrt(gr2[0] * gr2[0] + gr2[1] * gr2[1]);
					cout << "gr2_size" << "G" << i << "R" << k << ": " << gr2_size << endl;
					double radian = acos((gr1[0] * gr2[0] + gr1[1] * gr2[1]) / (gr2_size*gr1_size));
					radian = radian * (180 / PI);
					cout << "radian" << "G" << i << "R" << j << "R" << k << ": " << radian << endl;
					// check angle of vector gr1 and vector gr2. 
					// check length of vetor gr1 and vector gr2. 
					// determine they are LED of Squre or NOT.
					if (90 - error_radian < radian && radian < 90 + error_radian)
					{
						if ((1 - error_distance)*gr1_size < gr2_size && gr2_size < (1 + error_distance)*gr1_size)
						{
							r_vertical = k;
							vertical_cnt = vertical_cnt + 1;
							cout << "vertical" << endl;
						}
					}
					else if (45 - error_radian < radian && radian < 45 + error_radian)
					{
						if (sqrt(2)*(1 - error_distance)*gr1_size < gr2_size && gr2_size < sqrt(2)*(1 + error_distance)*gr1_size)
						{
							r_diagonal_1 = k;
							diagonal_cnt1 = diagonal_cnt1 + 1;
							cout << "diagonal1" << endl;

						}

						else if ((1 - error_distance)*gr1_size / sqrt(2) < gr2_size && gr2_size < (1 + error_distance)*gr1_size / sqrt(2))
						{
							r_diagoanl_2_tri = k;
							diagonal_cnt2 = diagonal_cnt2 + 1;
							cout << "diagonal2" << endl;
						}
					}

					if (vertical_cnt == 1)
					{
						cout << "ver_triangle found" << endl << endl;
						break;
					}

					if (diagonal_cnt1 == 1)
					{
						cout << "dia1_triangle found" << endl << endl;
						break;
					}

					if (diagonal_cnt2 == 1)
					{
						cout << "dia2_triangle found" << endl << endl;
						break;
					}
				}

				if (vertical_cnt == 1)
				{
					cout << "in ver==1" << endl;
					g1 = i;
					r1 = r_vertical;
					r3 = j;

					mark[0][0] = mark_temp_g[g1][0];
					mark[0][1] = mark_temp_g[g1][1];
					mark[1][0] = mark_temp_r[r1][0];
					mark[1][1] = mark_temp_r[r1][1];
					mark[3][0] = mark_temp_r[r3][0];
					mark[3][1] = mark_temp_r[r3][1];
					axialsymmetry(mark[1], mark[3], mark[0], mark[2]);

					cv::line(image_input, Point(mark[0][0], mark[0][1]), Point(mark[1][0], mark[1][1]), Scalar(0, 0, 0), 2, LINE_8);
					cv::line(image_input, Point(mark[1][0], mark[1][1]), Point(mark[3][0], mark[3][1]), Scalar(0, 0, 0), 2, LINE_8);
					cv::line(image_input, Point(mark[3][0], mark[3][1]), Point(mark[0][0], mark[0][1]), Scalar(0, 0, 0), 2, LINE_8);
					detected = 1;
				}

				else if (diagonal_cnt1 == 1)
				{
					cout << "in dia1==1" << endl;
					g1 = i;
					r2 = r_diagonal_1;
					r1 = j;


					mark[0][0] = mark_temp_g[g1][0];
					mark[0][1] = mark_temp_g[g1][1];
					mark[2][0] = mark_temp_r[r2][0];
					mark[2][1] = mark_temp_r[r2][1];
					mark[1][0] = mark_temp_r[r1][0];
					mark[1][1] = mark_temp_r[r1][1];
					axialsymmetry(mark[0], mark[2], mark[1], mark[3]);

					cv::line(image_input, Point(mark[0][0], mark[0][1]), Point(mark[1][0], mark[1][1]), Scalar(0, 0, 0), 2, LINE_8);
					cv::line(image_input, Point(mark[1][0], mark[1][1]), Point(mark[2][0], mark[2][1]), Scalar(0, 0, 0), 2, LINE_8);
					cv::line(image_input, Point(mark[2][0], mark[2][1]), Point(mark[0][0], mark[0][1]), Scalar(0, 0, 0), 2, LINE_8);

					Rect roi_t(mark[3][0] - roi_dis, mark[3][1] - roi_dis, 2 * roi_dis, 2 * roi_dis);
					Mat roi_t_image = image_input(roi_t);
					Mat roi_t_image_gray;
					cv::cvtColor(roi_t_image, roi_t_image_gray, COLOR_BGR2GRAY);
					Mat gra_x, gra_y, gra_xy;
					Sobel(roi_t_image_gray, gra_x, CV_64F, 1, 0);
					Sobel(roi_t_image_gray, gra_y, CV_64F, 0, 1);
					convertScaleAbs(gra_x, gra_x, 1, 0);
					convertScaleAbs(gra_y, gra_y, 1, 0);
					gra_xy = gra_x + gra_y;


					Mat bgr[3];
					cv::split(roi_t_image, bgr);

					Mat added_roi;
					addWeighted(gra_xy, 0.5, bgr[2], 0.5, 0, added_roi);

					Mat thres_roi;
					threshold(added_roi, thres_roi, 120, 255, THRESH_BINARY);
					//morphologyEx(thres_roi, thres_roi, MORPH_CLOSE, 3);

					namedWindow("gra_xy", WINDOW_NORMAL); // Create a window for display.
					imshow("gra_xy", gra_xy);
					namedWindow(" bgr[2]", WINDOW_NORMAL); // Create a window for display.
					imshow(" bgr[2]", bgr[2]);
					namedWindow("added_roi", WINDOW_NORMAL); // Create a window for display.
					imshow("added_roi", added_roi);
					namedWindow("thres_roi", WINDOW_NORMAL); // Create a window for display.
					imshow("thres_roi", thres_roi);

					detected = 1;

				}

				else if (diagonal_cnt2 == 1)
				{
					cout << "in dia2==1" << endl;
					g1 = i;
					r1 = r_diagoanl_2_tri;
					r2 = j;

					mark[0][0] = mark_temp_g[g1][0];
					mark[0][1] = mark_temp_g[g1][1];
					mark[1][0] = mark_temp_r[r1][0];
					mark[1][1] = mark_temp_r[r1][1];
					mark[2][0] = mark_temp_r[r2][0];
					mark[2][1] = mark_temp_r[r2][1];
					axialsymmetry(mark[0], mark[2], mark[1], mark[3]);
					cv::line(image_input, Point(mark[0][0], mark[0][1]), Point(mark[1][0], mark[1][1]), Scalar(0, 0, 0), 2, LINE_8);
					cv::line(image_input, Point(mark[1][0], mark[1][1]), Point(mark[2][0], mark[2][1]), Scalar(0, 0, 0), 2, LINE_8);
					cv::line(image_input, Point(mark[2][0], mark[2][1]), Point(mark[0][0], mark[0][1]), Scalar(0, 0, 0), 2, LINE_8);
					detected = 1;
				}

				if (detected == 1)
				{
					// to prevent mark1 and mark3 locate same direction about mark2.
					// if product of cross products is positive, they are same direction.  
					mark_center[0] = (mark[0][0] + mark[1][0] + mark[2][0] + mark[3][0]) / 4;
					mark_center[1] = (mark[0][1] + mark[1][1] + mark[2][1] + mark[3][1]) / 4;
					if (signextract(((mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]))) * signextract(((mark[2][0] - mark[0][0])*(mark[1][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[1][0] - mark[0][0]))) > 0)
					{
						cout << "MARK1 AND MARK3 ERROR: " << "They are same direction now." << endl;
						detected = 0;
					}

					//check length of side of squre and average LED diameter. (using non-change distance 3m(between led) and 0.2m(led diameter))
					double AREA = (contourArea(contours_g[g_candidate[g1]]) + contourArea(contours_r[r_candidate[r1]])) * 2;
					//cout << g1 << " " << r1 << " " << r2 << " " << r3 << endl;
					//cout << contourArea(contours_g[g_candidate[g1]]) << " " << contourArea(contours_r[r_candidate[r1]]) << " " << contourArea(contours_r[r_candidate[r2]]) << " " << contourArea(contours_r[r_candidate[r3]]) << endl;

					double DISTANCE_RATIO_CAL = 88 * pow(AREA, -0.72);
					double DISTANCE_RATIO_REAL = max(gr1_size, gr2_size) / AREA;
					cout << "DISTANCE_RATIO_CAL: " << DISTANCE_RATIO_CAL << endl;
					cout << "DISTANCE RATIO_REAL: " << DISTANCE_RATIO_REAL << endl;
					cout << "AREA: " << AREA << endl;

					// just test. delete it//
					cout << "real/cal: " << max(DISTANCE_RATIO_CAL, DISTANCE_RATIO_REAL) / min(DISTANCE_RATIO_CAL, DISTANCE_RATIO_REAL) << endl;
					// from here

					if (DISTANCE_RATIO_REAL > DISTANCE_RATIO_CAL * 2 || DISTANCE_RATIO_REAL < DISTANCE_RATIO_CAL*0.4)
					{
						cout << "DISTANCE_RATIO ERROR: rejected." << endl;
						detected = 0;
					}
				}

				// check numbering is counterclockwise. if not, change 1 and 3.
				if (detected == 1)
				{
					if ((mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]) > 0)
					{
						//cout << (mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) << endl;
						//cout << (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]) << endl;
						//cout << (mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]) << endl;
						int temp[2] = { mark[1][0],mark[1][1] };
						mark[1][0] = mark[3][0];
						mark[1][1] = mark[3][1];
						mark[3][0] = temp[0];
						mark[3][1] = temp[1];
					}
				}

				if (detected == 1)
				{
					for (int i = 0; i < 4; i++)
					{
						std::cout << "LED" << i << "x,y :" << mark[i][0] << ", " << mark[i][1] << endl;
						mark_flag[i] = true;
						putText(image_input, to_string(i), Point(mark[i][0], mark[i][1]), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 0), 2);
						circle(image_input, Point(mark[i][0], mark[i][1]), 4, Scalar(255, 0, 0), 3);
					}
					break;
				}
			}

			if (detected == 1)
			{
				//far = checkfar(mark_flag, max(gr1_size, gr2_size));
				break;
			}
		}
	}
	////
	//// triangle end
	////
	//cout << "Net time: " << (double)(end1 + end2 + end3 + end4 + end5 + end6 + end7 - start1 - start2 - start3 - start4 - start5 - start6 - start7) / CLOCKS_PER_SEC << endl;
	cv::namedWindow("image_input", WINDOW_NORMAL); // Create a window for display.
	cv::imshow("image_input", image_input);      // Show our image inside it.
	cv::waitKey(0); // Wait for a keystroke in the window


	return 0;
}


void func_findcandidate(vector<vector<Point>>contours, int(*mark_temp)[2], int(*mark)[2], int candidate[], int RESIZE_LED, int& cnt, int roi_dis, int lednum, int far)
{
	for (int i = 0; i < contours.size(); i++) {
		vector<Rect> boundRect(contours.size());
		boundRect[i] = boundingRect(Mat(contours[i]));
		vector<Moments> mu(contours.size());
		double AspectRatio = (double)boundRect[i].height / boundRect[i].width;
		double AreaRatio = (double)contourArea(contours[i]) / (boundRect[i].height*boundRect[i].width);
		std::cout << "   cnadidate" << i << "     " << "      ,             " << boundRect[i].height << "        " << boundRect[i].width << "        " << contourArea(contours[i]) << "        " << AspectRatio << "             " << AreaRatio << endl;
		if (AspectRatio < 1.65 && AspectRatio> 0.35) {
			if (AreaRatio > 0.3 && AreaRatio < 0.8) {
				mu[i] = moments(contours[i], false);
				double x = mu[i].m10 / mu[i].m00 + 1;
				double y = mu[i].m01 / mu[i].m00 + 1;
				if (RESIZE_LED == 1 && far == 1)
				{
					mark_temp[cnt][0] = (int)(mark[lednum][0] - roi_dis + x);
					mark_temp[cnt][1] = (int)(mark[lednum][1] - roi_dis + y);
				}
				else
				{
					mark_temp[cnt][0] = (int)(x * RESIZE_LED);
					mark_temp[cnt][1] = (int)(y * RESIZE_LED);
				}
				cout << "  CANDIDATE" << cnt << "     " << x << " ," << y << "     " << boundRect[i].height << "        " << boundRect[i].width << "        " << contourArea(contours[i]) << "        " << AspectRatio << "             " << AreaRatio << endl;
				candidate[cnt] = i;
				cnt = cnt + 1;
			}
		}
	}
}
