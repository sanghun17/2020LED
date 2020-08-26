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
	inRange(src_hsv, Scalar(0, 0, 200), Scalar(60, 255, 255), dst);
	inRange(src_hsv, Scalar(145, 0, 170), Scalar(179, 255, 255), dst_temp);

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


void func_findcandidate(vector<vector<Point>>contours, int(*mark_temp)[2], int(*mark)[2], int candidate[], int RESIZE_LED, int& cnt, int roi_dis, int lednum);



int main(int argc, char** argv) {

	double error_aspectratio = 0.65;
	double error_arearatio = 0.31;
	double error_radian = 4.5;
	double error_distance = 0.1;

	//int mark[4][2] = { 0 };
	int mark[4][2] = { {1126, 390}, {878, 382}, {874 , 621}, {1124, 625} };
	bool far =1 ;
	int RESIZE_LED;
	if (far == 1)
		RESIZE_LED = 1;
	else
		RESIZE_LED = 4;

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
	double gr1_size, gr2_size;

	int roi_dis = 80;

	Mat drawing_g, drawing_r;

	clock_t start1, start2, start3, start4, start5, start6, start7, end1, end2, end3, end4, end5, end6, end7;
	string imageName = "image13.bmp";

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
		cout << "reszie time: " << end1 - start1 << endl;

		//cvtcolor
		start2 = clock();
		Mat image_hsv;
		cvtColor(image_resized, image_hsv, COLOR_BGR2HSV);
		end2 = clock();
		cout << "cvtColor time: " << end2 - start2 << endl;

		//inrange
		start3 = clock();
		Mat image_binary_r = func_inrange_r(image_resized, image_hsv, RESIZE_LED);
		Mat image_binary_g = func_inrange_g(image_resized, image_hsv, RESIZE_LED);
		end3 = clock();
		cout << "inrange time: " << end3 - start3 << endl;


		//To remove dots and To fill up dots from binary images
		start4 = clock();
		image_binary_r = func_medianblur(image_binary_r, 6);
		image_binary_g = func_medianblur(image_binary_g, 6);
		end4 = clock();
		cout << "medianbluar time: " << end4 - start4 << endl;


		// finde contours at bianry image
		start6 = clock();
		findContours(image_binary_g, contours_g, hierarchy_g, 0, 1, Point());
		findContours(image_binary_r, contours_r, hierarchy_r, 0, 1, Point());

		end6 = clock();
		cout << "findcontours time: " << end6 - start6 << endl;

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


	start5 = clock();
	end5 = clock();
	cout << "morphology time: " << end5 - start5 << endl;



	// check Aspect Ratio, Area Ratio from bianry image 

	if (far == 1)
	{
		g_cnt = 0;
		cout << "GREEN" << endl;
		func_findcandidate(contours_g, mark_temp_g, mark, g_candidate, RESIZE_LED, g_cnt, roi_dis, 0);
		r_cnt = 0;
		cout << "RED1" << endl;
		func_findcandidate(contours_r1, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 1);
		cout << "RED2" << endl;
		func_findcandidate(contours_r2, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 2);
		cout << "RED3" << endl;
		func_findcandidate(contours_r3, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 3);
	}

	else
	{
		cout << "GREEN" << endl;
		g_cnt = 0;
		func_findcandidate(contours_g, mark_temp_g, mark, g_candidate, RESIZE_LED, g_cnt, roi_dis, 0);
		r_cnt = 0;
		cout << "RED" << endl;
		func_findcandidate(contours_r, mark_temp_r, mark, r_candidate, RESIZE_LED, r_cnt, roi_dis, 0);
		for (int i = 0; i < g_cnt; i++)
		{
			putText(image_input, to_string(i), Point(mark_temp_g[i][0], mark_temp_g[i][1]), FONT_HERSHEY_SIMPLEX, 1.8, Scalar(0, 255, 0), 1.8);
			circle(image_input, Point(mark_temp_g[i][0], mark_temp_g[i][1]), 4, Scalar(0, 255, 0), 3);
			//circle(drawing_g, Point(mark_temp_g[i][0] / RESIZE_LED, mark_temp_g[i][1] / RESIZE_LED), 0, Scalar(255,255 ,255), 1);
		}
		for (int i = 0; i < r_cnt; i++)
		{
			putText(image_input, to_string(i), Point(mark_temp_r[i][0], mark_temp_r[i][1]), FONT_HERSHEY_SIMPLEX, 1.8, Scalar(0, 0, 255), 1.8);
			circle(image_input, Point(mark_temp_r[i][0], mark_temp_r[i][1]), 4, Scalar(0, 0, 255), 3);
			//circle(drawing_r, Point(mark_temp_r[i][0] / RESIZE_LED, mark_temp_r[i][1] / RESIZE_LED), 0, Scalar(255, 255, 255), 1);
		}
	}
	start7 = clock();



	//FInd square from LED candidates.
	int g1 = 0;
	int r1 = 0;
	int r2 = 0;
	int  r3 = 0;

	for (int i = 0; i < g_cnt; i++) {
		for (int j = 0; j < r_cnt; j++) {
			//check LED Area.  green led should between 0.5~ 2 times of RED led.
			if ((0.3* contourArea(contours_g[g_candidate[i]]) > contourArea(contours_r[r_candidate[j]]) || (2.5 * contourArea(contours_g[g_candidate[i]])) < contourArea(contours_r[r_candidate[j]]))) {
				cout << "SIZE ERROR AT: G" << i << "R" << j << endl;
				continue;
			}
			int gr1[2] = { mark_temp_r[j][0] - mark_temp_g[i][0], mark_temp_r[j][1] - mark_temp_g[i][1] };
			gr1_size = sqrt(gr1[0] * gr1[0] + gr1[1] * gr1[1]);
			cout << "gr1_size" << i << j << ": " << gr1_size << endl;;
			for (int k = j + 1; k < r_cnt; k++) {
				//check LED Area.  green led should between 0.5~ 2 times of RED led.
				if ((0.3* contourArea(contours_g[g_candidate[i]]) >= contourArea(contours_r[r_candidate[k]]) || (2.5 * contourArea(contours_g[g_candidate[i]])) <= contourArea(contours_r[r_candidate[j]]))) {
					cout << "SIZE ERROR AT: G" << i << "R" << k << endl;
					continue;
				}
				int gr2[2] = { mark_temp_r[k][0] - mark_temp_g[i][0], mark_temp_r[k][1] - mark_temp_g[i][1] };
				gr2_size = sqrt(gr2[0] * gr2[0] + gr2[1] * gr2[1]);
				cout << "gr2_size" << i << k << ": " << gr2_size << endl;;
				double radian = acos((gr1[0] * gr2[0] + gr1[1] * gr2[1]) / (gr2_size*gr1_size));
				radian = radian * (180 / PI);
				//cout << "radian" << i << j <<k<< ": " << radian << endl;;
				// check angle of vector gr1 and vector gr2. 
				// check length of vetor gr1 and vector gr2. 
				// determine they are LED of Squre or NOT.
				if (90 - error_radian < radian && radian < 90 + error_radian) {
					if ((1 - error_distance)*gr1_size < gr2_size && gr2_size < (1 + error_distance)*gr1_size) {
						r_vertical = k;
						vertical_cnt = vertical_cnt + 1;
						cout << "vertical" << endl;
					}
				}
				else if (45 - error_radian < radian && radian < 45 + error_radian) {
					if (sqrt(2)*(1 - error_distance)*gr1_size < gr2_size && gr2_size < sqrt(2)*(1 + error_distance)*gr1_size) {
						r_diagonal_1 = k;
						diagonal_cnt1 = diagonal_cnt1 + 1;
						cout << "diagonal1" << endl;

					}

					else if ((1 - error_distance)*gr1_size / sqrt(2) < gr2_size && gr2_size < (1 + error_distance)*gr1_size / sqrt(2)) {
						r_diagonal_2[diagonal_cnt2] = k;
						diagonal_cnt2 = diagonal_cnt2 + 1;
						cout << "diagonal2" << endl;
					}
				}
			}

			if (vertical_cnt == 1 && diagonal_cnt1 == 1) {
				mark[0][0] = mark_temp_g[i][0];
				mark[0][1] = mark_temp_g[i][1];
				mark[1][0] = mark_temp_r[r_vertical][0];
				mark[1][1] = mark_temp_r[r_vertical][1];
				mark[2][0] = mark_temp_r[r_diagonal_1][0];
				mark[2][1] = mark_temp_r[r_diagonal_1][1];
				mark[3][0] = mark_temp_r[j][0];
				mark[3][1] = mark_temp_r[j][1];
				detected = 1;
			}

			else if (diagonal_cnt2 == 2) {
				mark[0][0] = mark_temp_g[i][0];
				mark[0][1] = mark_temp_g[i][1];
				mark[1][0] = mark_temp_r[r_diagonal_2[0]][0];
				mark[1][1] = mark_temp_r[r_diagonal_2[0]][1];
				mark[2][0] = mark_temp_r[j][0];
				mark[2][1] = mark_temp_r[j][1];
				mark[3][0] = mark_temp_r[r_diagonal_2[1]][0];
				mark[3][1] = mark_temp_r[r_diagonal_2[1]][1];
				detected = 1;

				// to prevent mark1 and mark3 locate same direction about mark2.
				// if product of cross products is positive, they are same direction.  
				mark_center[0] = (mark[0][0] + mark[1][0] + mark[2][0] + mark[3][0]) / 4;
				mark_center[1] = (mark[0][1] + mark[1][1] + mark[2][1] + mark[3][1]) / 4;
				if (((mark_center[0] - mark[0][0])*(mark[0][1] - mark[3][1]) - (mark[0][1] - mark_center[1])*(mark[3][0] - mark[0][0]))*((mark_center[0] - mark[0][0])*(mark[0][1] - mark[1][1]) - (mark[0][1] - mark_center[1])*(mark[1][0] - mark[0][0])) > 0)
					detected = 0;
				cout << "MARK1 AND MARK3 ERROR: " << "They are same direction now." << endl;
			}


			//check length of side of squre and average LED diameter. (using non-change distance 3m(between led) and 0.2m(led diameter))
			if (detected == 1) {
				double AREA = contourArea(contours_g[g1]) + contourArea(contours_r[r1]) + contourArea(contours_r[r2]) + contourArea(contours_r[r3]);
				double DISTANCE_RATIO_CAL = 165 * pow(AREA, -0.873);
				double DISTANCE_RATIO = max(gr1_size, gr2_size) / AREA;
				cout << "DISTANCE_RATIO_CAL: " << DISTANCE_RATIO_CAL << endl;
				cout << "DISTANCE RATIO_REAL: " << DISTANCE_RATIO << endl;
				cout << "AREA: " << (contourArea(contours_g[g1]) + contourArea(contours_r[r1]) + contourArea(contours_r[r2]) + contourArea(contours_r[r3])) << endl;
				if (DISTANCE_RATIO > DISTANCE_RATIO_CAL * 2 || DISTANCE_RATIO < DISTANCE_RATIO_CAL*0.4) {
					cout << "DISTANCE_RATIO ERROR: rejected." << endl;
					detected = 0;
				}
			}

			// check numbering is counterclockwise. if not, change 1 and 3.
			if (((mark_center[0] - mark[0][0])*(mark[0][1] - mark[3][1]) - (mark[0][1] - mark_center[1])*(mark[3][0] - mark[0][0])) < 0) {
				int temp[2] = { mark[1][0],mark[1][1] };
				mark[1][0] = mark[3][0];
				mark[1][1] = mark[3][1];
				mark[3][0] = temp[0];
				mark[3][1] = temp[1];
			}
			if (detected == 1) {
				for (int i = 0; i < 4; i++) {
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
			far = checkfar(mark_flag, max(gr1_size, gr2_size));
			break;
		}
	}
	if (detected != 1)
		std::cout << "detect fail" << endl;
	end7 = clock();
	cout << "findsquare time: " << end7 - start7 << endl;
	cout << "Net time: " << end1 + end2 + end3 + end4 + end5 + end6 + end7 - start1 - start2 - start3 - start4 - start5 - start6 - start7 << endl;
	cv::namedWindow("image_input", WINDOW_NORMAL); // Create a window for display.
	cv::imshow("image_input", image_input);      // Show our image inside it.
	cv::waitKey(0); // Wait for a keystroke in the window


	return 0;
}


void func_findcandidate(vector<vector<Point>>contours, int(*mark_temp)[2], int(*mark)[2], int candidate[], int RESIZE_LED, int& cnt, int roi_dis, int lednum)
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
				if (RESIZE_LED == 1)
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
