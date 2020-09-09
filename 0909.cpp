int signextract(int a)
{
	if (a >= 0)
		return 1;
	if (a < 0)
		return -1;
}
void led_extraction(int mark[4][2], Mat& _mat_original, bool mark_flag[4]){
	double AspectRatio_g, AspectRatio_r, AreaRatio_g, AreaRatio_r;
	double error_aspectratio = 0.65;
	double error_arearatio = 0.31;
	double error_radian = 4.5;
	double error_distance = 0.1;
	double mark_temp_r[100][2] = { 0 };
	double mark_temp_g[100][2] = { 0 };
	int g_candidate[100] = { 0 };
	int r_candidate[100] = { 0 };
	int detected = 0;
	Mat image_resized, image_hsv;
	int g_cnt, r_cnt;
	double mark_center[2];
	int vertical_cnt = 0;
	int diagonal_cnt1 = 0;
	int diagonal_cnt2 = 0;
	int r_vertical = 0;
	int r_diagonal_1 = 0;
	int r_diagonal_2[2] = { 0 };
	double gr1_size, gr2_size;
	resize(_mat_original, image_resized, Size(WIDTH / RESIZE_LED, HEIGHT / RESIZE_LED), INTER_NEAREST);
	medianBlur(image_resized, image_resized, 3);
	cvtColor(image_resized, image_hsv, COLOR_BGR2HSV);
	Mat image_binary_r, image_binary_r2, image_binary_g;
	//Generate R binary
	inRange(image_hsv, Scalar(0, 0, 60), Scalar(60, 255, 255), image_binary_r);
	inRange(image_hsv, Scalar(145, 20, 170), Scalar(179, 255, 255), image_binary_r2);
	//Ganerate G binary
	inRange(image_hsv, Scalar(61, 50, 80), Scalar(200, 255, 255), image_binary_g);
	uchar* image_data = (uchar*)image_resized.data;
	uchar* r2_data = (uchar*)image_binary_r2.data;
	uchar* r_data = (uchar*)image_binary_r.data;
	uchar* g_data = (uchar*)image_binary_g.data;
	int length = (WIDTH / RESIZE_LED) * (HEIGHT / RESIZE_LED);
	for (int i = 0; i < length; i++) {
		if (r2_data[i] == 255)
			r_data[i] = 255;
		if (image_data[3 * i + 2] < (image_data[3 * i] + image_data[3 * i + 1]) / 2 + 10)
			// If R is small
			r_data[i] = 0;
		if (image_data[3 * i + 1] < (image_data[3 * i + 2] + image_data[3 * i]) / 2 + 20)
			// If G is small
			g_data[i] = 0;
	}
	//To remove dots and To fill up dots from binary images
	medianBlur(image_binary_r, image_binary_r, 3);
	medianBlur(image_binary_r, image_binary_r, 3);
	medianBlur(image_binary_r, image_binary_r, 3);
	medianBlur(image_binary_g, image_binary_g, 3);
	medianBlur(image_binary_g, image_binary_g, 3);
	medianBlur(image_binary_g, image_binary_g, 3);
	// finde contours at bianry image
	vector<vector<Point> > contours_r, contours_g;
	vector<Vec4i> hierarchy_r, hierarchy_g;
	findContours(image_binary_g, contours_g, hierarchy_g, 0, 1, Point());
	findContours(image_binary_r, contours_r, hierarchy_r, 0, 1, Point());
	vector<Rect> boundRect_g(contours_g.size()), boundRect_r(contours_r.size());
	vector<Moments> mu_g(contours_g.size()), mu_r(contours_r.size());
	//Scalar color = Scalar(0, 255, 0);
	//Mat drawing_g = Mat::zeros(image_binary_g.size(), CV_8UC3);
	//Mat drawing_r = Mat::zeros(image_binary_r.size(), CV_8UC3);
	//for (int i = 0; i < contours_g.size(); i++)
	//{
	//	drawContours(drawing_g, contours_g, i, color, 1, 8, hierarchy_g, 0, Point());
	//}
	//for (int i = 0; i < contours_r.size(); i++)
	//{
	//	drawContours(drawing_r, contours_r, i, color, 1, 8, hierarchy_r, 0, Point());
	//}
	//cv::namedWindow("drawing_r", WINDOW_NORMAL); // Create a window for display.
	//cv::imshow("drawing_r", drawing_r);                // Show our image inside it.
	//cv::namedWindow("drawing_g", WINDOW_NORMAL); // Create a window for display.
	//cv::imshow("drawing_g", drawing_g);
	// check Aspect Ratio, Area Ratio from bianry image
	g_cnt = 0;
	for (int i = 0; i < contours_g.size(); i++) {
		boundRect_g[i] = boundingRect(Mat(contours_g[i]));
		AspectRatio_g = (double)boundRect_g[i].height / boundRect_g[i].width;
		AreaRatio_g = (double)contourArea(contours_g[i]) / (boundRect_g[i].height*boundRect_g[i].width);
		if (AspectRatio_g < 1 + error_aspectratio && AspectRatio_g> 1 - error_aspectratio) {
			if (AreaRatio_g > (1 - error_arearatio)*PI / 4 && AreaRatio_g < (1 + error_arearatio)*PI / 4) {
				mu_g[i] = moments(contours_g[i], false);
				double x = mu_g[i].m10 / mu_g[i].m00;
				double y = mu_g[i].m01 / mu_g[i].m00;
				mark_temp_g[g_cnt][0] = x * RESIZE_LED;
				mark_temp_g[g_cnt][1] = y * RESIZE_LED;
				// putText(image_resized, to_string(g_cnt), Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 0.5);
				// circle(image_resized, Point(x, y), 3, Scalar(255, 0, 0), 0.5);
				g_candidate[g_cnt] = i;
				g_cnt = g_cnt + 1;
			}
		}
	}
	// check Aspect Ratio, Area Ratio from bianry image
	r_cnt = 0;
	for (int i = 0; i < contours_r.size(); i++) {
		boundRect_r[i] = boundingRect(Mat(contours_r[i]));
		AspectRatio_r = (double)boundRect_r[i].height / boundRect_r[i].width;
		AreaRatio_r = (double)contourArea(contours_r[i]) / (boundRect_r[i].height*boundRect_r[i].width);
		if (AspectRatio_r < 1 + error_aspectratio && AspectRatio_r > 1 - error_aspectratio) {
			if (AreaRatio_r > (1 - error_arearatio)*PI / 4 && AreaRatio_r < (1 + error_arearatio)*PI / 4) {
				mu_r[i] = moments(contours_r[i], false);
				double x = mu_r[i].m10 / mu_r[i].m00;
				double y = mu_r[i].m01 / mu_r[i].m00;
				mark_temp_r[r_cnt][0] = x * RESIZE_LED;
				mark_temp_r[r_cnt][1] = y * RESIZE_LED;
				// putText(image_resized, to_string(r_cnt), Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 0.5);
				// circle(image_resized, Point(x, y), 3, Scalar(255, 0, 0), 0.5);
				r_candidate[r_cnt] = i;
				r_cnt = r_cnt + 1;
			}
		}
	}
	// FInd square from LED candidates.
	int g1 = 0;
	int r1 = 0;
	int r2 = 0;
	int r3 = 0;
	for (int i = 0; i < g_cnt; i++) {
		for (int j = 0; j < r_cnt; j++) {
			vertical_cnt = 0;
			diagonal_cnt1 = 0;
			diagonal_cnt2 = 0;
			//check LED Area.  green led should between 0.5~ 2 times of RED led.
			if ((0.5*contourArea(contours_g[g_candidate[i]]) > contourArea(contours_r[r_candidate[j]]) || (2 * contourArea(contours_g[g_candidate[i]])) < contourArea(contours_r[r_candidate[j]]))) {
				continue;
			}
			double gr1[2] = { mark_temp_r[j][0] - mark_temp_g[i][0], mark_temp_r[j][1] - mark_temp_g[i][1] };
			gr1_size = sqrt(gr1[0] * gr1[0] + gr1[1] * gr1[1]);
			for (int k = j + 1; k < r_cnt; k++) {
				//check LED Area.  green led should between 0.5~ 2 times of RED led.
				if ((0.5* contourArea(contours_g[g_candidate[i]]) >= contourArea(contours_r[r_candidate[k]]) || (2 * contourArea(contours_g[g_candidate[i]])) <= contourArea(contours_r[r_candidate[j]]))) {
					continue;
				}
				double gr2[2] = { mark_temp_r[k][0] - mark_temp_g[i][0], mark_temp_r[k][1] - mark_temp_g[i][1] };
				gr2_size = sqrt(gr2[0] * gr2[0] + gr2[1] * gr2[1]);
				double radian = acos((gr1[0] * gr2[0] + gr1[1] * gr2[1]) / (gr2_size*gr1_size));
				radian = radian * (180 / PI);
				// check angle of vector gr1 and vector gr2.
				// check length of vetor gr1 and vector gr2.
				// determine they are LED of Squre or NOT.
				if (90 - error_radian < radian && radian < 90 + error_radian) {
					if ((1 - error_distance)*gr1_size < gr2_size && gr2_size < (1 + error_distance)*gr1_size) {
						r_vertical = k;
						vertical_cnt = vertical_cnt + 1;
					}
				}
				else if (45 - error_radian < radian && radian < 45 + error_radian) {
					if (sqrt(2)*(1 - error_distance)*gr1_size < gr2_size && gr2_size < sqrt(2)*(1 + error_distance)*gr1_size) {
						r_diagonal_1 = k;
						diagonal_cnt1 = diagonal_cnt1 + 1;
					}
					else if ((1 - error_distance)*gr1_size / sqrt(2) < gr2_size && gr2_size < (1 + error_distance)*gr1_size / sqrt(2)) {
						r_diagonal_2[diagonal_cnt2] = k;
						diagonal_cnt2 = diagonal_cnt2 + 1;
					}
				}
			}
			if (vertical_cnt >= 1 && diagonal_cnt1 >= 1) {
				if (signextract(((mark_temp_r[j][0] - mark_temp_g[i][0])*(mark_temp_r[r_diagonal_1][1] - mark_temp_g[i][1]) - (mark_temp_r[j][1] - mark_temp_g[i][1])*(mark_temp_r[r_diagonal_1][0] - mark_temp_g[i][0]))) * signextract(((mark_temp_r[j][0] - mark_temp_g[i][0])*(mark_temp_r[r_vertical][1] - mark_temp_g[i][1]) - (mark_temp_r[j][1] - mark_temp_g[i][1])*(mark_temp_r[r_vertical][0] - mark_temp_g[i][0]))) > 0)
				{
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
				if (signextract(((mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]))) * signextract(((mark[2][0] - mark[0][0])*(mark[1][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[1][0] - mark[0][0]))) > 0)
					detected = 0;
			}
			// check length of side of squre and average LED diameter. (using non-change distance 3m(between led) and 0.2m(led diameter))
			if (detected == 1) {
				double AREA = contourArea(contours_g[g_candidate[g1]]) + contourArea(contours_r[r_candidate[r1]]) + contourArea(contours_r[r_candidate[r2]]) + contourArea(contours_r[r_candidate[r3]]);
				double DISTANCE_RATIO_CAL = 88 * pow(AREA, -0.72);
				double DISTANCE_RATIO_REAL = max(gr1_size, gr2_size) / AREA;
				if (DISTANCE_RATIO_REAL > DISTANCE_RATIO_CAL * 2 || DISTANCE_RATIO_REAL < DISTANCE_RATIO_CAL*0.4)
					detected = 0;
			
			}
			// check numbering is counterclockwise. if not, change 1 and 3.
			if ((mark[2][0] - mark[0][0])*(mark[3][1] - mark[0][1]) - (mark[2][1] - mark[0][1])*(mark[3][0] - mark[0][0]) > 0){
				int temp[2] = { mark[1][0],mark[1][1] };
				mark[1][0] = mark[3][0];
				mark[1][1] = mark[3][1];
				mark[3][0] = temp[0];
				mark[3][1] = temp[1];
			}
			if (detected == 1) {
				for (int i = 0; i < 4; i++) {
					mark_flag[i] = true;
					//std::cout << "LED" << i << "x,y :" << mark[i][0] << ", " << mark[i][1] << endl;
					//putText(image_resized, to_string(i), Point(mark[i][0] / 4, mark[i][1] / 4), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 1);
				}
				break;
			}
		}
		if (detected == 1)
			break;
	}
	if (detected != 1){
		//std::cout << "detect fail" << endl;
	}
	//cout << "done" << endl;
	//cv::namedWindow("image", WINDOW_NORMAL); // Create a window for display.
	//cv::imshow("image", image_resized);                // Show our image inside it.
	//cv::namedWindow("binary_r", WINDOW_NORMAL); // Create a window for display.
	//cv::imshow("binary_r", image_binary_r);                // Show our image inside it.
	//cv::namedWindow("binary_g", WINDOW_NORMAL); // Create a window for display.
	//cv::imshow("binary_g", image_binary_g);
}
