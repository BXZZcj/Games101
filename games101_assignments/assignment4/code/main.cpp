#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
    cv::Mat window = *(cv::Mat *)userdata;
    // new control_point updated, so we need to erase the previous bezier curve
    window.setTo(cv::Scalar(0));
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.00001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
    {
        return control_points[0];
    }

    int pointNum = control_points.size();
    std::vector<cv::Point2f> newControlPoints;
    cv::Point2f newPoint;
    for (int i = 0; i < pointNum - 1; i++)
    {
        newPoint = t * control_points[i] + (1 - t) * control_points[i + 1];
        newControlPoints.emplace_back(newPoint);
    }

    return recursive_bezier(newControlPoints, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window, cv::Vec3b color)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    if (control_points.size() == 0)
        return;

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        // anti-aliasing
        cv::Point2f roundPoints[9];
        int index = 0;
        for (int i = -1; i <= -1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                // roundPoints[index]={floor(point.x)+0.5+i,floor(point.y)+0.5+j};
                float x = floor(point.x) + 0.5 + i, y = floor(point.y) + 0.5 + j;
                float distance = sqrt(pow(x - point.x, 2) + pow(y - point.y, 2));
                float opacity = 1 - sqrt(2) / 3 * distance;
                window.at<cv::Vec3b>(floor(y), floor(x)) = opacity * color;
                index++;
            }
        }

        // window.at<cv::Vec3b>(point.y, point.x) = color;
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, &window);

    int key = -1;

    // for (auto &point : control_points)
    // {
    //     cv::circle(window, point, 3, {255, 255, 255}, 3);
    // }
    // bezier(control_points, window,{0,255,0});
    // cv::imshow("Bezier Curve", window);

    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }
        bezier(control_points, window, {0, 255, 0});
        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(10);
    }

    return 0;
}
