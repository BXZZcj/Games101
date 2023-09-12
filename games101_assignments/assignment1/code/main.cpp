#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f Rodrigues(Eigen::Vector3f axis, float angle)
{
    Eigen::Matrix4f rotate;
    Eigen::Matrix3f nonhomo_rotate;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N;
    N << 0, -axis.z(), -axis.y(),
        axis.z(), 0, -axis.x(),
        -axis.y(), axis.x(), 0;

    nonhomo_rotate = cos(angle / 180.0 * M_PI) * I + (1 - cos(angle / 180.0 * M_PI)) * axis * axis.transpose() + sin(angle / 180.0 * M_PI) * N;

    rotate << nonhomo_rotate, Eigen::Vector3f::Zero(), Eigen::Vector4f(0, 0, 0, 1).transpose();
    return rotate;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, Eigen::Vector3f axis = Eigen::Vector3f(0, 0, 1))
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.aroun
    // Then return it.
    // Eigen::Matrix4f rotate;

    Eigen::Matrix4f rotate = Rodrigues(axis, rotation_angle);
    model = model * rotate;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float n = -abs(zNear), f = -abs(zFar);
    float t = abs(n) * sin(eye_fov / 2 / 180.0 * M_PI), b = -t;
    float r = (t - b) * aspect_ratio / 2, l = -r;

    Eigen::Matrix4f ortho_sub1, ortho_sub2, persp2ortho;
    ortho_sub1 << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    ortho_sub2 << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    persp2ortho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, (n + f), -n * f,
        0, 0, 1, 0;
    projection = ortho_sub1 * ortho_sub2 * persp2ortho * projection;

    return projection;
}

int main(int argc, const char **argv)
{
    float rotate_angle = 0;
    bool command_line = false;
    Eigen::Vector3f axis = {0, 0, 1};
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        rotate_angle = std::stof(argv[2]); // -r by default
        if (argc == 5)
        {
            filename = std::string(argv[4]); //-f by default
        }
        if (argc == 9)
        {
            filename = std::string(argv[4]);
            axis[0] = std::stof(argv[6]), axis[1] = std::stof(argv[7]), axis[2] = std::stof(argv[2]); //-a by default
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos); // in pos_buf: 0:{{2,0,-2},{0,2,-2},{-2,0,-2}}
    auto ind_id = r.load_indices(ind);   // in ind_buf: 1:{{0,1,2}}

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(rotate_angle, axis));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(rotate_angle, axis));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            rotate_angle += 10;
        }
        else if (key == 'd')
        {
            rotate_angle -= 10;
        }
    }

    return 0;
}