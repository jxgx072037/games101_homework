// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

auto to_matrix4(const Eigen::Matrix3f& m3)
{
    Eigen::Matrix4f m4;
    m4 << m3(0, 0), m3(0, 1), m3(0, 2), 0.0f,
          m3(1, 0), m3(1, 1), m3(1, 2), 0.0f,
          m3(2, 0), m3(2, 1), m3(2, 2), 0.0f,
          0, 0, 0, 1.0f;
    return m4;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)

{
    Eigen::Matrix3f model = Eigen::Matrix3f::Identity();

    angle = angle/180 * std::acos(-1);
    Eigen::Matrix3f axis_skew_symmetric;
    axis_skew_symmetric << 0, -axis.z(), axis.y(),
                           axis.z(), 0, -axis.x(),
                           -axis.y(), axis.x(), 0;
                              
    model = std::cos(angle) * model + (1-std::cos(angle)) * axis * axis.transpose() + std::sin(angle) * axis_skew_symmetric;
    return to_matrix4(model);

}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float top = std::tan((eye_fov/2)/180 * std::acos(-1)) * abs(zNear);
    float right = top * aspect_ratio;

    Eigen::Matrix4f perspective_projection, orthographic_projection_trans, orthographic_projection_scale;
    perspective_projection << zNear, 0, 0, 0, 
                              0, zNear, 0, 0, 
                              0, 0, zNear+zFar, -zNear * zFar, 
                              0, 0, 1, 0;
    orthographic_projection_trans <<  1, 0, 0, 0, 
                                      0, 1, 0, 0, 
                                      0, 0, 1, -(zNear+zFar)/2,
                                      0, 0, 0, 1;
    orthographic_projection_scale << 1/right, 0, 0, 0, 
                                     0, 1/top, 0, 0, 
                                     0, 0, 2/(zNear-zFar), 0,
                                     0, 0, 0, 1;
    projection = orthographic_projection_scale * orthographic_projection_trans * perspective_projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.jpg";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    Eigen::Vector3f axis(1, 1, 1);

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        // change zNear and zFar to real z-axis value: -0.1, -50, not just distance;
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        // change zNear and zFar to real z-axis value: -0.1, -50, not just distance;
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        //angle += 10;
    }

    return 0;
}
// clang-format on