// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <limits.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (-50 + 0.1) / 2.0;
    float f2 = (-50 - 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        //std::cout << v << std::endl;
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

bool point_in_triangle(int x, int y, Triangle t){
    Eigen::Vector3d p(x, y, 1.0f);
    Eigen::Vector3d t1(t.v[0][0], t.v[0][1], 1.0f);
    Eigen::Vector3d t2(t.v[1][0], t.v[1][1], 1.0f);
    Eigen::Vector3d t3(t.v[2][0], t.v[2][1], 1.0f);

    float cross_1 = ((p-t1).cross(t2-t1))[2];
    float cross_2 = ((p-t2).cross(t3-t2))[2];
    float cross_3 = ((p-t3).cross(t1-t3))[2];

    if (((cross_1>0 && cross_2>0) && cross_3>0) || ((cross_1<0 && cross_2<0) && cross_3<0)){
        return true;
    } else {
        return false;
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    //std::cout << t.v[0][0] << std::endl;
    int bb_width_min = int(std::min({t.v[0][0], t.v[1][0], t.v[2][0]}));
    int bb_width_max = ceil(std::max({t.v[0][0], t.v[1][0], t.v[2][0]}));

    int bb_height_min = int(std::min({t.v[0][1], t.v[1][1], t.v[2][1]}));
    int bb_height_max = ceil(std::max({t.v[0][1], t.v[1][1], t.v[2][1]}));


    //no MSAA
    for (int bb_width = bb_width_min;bb_width < bb_width_max;bb_width++){
        for(int bb_height = bb_height_min;bb_height < bb_height_max;bb_height++){
            float x = bb_width + 0.5;
            float y = bb_height + 0.5;

            if (point_in_triangle(x, y, t)){
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                int z_index = get_index(bb_width, bb_height);

                if ((z_interpolated < depth_buf[z_index])){
                    depth_buf[z_index] = z_interpolated;
                    set_pixel(Eigen::Vector3f(bb_width, bb_height, 1.0f), t.getColor());
                }    
            }
        }
    }

    //MSAA with black edge
    // for (int bb_width = bb_width_min;bb_width < bb_width_max;bb_width++){
    //     for(int bb_height = bb_height_min;bb_height < bb_height_max;bb_height++){
    //         float x = bb_width + 0.5;
    //         float y = bb_height + 0.5;

    //         Eigen::MatrixXf mini_pixel(4,2);
    //         mini_pixel << 0.25, 0.25, 0.25, 0.75, 0.75, 0.25, 0.75, 0.75;

    //         //std::cout << mini_pixel << std::endl;

    //         float MSAA_flag = 0.0f;

    //         for(float mini_pixel_index=0;mini_pixel_index<4;mini_pixel_index++){
    //             if(point_in_triangle(x+mini_pixel(mini_pixel_index, 0), y+mini_pixel(mini_pixel_index, 1), t)){
    //                 MSAA_flag++;
    //             }
    //         }

    //         if (MSAA_flag==0){
    //             continue;
    //         } else{
    //             auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;

    //             int z_index = get_index(bb_width, bb_height);

    //             if ((z_interpolated < depth_buf[z_index])){
    //                 depth_buf[z_index] = z_interpolated;
    //                 set_pixel(Eigen::Vector3f(bb_width, bb_height, 1.0f), t.getColor() * (MSAA_flag/4.0f));
    //             }    
    //         }
    //     }
    // }

    //MSAA without black edge
    // for (int bb_width = bb_width_min;bb_width < bb_width_max;bb_width++){
    //     for(int bb_height = bb_height_min;bb_height < bb_height_max;bb_height++){
    //         float x = bb_width + 0.5;
    //         float y = bb_height + 0.5;

    //         Eigen::MatrixXf mini_pixel(4,2);
    //         mini_pixel << 0.25, 0.25, 0.25, 0.75, 0.75, 0.25, 0.75, 0.75;

    //         //std::cout << mini_pixel << std::endl;

    //         float MSAA_flag = 0.0f;

    //         for(float mini_pixel_index=0;mini_pixel_index<4;mini_pixel_index++){
    //             float mini_pixel_x = x+mini_pixel(mini_pixel_index, 0);
    //             float mini_pixel_y = y+mini_pixel(mini_pixel_index, 1);
    //             if(point_in_triangle(mini_pixel_x, mini_pixel_y, t)){
    //                 MSAA_flag++;
    //             }
    //         }

    //         if (MSAA_flag==0){
    //             continue;
    //         } else{
    //             auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;

    //             int z_index = get_index(bb_width, bb_height);

    //             for(int msaa_num=0;msaa_num<MSAA_flag;msaa_num++){
    //                 if(depth_buf[z_index](msaa_num,0) < 1.0f){
    //                     if (z_interpolated < depth_buf[z_index](msaa_num,0) and MSAA_flag==4){
    //                         depth_buf[z_index][msaa_num] = z_interpolated;
    //                         color_buf[z_index][msaa_num] = t.getColor();

    //                     } else {
    //                         continue;
    //                     }
    //                 } else {
    //                     depth_buf[z_index][msaa_num] = z_interpolated;
    //                     // std::cout << t.getColor() << std::endl;
    //                     // std::cout << color_buf[z_index][msaa_num] << std::endl;
    //                     color_buf[z_index][msaa_num] = t.getColor();
    //                 }
    //             } 

    //             Eigen::Vector3f pixel_color(0.0f, 0.0f, 0.0f);
    //             for(int i=0;i<4;i++){ 
    //                 pixel_color += 0.25 * color_buf[z_index][i];
    //             }
    //             set_pixel(Eigen::Vector3f(bb_width, bb_height, 1.0f), pixel_color);
    //         }        
    //     }
    // }

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {   
        //MSAA without black edge
        // Eigen::Vector4f depth_buf_vector(1.0f, 1.0f, 1.0f, 1.0f);
        // std::fill(depth_buf.begin(), depth_buf.end(), depth_buf_vector);

        //MSAA with black edge and no MSAA
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    //MSAA without black edge
    // if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    // {   
    //     std::vector<Eigen::Vector3f> cols 
    //     {
    //         {0.0, 0.0, 0.0},
    //         {0.0, 0.0, 0.0},
    //         {0.0, 0.0, 0.0},
    //         {0.0, 0.0, 0.0}
    //     };
    //     std::fill(color_buf.begin(), color_buf.end(), cols);
    // }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    color_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    auto ind = (height-point.y())*width + point.x();
    //std::cout << "ind: " << ind <<std::endl;
    //std::cout << "frame_buf's size: " << float(frame_buf.size()) << std::endl;
    ind = std::min(ind, float(frame_buf.size()));
    frame_buf[ind] = color;

}

// clang-format on