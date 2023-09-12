// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace std;


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

static float crossProd(Vector2f v1,Vector2f v2)
{
    return v1.x()*v2.y()-v2.x()*v1.y();
}

static float distancePointLine(Vector2f point,std::array<Vector2f,2> line)
{
    Vector2f vectorLine={line[0].x()-line[1].x(),line[0].y()-line[1].y()};
    Vector2f vectorPoint={point.x()-line[1].x(),point.y()-line[1].y()}; 
    float distance=abs(crossProd(vectorLine,vectorPoint))/sqrt(pow(vectorLine[0],2)+pow(vectorLine[1],2));
    return distance;
}

static float insideTriangle(int xFloor, int yFloor, const Vector3f* _v)
{   
    float x=xFloor+0.5,y=yFloor+0.5;
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f side0={_v[0].x()-_v[1].x(),_v[0].y()-_v[1].y()},V0={x-_v[1].x(),y-_v[1].y()};
    Vector2f side1={_v[1].x()-_v[2].x(),_v[1].y()-_v[2].y()},V1={x-_v[2].x(),y-_v[2].y()};
    Vector2f side2={_v[2].x()-_v[0].x(),_v[2].y()-_v[0].y()},V2={x-_v[0].x(),y-_v[0].y()};

    int insideNum=0;
    std::array<Vector2f,2> line0{Vector2f(_v[0].x(),_v[0].y()),Vector2f(_v[1].x(),_v[1].y())};
    std::array<Vector2f,2> line1{Vector2f(_v[1].x(),_v[1].y()),Vector2f(_v[2].x(),_v[2].y())};
    std::array<Vector2f,2> line2{Vector2f(_v[2].x(),_v[2].y()),Vector2f(_v[0].x(),_v[0].y())};
    if(distancePointLine({x,y},line0)<0.36
        ||distancePointLine({x,y},line1)<0.36
        ||distancePointLine({x,y},line2)<0.36)
    {
        for(float i=y-0.25;i<=y+0.25;i+=0.5)
        {
            for(float j=x-0.25;j<=x+0.25;j+=0.5)
            {
                Vector2f supV0={j-_v[1].x(),i-_v[1].y()};
                Vector2f supV1={j-_v[2].x(),i-_v[2].y()};
                Vector2f supV2={j-_v[0].x(),i-_v[0].y()};
                float cross0=crossProd(side0,supV0),cross1=crossProd(side1,supV1),cross2=crossProd(side2,supV2);
                
                if(cross0>=0&&cross1>=0&&cross2>=0
                    ||cross0<=0&&cross1<=0&&cross2<=0)
                    insideNum++;
            }
        }
    }
    else
    {
        float cross0=crossProd(side0,V0),cross1=crossProd(side1,V1),cross2=crossProd(side2,V2);
        if(cross0>=0&&cross1>=0&&cross2>=0
            ||cross0<=0&&cross1<=0&&cross2<=0)
            insideNum=4;
        else
            insideNum=0;
    }
    return insideNum/4.0;
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

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)//in this projection ,this for will cycle twice for 2 triangles
    {
        //t is a triangle which has 3 vertex and every vertex coordination isn't homogeneous
        Triangle t;
        //v has 3 vertex corresponding to a triangle..And a vertex is a homogeneous
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
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

        //render a triangle which holds 3 vertex
        rasterize_triangle(t);
    }
}


//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    
    // If so, use the following code to get the interpolated z value.
    std::vector<Vector3f> pointSet=std::vector<Vector3f>(t.v,t.v+3);
    std::array<float,6> BBox=getTriangleBBox(pointSet);
    float xMin=BBox[0],xMax=BBox[1],yMin=BBox[2],yMax=BBox[3];
    
    float opacity;
    for(int x=floor(xMin);x<ceil(xMax);x++)
    {
        for(int y=floor(yMin);y<ceil(yMax);y++)
        {
            opacity=insideTriangle(x, y, t.v);
            if(opacity-0>1e-5)
            {
                auto [alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                //50 equals to 2*f1.Find f1 in function draw.
                if((50-z_interpolated)<depth_buf[get_index(x,y)])
                {   
                    depth_buf[get_index(x,y)]=z_interpolated;
                    set_pixel(Vector3f(x,y,1.0), opacity*t.getColor());
                }
            }
        }
    }

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

std::array<float,6> rst::rasterizer::getTriangleBBox(const std::vector<Vector3f> pointSet)
{
    float inf=std::numeric_limits<float>::infinity();
    float xMin=inf,xMax=-inf,yMin=inf,yMax=-inf,zMin=inf,zMax=-inf;
    float x,y,z;
    for(auto point:pointSet)
    {
        x=point.x(),y=point.y(),z=point.z();
        if(x<xMin)xMin=x;
        if(x>xMax)xMax=x;
        if(y<yMin)yMin=y;
        if(y>yMax)yMax=y;
        if(z<zMin)zMin=z;
        if(z>zMax)zMax=z;
    }
    return {xMin,xMax,yMin,yMax,zMin,zMax};
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
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on