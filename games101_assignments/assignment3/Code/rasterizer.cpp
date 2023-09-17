//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f> &normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}

// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1)
    {
        if (dx >= 0)
        {
            x = x1;
            y = y1;
            xe = x2;
        }
        else
        {
            x = x2;
            y = y2;
            xe = x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = x1;
            y = y1;
            ye = y2;
        }
        else
        {
            x = x2;
            y = y2;
            ye = y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static float crossProd(Vector2f v1, Vector2f v2)
{
    return v1.x() * v2.y() - v2.x() * v1.y();
}

static float distancePointLine(Vector2f point, std::array<Vector2f, 2> line)
{
    Vector2f vectorLine = {line[0].x() - line[1].x(), line[0].y() - line[1].y()};
    Vector2f vectorPoint = {point.x() - line[1].x(), point.y() - line[1].y()};
    float distance = abs(crossProd(vectorLine, vectorPoint)) / sqrt(pow(vectorLine[0], 2) + pow(vectorLine[1], 2));
    return distance;
}

static float insideTriangle(int xFloor, int yFloor, const Vector4f *_v)
{
    float x = xFloor + 0.5, y = yFloor + 0.5;
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f side0 = {_v[0].x() - _v[1].x(), _v[0].y() - _v[1].y()}, V0 = {x - _v[1].x(), y - _v[1].y()};
    Vector2f side1 = {_v[1].x() - _v[2].x(), _v[1].y() - _v[2].y()}, V1 = {x - _v[2].x(), y - _v[2].y()};
    Vector2f side2 = {_v[2].x() - _v[0].x(), _v[2].y() - _v[0].y()}, V2 = {x - _v[0].x(), y - _v[0].y()};

    int insideNum = 0;
    std::array<Vector2f, 2> line0{Vector2f(_v[0].x(), _v[0].y()), Vector2f(_v[1].x(), _v[1].y())};
    std::array<Vector2f, 2> line1{Vector2f(_v[1].x(), _v[1].y()), Vector2f(_v[2].x(), _v[2].y())};
    std::array<Vector2f, 2> line2{Vector2f(_v[2].x(), _v[2].y()), Vector2f(_v[0].x(), _v[0].y())};
    if (distancePointLine({x, y}, line0) < 0.36 || distancePointLine({x, y}, line1) < 0.36 || distancePointLine({x, y}, line2) < 0.36)
    {
        for (float i = y - 0.25; i <= y + 0.25; i += 0.5)
        {
            for (float j = x - 0.25; j <= x + 0.25; j += 0.5)
            {
                Vector2f supV0 = {j - _v[1].x(), i - _v[1].y()};
                Vector2f supV1 = {j - _v[2].x(), i - _v[2].y()};
                Vector2f supV2 = {j - _v[0].x(), i - _v[0].y()};
                float cross0 = crossProd(side0, supV0), cross1 = crossProd(side1, supV1), cross2 = crossProd(side2, supV2);

                if (cross0 >= 0 && cross1 >= 0 && cross2 >= 0 || cross0 <= 0 && cross1 <= 0 && cross2 <= 0)
                    insideNum++;
            }
        }
    }
    else
    {
        float cross0 = crossProd(side0, V0), cross1 = crossProd(side1, V1), cross2 = crossProd(side2, V2);
        if (cross0 >= 0 && cross1 >= 0 && cross2 >= 0 || cross0 <= 0 && cross1 <= 0 && cross2 <= 0)
            insideNum = 4;
        else
            insideNum = 0;
    }
    return insideNum / 4.0;
}
// static bool insideTriangle(float x, float y, const Vector4f* _v){
//     Vector3f v[3];
//     for(int i=0;i<3;i++)
//         v[i] = {_v[i].x(),_v[i].y(), 1.0};
//     Vector3f f0,f1,f2;
//     f0 = v[1].cross(v[0]);
//     f1 = v[2].cross(v[1]);
//     f2 = v[0].cross(v[2]);
//     Vector3f p(x,y,1.);
//     if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
//         return true;
//     return false;
// }

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f *v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList)
{
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto &t : TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm{
            (view * model * t->v[0]),
            (view * model * t->v[1]),
            (view * model * t->v[2])};

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto &v)
                       { return v.template head<3>(); });

        Eigen::Vector4f v[] = {
            mvp * t->v[0],
            mvp * t->v[1],
            mvp * t->v[2]};
        // Homogeneous division
        for (auto &vec : v)
        {
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
            inv_trans * to_vec4(t->normal[0], 0.0f),
            inv_trans * to_vec4(t->normal[1], 0.0f),
            inv_trans * to_vec4(t->normal[2], 0.0f)};

        // Viewport transformation
        for (auto &vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            // screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            // view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148, 121.0, 92.0);
        newtri.setColor(1, 148, 121.0, 92.0);
        newtri.setColor(2, 148, 121.0, 92.0);
        // newtri.setColor(0, 0,0,255);
        // newtri.setColor(1, 0,0,255);
        // newtri.setColor(2, 0,0,255);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f &vert1, const Eigen::Vector3f &vert2, const Eigen::Vector3f &vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f &vert1, const Eigen::Vector2f &vert2, const Eigen::Vector2f &vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &view_pos)
{
    auto vertex = t.toVector4();
    float xMin = std::min({vertex[0].x(), vertex[1].x(), vertex[2].x()});
    float xMax = std::max({vertex[0].x(), vertex[1].x(), vertex[2].x()});
    float yMin = std::min({vertex[0].y(), vertex[1].y(), vertex[2].y()});
    float yMax = std::max({vertex[0].y(), vertex[1].y(), vertex[2].y()});

    float opacity;
    for (int x = floor(xMin); x < ceil(xMax); x++)
    {
        for (int y = floor(yMin); y < ceil(yMax); y++)
        {
            opacity = insideTriangle(x + 0.5, y + 0.5, t.v);
            if ((abs(opacity) - 0) > 1e-5)
            {
                auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                float w_reciprocal = 1.0 / (alpha / view_pos[0].z() + beta / view_pos[1].z() + gamma / view_pos[2].z());
                // interpolate the attributes
                float interpolated_z = alpha * view_pos[0].z() / view_pos[0].z() + beta * view_pos[0].z() / view_pos[1].z() + gamma * view_pos[0].z() / view_pos[2].z();
                interpolated_z *= w_reciprocal;

                if ((-interpolated_z) < depth_buf[get_index(x, y)])
                {
                    auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], t.color[1], t.color[2], 1);
                    auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1);
                    auto interpolated_texcoords = interpolate(alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1);
                    auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);

                    fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    payload.view_pos = interpolated_shadingcoords;
                    auto pixel_color = fragment_shader(payload);

                    depth_buf[get_index(x, y)] = -interpolated_z;
                    // the anti-aliasing performance is not good
                    set_pixel(Vector2i(x, y), pixel_color);
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
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

    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    // old index: auto ind = point.y() + point.x() * width;
    int ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}