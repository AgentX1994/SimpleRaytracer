#include "obj_loader.hpp"

#include <algorithm>
#include <cstring>
#include <format>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <vector>

namespace raytracer
{

// trim functions from https://stackoverflow.com/a/217605
// trim from start (in place)
inline void ltrim(std::string &s)
{
    s.erase(s.begin(),
            std::find_if(s.begin(), s.end(),
                         [](unsigned char ch) { return !std::isspace(ch); }));
}

// trim from end (in place)
inline void rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         [](unsigned char ch) { return !std::isspace(ch); })
                .base(),
            s.end());
}

inline void trim(std::string &s)
{
    ltrim(s);
    rtrim(s);
}

std::vector<char *> split(std::string &s, const std::string_view delimiter,
                          bool allow_empty = false)
{
    // Adapted from: https://stackoverflow.com/a/14266139
    std::vector<char *> tokens;
    size_t last = 0;
    size_t next = 0;
    std::string_view token;
    while ((next = s.find(delimiter, last)) != std::string_view::npos)
    {
        token = s.substr(last, next - last);
        s[next] = '\0';
        auto ptr = s.data() + last;
        if (*ptr && !allow_empty)
        {
            tokens.push_back(ptr);
        }
        last = next + 1;
    }
    auto ptr = s.data() + last;
    if (*ptr && !allow_empty)
    {
        tokens.push_back(ptr);
    }

    return tokens;
}

template <typename T>
std::tuple<T, T, T> parse_three_of(std::string &str,
                                   std::string_view delimiter = " ",
                                   bool allow_empty = false)
{
    std::string backup = str;
    auto tokens = split(str, delimiter);
    if (tokens.size() != 3)
    {
        throw std::runtime_error(std::format(
            "Invalid line: {}, only found {} tokens", backup, tokens.size()));
    }
    T first, second, third;
    // All because Apple refuses to support from_chars for floats...
    if constexpr (std::is_floating_point_v<T>)
    {
        char *strend;
        first = std::strtof(tokens[0], &strend);
        if (tokens[0] == strend)
        {
            throw std::runtime_error(std::format(
                "Invalid number: \"{}\" in line: \"{}\"", tokens[0], backup));
        }
        second = std::strtof(tokens[1], &strend);
        if (tokens[1] == strend)
        {
            throw std::runtime_error(std::format(
                "Invalid number: \"{}\" in line: \"{}\"", tokens[1], backup));
        }
        third = std::strtof(tokens[2], &strend);
        if (tokens[2] == strend)
        {
            throw std::runtime_error(std::format(
                "Invalid number: \"{}\" in line: \"{}\"", tokens[2], backup));
        }
    }
    else if constexpr (std::is_integral_v<T>)
    {
        char *strend;
        first = std::strtoul(tokens[0], &strend, 0);
        if (tokens[0] == strend)
        {
            throw std::runtime_error(std::format("Invalid number: {}", str));
        }
        second = std::strtoul(tokens[1], &strend, 0);
        if (tokens[1] == strend)
        {
            throw std::runtime_error(std::format("Invalid number: {}", str));
        }
        third = std::strtoul(tokens[2], &strend, 0);
        if (tokens[2] == strend)
        {
            throw std::runtime_error(std::format("Invalid number: {}", str));
        }
    }
    return std::tuple(first, second, third);
}

std::tuple<float, float, float> parse_three_floats(std::string &str)
{
    return parse_three_of<float>(str);
}

std::tuple<size_t, size_t, size_t> parse_three_ints(std::string &str)
{
    return parse_three_of<size_t>(str);
}

struct TriangleIndices
{
    std::array<int64_t, 3> vertices = {0};
    std::array<int64_t, 3> uvs = {0};
    std::array<int64_t, 3> normals = {0};
};

std::ostream &operator<<(std::ostream &ostr, const TriangleIndices &indices)
{
    ostr << "TriangleIndices:"
         << "\n\tVertices: " << indices.vertices[0] << ", "
         << indices.vertices[1] << ", " << indices.vertices[2]
         << "\n\tUVs: " << indices.uvs[0] << ", " << indices.uvs[1] << ", "
         << indices.uvs[2] << "\n\tNormals: " << indices.normals[0] << ", "
         << indices.normals[1] << ", " << indices.normals[2];
    return ostr;
}

// This does the same thing as strtok_r, but doesn't count consecutive
// delimiters as one delimiter
char *my_strtok(char *ptr, const char *delimiter, char **saveptr)
{
    if (saveptr == nullptr)
    {
        throw std::runtime_error("saveptr cannot be null");
    }
    // check if this is starting a new string or not
    char *start;
    if (ptr != nullptr)
    {
        start = ptr;
    }
    else
    {
        if (*saveptr == nullptr)
        {
            throw std::runtime_error("Invalid saveptr");
        }
        start = *saveptr;
    }
    // new string
    char *next = strpbrk(start, delimiter);
    if (next == nullptr)
    {
        if (*start == '\0')
        {
            return nullptr;
        }
        else
        {
            return start;
        }
    }
    else
    {
        *next = '\0';
        *saveptr = next + 1;
        return start;
    }
}

std::vector<TriangleIndices> ParseFace(std::string &str)
{
    auto index_groups = split(str, " ");
    std::vector<int64_t> vertices(index_groups.size(), 0);
    std::vector<int64_t> uvs(index_groups.size(), 0);
    std::vector<int64_t> normals(index_groups.size(), 0);
    if (index_groups.size() < 3)
    {
        throw std::runtime_error(std::format(
            "Invalid number of vertices in face: {}", index_groups.size()));
    }

    // per v/uv/n group
    for (size_t i = 0; i < index_groups.size(); ++i)
    {
        char *saveptr;
        // parse vertex index
        auto ptr = my_strtok(index_groups[i], "/", &saveptr);
        if (ptr == nullptr)
        {
            throw std::runtime_error("No vertex in f directive");
        }
        char *strend;
        vertices[i] = std::strtoul(ptr, &strend, 0);
        if (ptr == strend)
        {
            throw std::runtime_error(std::format("Invalid number: {}", str));
        }
        // Parse UV index
        ptr = my_strtok(nullptr, "/", &saveptr);
        if (ptr != nullptr)
        {
            // We have a uv
            uvs[i] = std::strtoul(ptr, &strend, 0);
            // TODO what do we do with the return value?
        }
        // Parse normal
        ptr = my_strtok(nullptr, "/", &saveptr);
        if (ptr != nullptr)
        {
            // We have a normal
            normals[i] = std::strtoul(ptr, &strend, 0);
            // TODO what do we do with the return value?
        }
    }
    // TODO: ensure polygon is convex

    size_t number_of_tris = index_groups.size() - 2;
    std::vector<TriangleIndices> tris(number_of_tris);
    // Per triangle in face
    for (size_t i = 0; i < number_of_tris; ++i)
    {
        tris[i].vertices[0] = vertices[0];
        tris[i].vertices[1] = vertices[i + 1];
        tris[i].vertices[2] = vertices[i + 2];
        tris[i].uvs[0] = uvs[0];
        tris[i].uvs[1] = uvs[i + 1];
        tris[i].uvs[2] = uvs[i + 2];
        tris[i].normals[0] = normals[0];
        tris[i].normals[1] = normals[i + 1];
        tris[i].normals[2] = normals[i + 2];
    }

    return tris;
}

std::shared_ptr<Mesh> load_obj_file(const std::filesystem::path &file_path)
{
    // TODO: Support normals and UVs
    std::vector<Point3f> vertices;
    std::vector<Vec3f> normals;
    std::vector<TriangleIndices> index_list;
    std::vector<Triangle> tris;

    std::ifstream obj_file(file_path);

    for (std::string line; std::getline(obj_file, line);)
    {
        trim(line);
        if (line.starts_with('#'))
        {
            // Comment, ignore
            continue;
        }
        else if (line.starts_with("v "))
        {
            std::string to_parse = line.substr(2);
            auto [x, y, z] = parse_three_floats(to_parse);
            Point3f vert(x, y, z);
            vertices.push_back(vert);
        }
        else if (line.starts_with("vn"))
        {
            std::string to_parse = line.substr(2);
            auto [x, y, z] = parse_three_floats(to_parse);
            Vec3f norm(x, y, z);
            norm.Normalize();
            normals.push_back(norm);
        }
        else if (line.starts_with('f'))
        {
            std::string to_parse = line.substr(2);
            std::vector<TriangleIndices> tris;
            try
            {
                tris = ParseFace(to_parse);
            }
            catch (const std::exception &e)
            {
                throw std::runtime_error(
                    std::format("Error parsing line: {}: {}", line, e.what()));
            }
            for (auto indices : tris)
            {
                if (indices.vertices[0] == 0 || indices.vertices[1] == 0 ||
                    indices.vertices[2] == 0)
                {
                    throw std::runtime_error(
                        "Invalid triangle vertex indices!");
                }
                index_list.push_back(indices);
            }
        }
        else
        {
            std::cout << "Unhandled line: " << line << '\n';
        }
    }

    for (auto index : index_list)
    {
        Point3f v0 = index.vertices[0] > 0
                         ? vertices[index.vertices[0] - 1]
                         : vertices[vertices.size() + index.vertices[0]];
        Point3f v1 = index.vertices[1] > 0
                         ? vertices[index.vertices[1] - 1]
                         : vertices[vertices.size() + index.vertices[1]];
        Point3f v2 = index.vertices[2] > 0
                         ? vertices[index.vertices[2] - 1]
                         : vertices[vertices.size() + index.vertices[2]];
        Vec3f n0, n1, n2;
        if (index.normals[0] != 0 && index.normals[1] != 0 &&
            index.normals[2] != 0 && !normals.empty())
        {
            n0 = index.normals[0] > 0
                     ? normals[index.normals[0] - 1]
                     : normals[normals.size() + index.normals[0]];
            n1 = index.normals[1] > 0
                     ? normals[index.normals[1] - 1]
                     : normals[normals.size() + index.normals[1]];
            n2 = index.normals[2] > 0
                     ? normals[index.normals[2] - 1]
                     : normals[normals.size() + index.normals[2]];
        }
        Triangle tri(v0, v1, v2, n0, n1, n2);
        tris.push_back(tri);
    }

    std::cout << "Loaded mesh: " << file_path << ".\n";
    std::cout << "\tVertices: " << vertices.size() << '\n';
    std::cout << "\tnormals: " << normals.size() << '\n';
    std::cout << "\tTriangles: " << tris.size() << '\n';
    return make_shared<Mesh>(std::move(tris));
}
}  // namespace raytracer
