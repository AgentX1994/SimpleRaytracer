#include "obj_loader.hpp"

#include <algorithm>
#include <charconv>
#include <format>
#include <fstream>
#include <iostream>
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

std::vector<const char *> split(std::string &s,
                                const std::string_view delimiter)
{
    // Adapted from: https://stackoverflow.com/a/14266139
    std::vector<const char *> tokens;
    size_t last = 0;
    size_t next = 0;
    std::string_view token;
    while ((next = s.find(delimiter, last)) != std::string_view::npos)
    {
        token = s.substr(last, next - last);
        s[next] = '\0';
        auto ptr = s.data() + last;
        if (*ptr)
        {
            tokens.push_back(ptr);
        }
        last = next + 1;
    }
    auto ptr = s.data() + last;
    if (*ptr)
    {
        tokens.push_back(ptr);
    }

    return tokens;
}

template <typename T>
std::tuple<T, T, T> parse_three_of(std::string &str)
{
    std::string backup = str;
    auto tokens = split(str, " ");
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

std::shared_ptr<Mesh> load_obj_file(const std::filesystem::path &file_path)
{
    // TODO: Support normals and UVs
    std::vector<Point3f> vertices;
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
        else if (line.starts_with('v'))
        {
            std::string to_parse = line.substr(2);
            auto [x, y, z] = parse_three_floats(to_parse);
            Point3f vert(x, y, z);
            vertices.push_back(vert);
        }
        else if (line.starts_with('f'))
        {
            std::string to_parse = line.substr(2);
            auto [v0_index, v1_index, v2_index] = parse_three_ints(to_parse);
            Point3f v0 = vertices[v0_index];
            Point3f v1 = vertices[v1_index];
            Point3f v2 = vertices[v2_index];
            Triangle tri(v0, v1, v2);
            tris.push_back(tri);
        }
        else
        {
            std::cout << "Unhandled line: " << line << '\n';
        }
    }

    std::cout << "Loaded mesh with " << tris.size() << " triangles\n";
    return make_shared<Mesh>(std::move(tris));
}
}  // namespace raytracer
