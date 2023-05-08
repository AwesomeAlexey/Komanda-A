#pragma once

#include <bspline/splines.h>
#include <cppmisc/json.h>


template <typename T, int NRows>
struct Converter<Eigen::Matrix<T, NRows, 1>>
{
    using Mat = Eigen::Matrix<T, NRows, 1>;

    static Mat doit(Json::Value const& json)
    {
        const int nelems = json.size();
        Mat mat;
        if constexpr (NRows < 0)
        {
            mat.resize(nelems, 1);
        }
        else
        {
            if (NRows != nelems)
                throw_exception<Json::ConvertationError>("Going to parse ", NRows, "-sized vector, but json has ", nelems, " elements");
        }

        for (int i = 0; i < nelems; ++ i)
        {
            if (json[i].isArray())
            {
                if (json[i].size() != 1)
                    throw_exception<Json::ConvertationError>("Exected 1-element list, but it has ", json[i].size(), " elements");

                mat(i) = Converter<T>::doit(json[i][0]);
            }
            else if (json[i].isNumeric())
            {
                mat(i) = Converter<T>::doit(json[i]);
            }
        }
        
        return mat;
    }
};

template <typename T, int NCols>
struct Converter<Eigen::Matrix<T, 1, NCols>>
{
    using Mat = Eigen::Matrix<T, 1, NCols>;

    static Mat doit(Json::Value const& json)
    {
        const int nelems = json.size();
        Mat mat;
        if constexpr (NCols < 0)
        {
            mat.resize(nelems, 1);
        }
        else
        {
            if (NCols != nelems)
                throw_exception<Json::ConvertationError>("Going to parse ", NCols, "-sized vector, but json has ", nelems, " elements");
        }

        for (int i = 0; i < nelems; ++ i)
            mat(i) = Converter<T>::doit(json[i]);

        return mat;
    }
};

template <typename T, int Dim>
struct Converter<Spline<T, Dim>>
{
    using Sp = Spline<T, Dim>;

    static Sp doit(Json::Value const& json)
    {
        std::vector<T> knots;
        json_get(json, "knots", knots);
        std::vector<typename Sp::ElemType> controls;
        json_get(json, "controls", controls);
        int degree;
        json_get(json, "degree", degree);
        return Sp(knots, controls, degree);
    }
};
