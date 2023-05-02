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
