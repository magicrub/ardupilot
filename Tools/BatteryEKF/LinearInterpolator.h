#include <math.h>

template<typename T>
class LinearInterpolator {
public:
    LinearInterpolator(T* x_points, T* y_points, size_t num_points):
    _x_points(x_points),
    _y_points(y_points),
    _num_points(num_points)
    {}

    T eval(T x) {
        T a,b;
        get_a_b(x,a,b);
        return a*x+b;
    }

    T slope(T x) {
        T a,b;
        get_a_b(x,a,b);
        return a;
    }

    T definite_integral(T x0, T x1) {
        if (_num_points <= 1) {
            return NAN;
        }

        // ensure x0 is less than x1
        if (x0 > x1) {
            T temp = x1;
            x1 = x0;
            x0 = temp;
        }

        T ret = 0;
        T a,b;

        for(size_t i=1; i<_num_points; i++) {
            if (x1 <= _x_points[i]) {
                get_a_b(_x_points[i-1], _x_points[i], _y_points[i-1], _y_points[i], a, b);
                ret += integrate_line(a,b,x0,x1);
                return ret;
            }

            if (x0 <= _x_points[i]) {
                get_a_b(_x_points[i-1], _x_points[i], _y_points[i-1], _y_points[i], a, b);
                ret += integrate_line(a,b,x0,_x_points[i]);
                x0 = _x_points[i];
            }
        }
        get_a_b(_x_points[_num_points-2], _x_points[_num_points-1], _y_points[_num_points-2], _y_points[_num_points-1], a, b);
        ret += integrate_line(a,b,x0,x1);
        return ret;
    }

private:
    T integrate_line(T a, T b, T x1, T x2) {
        return -a*(x1*x1)/2 + a*(x2*x2)/2 - b*x1 + b*x2;
    }

    void get_a_b(T x1, T x2, T y1, T y2, T& a_ret, T& b_ret) {
        a_ret = (y2-y1)/(x2-x1);
        b_ret = y1-a_ret*x1;
    }

    void get_a_b(T x, T& a_ret, T& b_ret) {
        if (_num_points <= 1) {
            a_ret = NAN;
            b_ret = NAN;
            return;
        }

        for(size_t i=1; i<_num_points; i++) {
            if (x < _x_points[i]) {
                get_a_b(_x_points[i-1], _x_points[i], _y_points[i-1], _y_points[i], a_ret, b_ret);
                return;
            }
        }
        get_a_b(_x_points[_num_points-2], _x_points[_num_points-1], _y_points[_num_points-2], _y_points[_num_points-1], a_ret, b_ret);
        return;
    }


    T* _x_points;
    T* _y_points;
    size_t _num_points;
};
