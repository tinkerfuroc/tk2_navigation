#ifndef __TINKER_HUMANTRACK_UTILITIES_H__
#define __TINKER_HUMANTRACK_UTILITIES_H__

namespace tinker {
namespace navigation {

sturct Point {
    double x;
    double y;
};

double Distance(const Point & point1, const Point & point2);

double OriginIsSameSide(const Point & start, const Point & end, const Point & point);

double GetInscribeAngle(const Point & start, const Point & end, const Point & point);

}
}

#endif

