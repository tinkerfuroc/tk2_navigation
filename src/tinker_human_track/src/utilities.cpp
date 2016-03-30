#include <tinker_human_track/utilities.h>
#include <math.h>

namespace tinker{
namespace navigation {

double Distance(const Point & point1, const Point & point2) {
    return sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
}

double OriginIsSameSide(const Point & start, const Point & end, const Point & point) {
    double origin_side = start.x * end.y - end.x * start.y;
    double point_side = (point.x - start.x) * (point.y - end.y) - (point.x - end.x) * (point.y - start.y);
    return origin_side * point_side > 0;
}

double GetInscribeAngle(const Point & start, const Point & end, const Point & point) {
    const Point origin = {0, 0};
    double x1 = point.x - start.x;
    double x2 = point.x - end.x;
    double y1 = point.y - start.y;
    double y2 = point.y - end.y;
    double dot_product = x1 * x2 + y1 * y2;
    return acos(dot_product / (Distance(start, origin) * Distance(end, origin)));
}

}
}

