#include "vector"
#include "cmath"
#include "algorithm"

using namespace std;

struct Point{
    float x;
    float y;
    Point(float x_n, float y_n){
        x = x_n;
        y = y_n;
    }

};
Point mult(Point x, float lambda){
    return Point(x.x * lambda, x.y * lambda);
}

Point add(Point x_1, Point x_2){
    return Point(x_1.x + x_2.x, x_1.y + x_2.y);
}
float norm(Point x){
    return sqrt(x.x * x.x + x.y * x.y);
}


float norm_inner_product(Point x, Point y){
    if (norm(x) == 0 || norm(y) == 0){
        return 0;
    }
    return (x.x * y.x + x.y * y.y) / (norm(x) * norm(y));
}

Point perp_unit_vector(Point x){
    Point x_n = Point(-1* x.y  / norm(x), x.x / norm(x));
    return x_n;
}

float min_dist_line (Point x, Point v_0, Point v_1){
    return abs((v_1.y - v_0.y)*x.x - (v_1.x - v_0.x) * x.y - v_1.x* v_0.y - v_1.y * v_0.x)  / norm(add(v_1, mult(v_0, -1)));
}


vector<Point> predict(Point pos, Point velocity, float init_rad, int xres, int yres, Point scene_min, Point scene_max, float t){
        vector<Point> res = vector<Point>();
        float cone = 0.9;

        for (int x = 0; x <  xres; x++){
            float pos_x = scene_min.x + (scene_max.x - scene_min.x)/float(xres) * x;
            for (int y = 0; y < yres; y++){
                float pos_y = scene_min.y + (scene_max.y - scene_min.y)/float(yres) * y;
                Point test = Point(pos_x, pos_y);
                Point perp = perp_unit_vector(velocity);
                if (((norm_inner_product(add(test, mult(pos, -1)), velocity) > cone) || 
                      min_dist_line(test, add(pos, mult(perp, init_rad)),  add(add(pos, mult(perp, init_rad)), velocity)) < init_rad||
                      min_dist_line(test, add(pos, mult(perp, init_rad)),  add(add(pos, mult(perp, -1* init_rad)), velocity)) < init_rad) 
                    && norm(add(test, mult(pos, -1))) < max(norm(velocity) * t, init_rad)) {
                    res.push_back(test);
                }
            }

        }
        return res;

}