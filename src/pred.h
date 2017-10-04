#include "vector"
#include "cmath"
#include "algorithm"

using namespace std;

struct PointPred{
    float x;
    float y;
    PointPred(float x_n, float y_n){
        x = x_n;
        y = y_n;
    }

};
PointPred mult(PointPred x, float lambda);

PointPred add(PointPred x_1, PointPred x_2);
float norm(PointPred x);


float norm_inner_product(PointPred x, PointPred y);

PointPred perp_unit_vector(PointPred x);

float min_dist_line (PointPred x, PointPred v_0, PointPred v_1);


vector<PointPred> predict(PointPred pos, PointPred velocity, float init_rad, float x_step, float y_step, float cone, float t);