#include "vector"
#include "cmath"
#include "algorithm"
#include "pred.h"
#include "iostream"

using namespace std;

PointPred mult(PointPred x, float lambda){
    return PointPred(x.x * lambda, x.y * lambda);
}

PointPred add(PointPred x_1, PointPred x_2){
    return PointPred(x_1.x + x_2.x, x_1.y + x_2.y);
}
float norm(PointPred x){
    return sqrt(x.x * x.x + x.y * x.y);
}


float norm_inner_product(PointPred x, PointPred y){
    if (norm(x) == 0 || norm(y) == 0){
        return 0;
    }
    return (x.x * y.x + x.y * y.y) / (norm(x) * norm(y));
}

PointPred perpendicular_unit_vector(PointPred x){
    if (norm(x) == 0){
        return PointPred(0,0);
    }
    return mult(PointPred(-x.y, x.x), 1/norm(x));
}


vector<PointPred> predict(PointPred pos, PointPred velocity, float min_radius, float max_radius, float x_step, float y_step, float t){
        vector<PointPred> res = vector<PointPred>();

        int x = -1 * (min_radius/x_step - 1);
        int y = 0;

        PointPred perp = perpendicular_unit_vector(velocity);
        PointPred unit_velocity = mult(velocity, 1/norm(velocity));

        float vn = norm(velocity);
        float steps = 20;
        float rad;
        bool flag;
        while (x * x_step <= vn * t + max_radius) {    
            while (y * y_step <= min_radius + max(0, x) * x_step/(vn * t) * (max_radius - min_radius)) {
                if (x * x_step + min_radius <= min_radius && y * y_step >  sqrt( min_radius*min_radius - (abs(x) * x_step)*(abs(x) * x_step) ) ){
                    break;
                } else if (x * x_step > vn * t && y * y_step > sqrt( max_radius*max_radius - (x * x_step - vn * t)*(x * x_step - vn * t) ) ){
                    break;
                }
                res.push_back(add(add(pos, mult(unit_velocity, x*x_step)), mult(perp, y * y_step)));
                res.push_back(add(add(pos, mult(unit_velocity, x*x_step)), mult(perp, -1 * y * y_step)));
                // if (x > 0){
                //     cout << "FIRST TERM " << max(0, x) << endl;
                //     cout << "SECOND TERM " << x_step << endl;
                //     cout << "THIRD TERM " << (vn * t) << endl;
                // }
                y += 1;
            }
            x += 1;
            y = 0;

        }
        return res;
}

int main(){
    PointPred velocity = PointPred(1,0);
    PointPred position = PointPred(0,0);


}
