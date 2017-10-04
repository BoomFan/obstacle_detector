#include "vector"
#include "cmath"
#include "algorithm"
#include "pred.h"

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


vector<PointPred> predict(PointPred pos, PointPred velocity, float init_rad, float x_step, float y_step, float cone, float t){
        vector<PointPred> res = vector<PointPred>();
        PointPred vec_sign = PointPred(velocity.x / abs(velocity.x), velocity.y / abs(velocity.y));
        PointPred min = add(pos, PointPred(init_rad * -vec_sign.x, init_rad * -vec_sign.y));
        int x = 0;
        int y = 0;
        if (velocity.y < 0){
            y_step = abs(y_step) * -1;
        }
        else{
            y_step = abs(y_step);
        }
        if (velocity.x < 0){
            x_step = abs(x_step) * -1;
        }
        else{
            x_step = abs(x_step);
        }

        PointPred test = add(min, PointPred(x * x_step, y * y_step));
        while  (norm(add(mult(pos, -1), test)) < max(init_rad, t * norm(velocity))){
            

            while(norm(add(mult(pos, -1), test)) < max(init_rad, t * norm(velocity))) {
                if ((norm_inner_product(add(test, mult(pos, -1)), velocity) > cone) || (norm(add(test, mult(pos, -1))) < init_rad)) {
                    res.push_back(test);
                }
                y += 1;
                test = add(min, PointPred(x * x_step, y * y_step));
            }
            x += 1;
            y = 0;
            test = add(min, PointPred(x * x_step, y * y_step));

        }
        return res;

}

