#include <iostream>
#include <cmath>

// Simple 3‑D vector
struct Vec3 {
    double x, y, z;
    Vec3(double x_=0, double y_=0, double z_=0) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& rhs) const { return Vec3(x+rhs.x, y+rhs.y, z+rhs.z); }
    Vec3 operator-(const Vec3& rhs) const { return Vec3(x-rhs.x, y-rhs.y, z-rhs.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator*(const Vec3& rhs) const { return Vec3(x*rhs.x, y*rhs.y, z*rhs.z); }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

// 3×3 matrix (diagonal inertia)
struct Mat3 {
    double m[3][3];
    Mat3() { for (int i=0;i<3;++i) for (int j=0;j<3;++j) m[i][j]=0.0; }
    Mat3(double diag) { for (int i=0;i<3;++i) for (int j=0;j<3;++j) m[i][j]=(i==j?diag:0.0); }
};

// Matrix‑vector product (unused after fix but kept for completeness)
Vec3 operator*(const Mat3& m,const Vec3& v){
    return Vec3(
        m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z,
        m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z,
        m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z);
}

// Spacecraft rigid‑body attitude dynamics
class Spacecraft{
public:
    Vec3 q, w; // attitude error (small‑angle) and angular velocity
    Mat3 inertia;
    Spacecraft():q(0,0,0),w(0,0,0){
        inertia.m[0][0]=100; inertia.m[1][1]=100; inertia.m[2][2]=50;
    }
    void update(const Vec3& torque,double dt=0.1){
        Vec3 alpha(
            torque.x / inertia.m[0][0],
            torque.y / inertia.m[1][1],
            torque.z / inertia.m[2][2]);
        w = w + alpha*dt;
        q = q + w*dt;
    }
};

// PID controller
class PIDController{
public:
    Vec3 kp,ki,kd,integral,prev_error;
    PIDController(double kp_=0.1,double ki_=0.01,double kd_=0.05):
        kp(kp_,kp_,kp_),ki(ki_,ki_,ki_),kd(kd_,kd_,kd_),integral(0,0,0),prev_error(0,0,0){}
    Vec3 compute(const Vec3& error,double dt=0.1){
        integral = integral + error*dt;
        const double maxI = 5.0;
        if(integral.norm()>maxI) integral = integral*(maxI/integral.norm());
        Vec3 derivative = (error - prev_error)*(1.0/dt);
        prev_error = error;
        return (kp*error)+(ki*integral)+(kd*derivative);
    }
};

int main(){
    Spacecraft sc;
    PIDController pid(1.2, 0.08, 0.2);
    const Vec3 target_q(0,0,0);

    for(int step=0; step<100; ++step){
        if(step==10) sc.q = Vec3(0.1,0.1,0.1); // disturbance
        Vec3 error = target_q - sc.q;
        Vec3 torque = pid.compute(error);
        sc.update(torque);

        std::cout << "Step " << step
                  << " | Error: " << error.norm()
                  << " | w: (" << sc.w.x << ", " << sc.w.y << ", " << sc.w.z << ")\n";
    }
    return 0;
}
