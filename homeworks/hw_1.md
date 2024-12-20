<!-- To generate a pdf, do pandoc hw.md -o hw.pdf --pdf-engine=xelatex -->
## Exercises

### Find $\frac{\partial R^{-1}p}{\partial R}$ using left and right perturbations

Right Perturbation:

$$\begin{aligned}
& \frac{\partial R^{-1}p}{\partial R} = \lim_{\phi \rightarrow 0} \frac{(Rexp(\phi^{\land}))^{-1} p - R^{-1}p}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{exp(\phi^{\land})^{-1} R^{-1} p - R^{-1}p}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{exp(-\phi^{\land}) R^{-1} p - R^{-1}p}{\phi}\\
& \approx \lim_{\phi \rightarrow 0} \frac{(I - \phi^{\land}) R^{-1} p - R^{-1}p}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{- \phi^{\land} R^{-1} p}{\phi}
\\
& = \lim_{\phi \rightarrow 0} \frac{(R^{-1} p)^{\land} \phi}{\phi}
\\
& = (R^{-1} p)^{\land}\end{aligned}
$$

Left Perturbation:

$$
\begin{aligned}
& \frac{\partial R^{-1}p}{\partial R} = \lim_{\phi \rightarrow 0} \frac{(exp(\phi^{\land}) R)^{-1}p - R^{-1}p }{\phi}\\
& \frac{\partial R^{-1}p}{\partial R} = \lim_{\phi \rightarrow 0} \frac{R^{-1} exp(\phi^{\land})^{-1} p - R^{-1}p}{\phi}
\\
& \frac{\partial R^{-1}p}{\partial R} = \lim_{\phi \rightarrow 0} \frac{R^{-1} exp(-\phi^{\land}) p - R^{-1}p}{\phi}\\
& \frac{\partial R^{-1}p}{\partial R} \approx \lim_{\phi \rightarrow 0} \frac{R^{-1} (I - \phi^{\land}) p - R^{-1}p}{\phi}
\\
& \frac{\partial R^{-1}p}{\partial R} = \lim_{\phi \rightarrow 0} \frac{-R^{-1} \phi^{\land} p }{\phi}
\\
& \frac{\partial R^{-1}p}{\partial R} = \lim_{\phi \rightarrow 0} \frac{R^{-1} p^{\land} \phi }{\phi}\\
& = R^{-1} p^{\land}
\end{aligned}
$$

### Find $\frac{\partial R_1R_2^{-1}}{\partial R_2}$ using left and right perturbations

I'm not sure about $\lim_{\phi \rightarrow 0}\frac{-\phi^{\land}}{\phi}$

- Right Perturbation:

$$
\begin{aligned}
& \frac{\partial R_1R_2^{-1}}{\partial R_2} = \lim_{\phi \rightarrow 0} \frac{R_1(R_2 exp(\phi^{\land}))^{-1} - R_1R_2^{-1}}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{R_1exp(-\phi^{\land}) R_2 ^{-1} - R_1R_2^{-1}}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{R_1 R_2^T R_2exp(-\phi^{\land})R_2^{T} - R_1R_2^{T}}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{R_1 R_2^T exp(-R_2 \phi^{\land}) - R_1R_2^{T}}{\phi}
\\
& \approx \lim_{\phi \rightarrow 0} \frac{R_1 R_2^T (I -R_2 \phi^{\land}) - R_1R_2^{T}}{\phi}
\\
& = \lim_{\phi \rightarrow 0} \frac{-R_1  \phi^{\land}}{\phi}\\
& = -R_1
\end{aligned}
$$

Left Perturbation:

$$
\begin{aligned}
& \frac{\partial R_1R_2^{-1}}{\partial R_2} = \lim_{\phi \rightarrow 0} \frac{R_1 (exp(\phi^{\land}) R_2)^{-1} - R_1R_2^{-1}}{\phi}
\\
& = \lim_{\phi \rightarrow 0} \frac{R_1 R_2^{-1} exp(-\phi^{\land}) - R_1R_2^{-1}}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{R_1 R_2^{-1} (I -\phi^{\land}) - R_1R_2^{-1}}{\phi}\\
& = \lim_{\phi \rightarrow 0} \frac{-R_1 R_2^{-1} \phi^{\land}}{\phi}\\
& = -R_1 R_2^{-1}
\end{aligned}
$$

### Programming Exercise

```cpp
//
// Created by xiang on 22-12-29. Modified by Rico 2024-12-19
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 本节程序演示一个正在作圆周运动的车辆
/// 车辆的角速度与线速度可以在flags中设置

DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    double z_acc = -0.1;
    SE3 pose;                                                                    // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    const double dt = 0.05;                                                      // 每次更新的时间

    while (ui.ShouldQuit() == false) {
        // 更新自身位置
        Vec3d v_world = pose.so3() * v_body;
        pose.translation() += v_world * dt;

        // 更新自身旋转
        if (FLAGS_use_quaternion) {
            // theta is halved in the quaternion world
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            // Quatd q = pose.unit_quaternion() * Quatd(std::cos(0.5 * angular_velocity_rad * dt), 0, 0, std::sin(0.5 * angular_velocity_rad * dt));
            q.normalize();
            // auto& quat = q;
            // std::cout << "=========Quaternion coefficients: "
            //   << "w = " << quat.w() << ", "
            //   << "x = " << quat.x() << ", "
            //   << "y = " << quat.y() << ", "
            //   << "z = " << quat.z() << std::endl;
            pose.so3() = SO3(q);
        } else {
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }
        v_body += Vec3d(0, 0, z_acc * dt);
        LOG(INFO) << "pose: " << pose.translation().transpose();
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);
    }

    ui.Quit();
    return 0;
}
```

# Below is from my [blogpost](https://ricojia.github.io/2024/07/11/Robotics-Gauss-Newton-LM/)

## Gauss-Newton Optimization

In Gauss Newton, we specifically look at minimizing a least squares problem. Assume we have a:

- scalar-valued cost function $c(x)$,
- vector-valued function: $f(x)$, `[m, 1]`
- Jacobian $J_0$ at $x_0$ is consequently `[m, n]`
- Hessian $H$ is $D^2c(x)$. It's approximated as $J^T J$

$$
\begin{aligned}
& c(x) = |f(x)^2|
\\
& x* = argmin(|f(x)^2|)
\\
\text{First order Taylor Expansion:}
\\
& argmin_{\Delta x}(|f(x + \Delta x)^2|)
\\
&= argmin_{\Delta x}[(f(x_0) + J_0 \Delta x)^T (f(x_0) + J_0 \Delta x)]
\\
& = argmin_{\Delta x}[f(x_0)^T f(x_0) + f(x_0)^T J_0 \Delta x + (J_0 \Delta x)^T f(x_0) + (J_0 \Delta x)^T (J_0 \Delta x)]
\\
& = argmin_{\Delta x}[f(x_0)^T f(x_0) + 2 f(x_0)^T J_0 \Delta x + (J_0 \Delta x)^T (J_0 \Delta x)]
\end{aligned}
$$

Take the derivative of the above and set it to 0, we get

$$
\begin{aligned}
& \frac{\partial f(x + \Delta x)^2}{\partial \Delta x} = 2J_0^T f(x_0) + [(J_0^TJ_0) + (J_0^TJ_0)^T]\Delta x
\\
& = 2J_0^T f(x_0) + 2(J_0^TJ_0) \Delta x
\\
& = 0
\end{aligned}
$$

So we can solve for $\Delta x$ with $H = J_0^TJ_0$, $b = - J_0^T f(x_0)$:

$$
\begin{aligned}
& (J_0^TJ_0) \Delta x = - J_0^T f(x_0)
\\
& \rightarrow H \Delta x = g
\end{aligned}
$$

- Note: because $J_0$ may not have an inverse, here we cannot multiply $J_0^{-1}$ to eliminate $J_0^T$
- In fact, to $\Delta x$ is available if and only if $H$ is **positive definite**.
- In least square, $f(x)$ is a.k.a residuals. Usually, it represents the **error between a data point and from its ground truth**.

In SLAM, we always frame this least squares problem with `e = [observered_landmark - predicted_landmark]` at each landmark. So all together, we want to **minimize the total least squares of the difference between  observations and predictions.** In the meantime, at each landmark, there is an error covariance, so all together, there's an error matrix $\Sigma$. Here in cost calculation, we take $\Sigma^{-1}$ so the **larger the error covariance, the lower the weight the corresponding difference gets.**

With $e(x + \Delta x) \approx e(x) + J \Delta x$,

$$
\begin{aligned}
& x* = argmin(|e^T \Sigma^{-1} e|)
\\
& \rightarrow argmin_{\Delta x}(|e(x + \Delta x)^T \Sigma^{-1} e(x + \Delta x)|)
\\
& \text{similar steps as above ... }
\\
& \rightarrow (J_0^T \Sigma^{-1}J_0) \Delta x = - J_0^T \Sigma^{-1} f(x_0)
\end{aligned}
$$

Using Cholesky Decomposition, one can get $\Sigma^{-1} = A^T A$. Then we can write the above as

$$
\begin{aligned}
& ((AJ_0)^T (AJ_0)) \Delta x  = - (AJ_0)^T Af(x_0)
\end{aligned}
$$

For a more detailed derivation, [please see here](./2024-07-11-rgbd-slam-bundle-adjustment.markdown)

## Levenberg-Marquardt (LM) Optimization

Again, **Taylor expansion** works better when $\Delta x$ is small, so the function can be better estimated by it. So, similar to regularization techniques on step sizes in deep learning, like L1, L2 regularization, we can regularize the step size, $\Delta x$

$$
\begin{aligned}
& (H + \mu I_H) \Delta x = -J_0^Tf(x_0) = g
\end{aligned}
$$

Intuitively,

- as $\mu$ grows, the diagonal identity matrix $\mu I_H$ grows, so $H + \mu I_H \rightarrow \mu I_H$. So, $\Delta x \approx (H + \mu I_H)^{-1}g = \frac{g}{\mu}$, which means $\Delta x$ grows smaller. In the meantime, $\Delta x$ will be similar to that in gradient descent.
- as $\mu$ becomes smaller, $\Delta x$ will become more like Gauss-Newton. However, due to $\mu I_H$, $(H + \mu I_H)$ is positive semi-definite, which provides more stability for solving for $\Delta x$.
