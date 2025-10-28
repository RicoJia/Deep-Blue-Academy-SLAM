
## Chp7-1
- 协方差矩阵投影，实际上他对预测的影响不一定观察得到
- 很多时候 slam 人会根据自己数学的直觉去改进（点云应该密一点），但是很难对每一个数据集调参。
- LINS = LIO; VINS = VIO; 松耦合比较好维护，模块化， 工业界讲求实用，不讲求复杂，但是各模块有自己的glitch,比方说GPS 不来。当视觉纹理（比方说白墙）比较好的时候，LO运行好,但是这个纹理好坏比较难量化。这个又叫点云退化，比方说一面墙-这个时候满足了你所有约束，但是你的pose还有一个额外自由度，叫gauge freedom， 其中一个例子是visual slam 里的尺度不确定性,另一个例子就是当你的pose沿某方向移动（比方说长廊），点云配准依然成立。所以松耦合一般都会有退化检测，但这个又牵扯到灵敏度的问题。而因为我们累加pose estimate,整个系统的cov 一定是发散. Hessian's rank might be full, but some eigen values might be very close to zero. This is similar to ESKF's number of observable variables. In loosely coupled lio, each individual component might have gauge freedom that can't be cancelled out
- 紧耦合 all residuals are in the same problem. So the entire state estimate is determined by all modules. If certain modules degenerate, state estimate might still be valid. The Hessian might be better. IMU gives the distribution of pose, LO's residual shouldn't be too far from this distribution. Generally, if residuals from different modules are optimized in the same optimization step, they are "tightly coupled system", like Tightly coupled LIO, GINS, VIO. In GINS, when RTK is bad, tightly coupled GINS might perform better than the loosely coupled equivalent

- In Tightly coupled LIO, residuals NDT/ICP can be plugged into ESKF. However, since each time relative_pose has changed, point->mean difference or point->point correspondence would change. Then, the total residual would change too. So we need an iterative version of ESKF: iterated error state extended kalman filter (IESEKF, or just IEKF)
    - During each iteration, one still needs to solve for nearest neighbor
        - In theory, We need to project delta x onto the tangent space. But we don't need to (because we only choose the last update) and just do it at the end of the last iteration
        - K and P, and H are different matriceso

- So, we calculate \delta theta; Jacobian_theta -> `Jacobian _P -> Cov = J * P * J` -> K_k, `delta x_k, P_k+1`
- delta xk = argmin (|z - HK (xK + delta x)|^2 + |delta x_k|^2_{P_k}) : point cloud residuals + prior residuals

- Use Sherman-Morrison-woodbury equation, we don't need to calculate inverse of 10000x10000, instead just 18x18 inverses

## Relationship between IEKF and NDT 

As a refresher, we want to minimize sum(e_ij^T \Sig e_ij), so we want dx = sum(dx_ij) = sum (-H_ij^T b_ij). So H and b has to be:
    H = [H_11, H_12, ... H1N; H21, H_22, ...; ], B = [b_11, b_21, b_31 ...; b_12 ....]


In vanilla NDT,

- dx = [p, v, q, b_a, b_g, dg] (18x1)
- for each point: res r= Rq + t - mu; e = r \sig r^T; res is 3x1, R is x3
    - Jacobian: dr / dR = d(R q) / d dtheta = d(R q_x dtheta) / d dtheta =  -Rq^;  ?? Review
    - dr / dt = I -> J () = dr / [dr / dt, 0, dr / dR, 03, 03, 03]  (3 x 18)
    - The general jacobian is [dr / dt, dr / dt', dr / dR, dr / db_a, dr / db_g, dr / dg]  (3 x 18)

- Gauss newton was trying to do:
    sum (J_ij^T \Sig^-1 J_ij^T) \dx_ij  = - sum (J^T \Sig^-1 e_i)? 


In IEKF, dr = [r1, r2, ... rN] (3xN, 1). dx = [x,y,z,r, p, y]? J = [J_1; J_2; ... J_N]; H_ij = J_ij^T sig^-1 J_ij

dx = [dt, dtheta, dx1, dx2]? So J is in H[... J_j ... ]

delta x_k in eskf and NDT's delta x in gauss newton are related


## Tightly Coupled Lio

- TODO; 状态估计书，3.3.2??
- NDT 天生有误差，凡是LIO必然有重影。大部分slam 其实都是离线见图，除了圆盘扫地机器人。
- TODO：point - plane iEKF
- 一定要自己tune。很多建图效果其实都是靠调参弄出来的

