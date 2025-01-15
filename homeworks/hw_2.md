<!-- To generate a pdf, do pandoc hw.md -o hw.pdf --pdf-engine=xelatex -->
<!-- Solution: https://blog.csdn.net/Walking_roll/article/details/134310443 -->

# Homework 2

## [Question 1]

If a Gaussian random variable vector $a \sim \mathcal{N(0, \Sigma)}$, where $Sigma = diag(b^2, ... b^2)$ (isotropic), then prove that with rotation matrix $R$, $E[Ra] = 0$, $cov[Ra] = \Sigma$

Proof:

For the mean, since rotation $R$ is a constant, we can take it out of the expectation calculation. So it's easy to show that

$$
\begin{gather*}
\begin{aligned}
& E[Ra] = R E[a] = 0
\end{aligned}
\end{gather*}
$$

For the covariance, we can use its definition and get

$$
\begin{gather*}
\begin{aligned}
& cov[Ra] = E[(Ra - E[Ra])(Ra - E[Ra])^T] 
\\ &
= E[(Ra - 0)(Ra - 0)^T]
\\ &
= E[Raa^TR^T]
\\ &
= RE[aa^T]R^T
\end{aligned}
\end{gather*}
$$

Then, since $E[aa^T] = \lambda I$, we can get:

$$
\begin{gather*}
\begin{aligned}
& RE[aa^T]R^T = \lambda R I R^T = \lambda I = \Sigma
\end{aligned}
\end{gather*}
$$

## [Question 2]

In the motion process code, decompose the $F$ matrix and implement the motion equations for each type of state variable separately. Please provide the formulas and an explanation of the code implementation.

After linearizing the error kinematics especially around rotational term, we can get its matrix form:

$$
\begin{gather*}
\begin{aligned}
& \delta p_{k+1} = \delta p_{k} + \delta v \Delta t

\\ &
\delta v_{k+1} = \delta v_{k} + (- R(\tilde{a} - b_a)^{\land}\delta \theta - R\delta b_a + \delta g) \Delta t - \eta_v
\\ &
(\delta \theta)_{k+1} \approx exp(-(\tilde{w} - b_{g}) \Delta t)\delta \theta - \delta b_g \Delta t - \eta_{\theta}
\\ &
\delta (b_g)_{k+1} = \delta (b_g)_{k} + \eta_{bg}
\\ &
\delta (b_a)_{k+1} = \delta (b_a)_{k} + \eta_{ba}
\\ &
\delta g_{k+1} = \delta g_{k}
\end{aligned}
\end{gather*}
$$

So,
$$
\begin{gather*}

\delta x_{k+1}^* = F \delta x_{k}

\\
\Rightarrow
\\

\begin{bmatrix}
\delta p_{k+1}*\\
\delta v_{k+1}* \\
\delta \theta_{k+1}*\\
\delta  b_{g, k+1}* \\
\delta  b_{a, k+1}*\\
\delta g_{k+1}* \\
\end{bmatrix}

=

\begin{bmatrix}
I & I\Delta t & 0 & 0 & 0 &0 \\
0 & I\Delta t & -R(\tilde{a}-b_a)^{\land}\Delta t & 0 & -R \Delta t & I\Delta t \\
0 & 0 & exp(-(\tilde{w} - b_{g}) \Delta t) & -I \Delta t & 0 & 0 \\
0 & 0 & 0 & I & 0 & 0 \\
0 & 0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & 0 & I \\
\end{bmatrix}

\begin{bmatrix}
\delta p_{k} \\
\delta v_{k} \\
\delta \theta_{k} \\
\delta  b_{g, k} \\
\delta  b_{a, k} \\
\delta g_{k} \\
\end{bmatrix}

\end{gather*}
$$

CODE: TODO

## [Question 3]

Use the left perturbation model to derive ESKF's kinematics and noise model. Also paste a code implementation.

Using:

$$
\begin{gather*}
\begin{aligned}
& b_{gt} = b_g + \delta b_g
\end{aligned}
\end{gather*}
$$

### Angular Error Derivative

We can write out the rotation matrix:

$$
\begin{gather*}
\begin{aligned}
\text{Using R' = Rw:}
\\
& R_t' := R_t (\tilde{w} - b_{gt} - \eta_g)^{\land}
\\ & 
\\
\text{Meanwhile using the left perturbation:}
\\
& R_t = \delta R R 
\\
& \rightarrow R_t' =  \delta R R' + \delta R' R


\\
\text{Using}: \delta R = exp(\delta \theta^{\land}):
\\ &
\delta R' = exp(\delta \theta^{\land})' = exp(\delta \theta^{\land}) (\delta \theta')^{\land}

\\
\text{Combine the two together}
\\
& R_t' = \delta R R (\tilde{w} - b_{gt} - \eta_g)^{\land} =  \delta R R' + \delta R' R

\\ &
\rightarrow exp(\delta \theta^{\land}) R (\tilde{w} - b_{gt} - \eta_g)^{\land} = exp(\delta \theta^{\land}) R' + exp(\delta \theta^{\land}) (\delta \theta')^{\land} R

\\
\text{Using}: R' = R [\tilde{w} - b_{g}]^{\land}
\\ &
\rightarrow R (\tilde{w} - b_{gt} - \eta_g)^{\land} = R [\tilde{w} - b_{g}]^{\land} + (\delta \theta')^{\land} R

\\ \text{Using}: \phi^{\land} R = R(R^T \phi)^{\land}:

\\ &
\rightarrow R [\tilde{w} - b_{gt} - \eta_g]^{\land} = R [\tilde{w} - b_{g}]^{\land} + R(R^T (\delta \theta'))^{\land}
\\ &
\rightarrow [\tilde{w} - b_{gt} - \eta_g] = [\tilde{w} - b_{g}] = R^T (\delta \theta')
\\ 
\text{Using lemma from question 1}: R \eta_g = \eta_g
\\ &
\rightarrow \delta \theta' = R(-\delta b_g) - \eta_g

\end{aligned}
\end{gather*}
$$

### Velocity Error Derivative

From the definition of true value error and estimate error, we know

$$
\begin{gather*}
\begin{aligned}
& v_t' = v' + \delta v' := R_t(\tilde{a} - b_{at} - \eta_a) + g_t

\\ &
\text{Meanwhile}: v' + \delta v' := R(\tilde{a} - b_a) + g + \delta v'
\end{aligned}
\end{gather*}
$$

Expanding $R_t = Exp(\delta \theta) R$ and $b_{at} = b_a + \delta b_a$ gives:

$$
\begin{gather*}
\begin{aligned}
& v_t' = Exp(\delta \theta) R (\tilde{a} - b_{a} - \delta b_a - \eta_a) + g_t

\\ \text{Using Taylor Expansion:}
\\ &
\approx (I + \delta \theta^{\land}) R (\tilde{a} - b_{a} - \delta b_a - \eta_a) + g_t

\\ \text{Ignoring small values}: - \delta \theta^{\land}(\delta b_a - \eta_a)
\\ &
\approx R (\tilde{a} - b_{a} - \delta b_a - \eta_a) + \delta \theta^{\land}R (\tilde{a} - b_{a}) + g_t
\end{aligned}
\end{gather*}
$$

To combine the above with the volocity error and using 
- $g_t = g + \delta g$
- $R\eta_a = \eta_a$
- $\delta \theta^{\land} m = -m^{\land} \delta \theta$

$$
\begin{gather*}
\begin{aligned}
& R(\tilde{a} - b_a) + g + \delta v' \approx R (\tilde{a} - b_{a} - \delta b_a - \eta_a) + \delta \theta^{\land}R (\tilde{a} - b_{a}) + g + \delta g

\\ &
\rightarrow 

\\ & \delta v' \approx R(- \delta b_a) - \eta_a + \delta g - [R(\tilde{a} - b_{a})]^{\land} \delta \theta
\end{aligned}
\end{gather*}
$$

The above is a linear form! 😊

So all together, 

$$
\begin{gather*}
\begin{aligned}
& \delta p' = \delta v
\\ &
\delta v' = R(- \delta b_a) - \eta_a + \delta g - [R(\tilde{a} - b_{a})]^{\land} \delta \theta
\\ & 
\delta \theta' = R(-\delta b_g) - \eta_g
\\ &
\delta b_g' = \eta_{bg}'
\\ &
\delta b_a' = \eta_{ba}'
\\ &
\delta g' = 0
\end{aligned}
\end{gather*}
$$

### Discrete Time Kinematics Model

$$
\begin{gather*}
\begin{aligned}
& \delta p_{k+1} = \delta p_{k} + \delta v \Delta t
\\ &
\delta v_{k+1} = \delta v_{k} + ( R(- \delta b_a) + \delta g - [R(\tilde{a} - b_{a})]^{\land} \delta \theta)\Delta t - \eta_v
\\ &
(\delta \theta)_{k+1} \approx \delta \theta - R\delta b_g \Delta t - \eta_{\theta}
\\ &
\delta (b_g)_{k+1} = \delta (b_g)_{k} + \eta_{bg}
\\ &
\delta (b_a)_{k+1} = \delta (b_a)_{k} + \eta_{ba}
\\ &
\delta g_{k+1} = \delta g_{k}
\end{aligned}
\end{gather*}
$$

So F becomes:

$$
\begin{gather*}
\begin{aligned}
& \begin{bmatrix}
I & I\Delta t & 0 & 0 & 0 &0 \\
0 & I\Delta t & -(R(\tilde{a}-b_a))^{\land}\Delta t & 0 & -R \Delta t & I\Delta t \\
0 & 0 & I & -R \Delta t & 0 & 0 \\
0 & 0 & 0 & I & 0 & 0 \\
0 & 0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & 0 & I \\
\end{bmatrix}
\end{aligned}
\end{gather*}
$$

Predictions of states are:

$$
\begin{gather*}
\begin{aligned}
& p_{k+1, pred} = p_k + v \Delta t + \frac{1}{2}(R(\tilde{a} - b_a)) \Delta t^2 + \frac{1}{2} g \Delta t^2
\\ &
v_{k+1, pred} = v_k + R(\tilde{a} - b_a) \Delta t + g \Delta t
\\ &
\theta_{k+1, pred} = Log(Exp((\tilde{w} - b_g) \Delta t)R_k)
\\ &
bg_{k+1, pred} = bg_k
\\ &
ba_{k+1, pred} = ba_k
\\ &
g_{k+1, pred} = g_k
\end{aligned}
\end{gather*}
$$

Predictions of errors are:

$$
\begin{gather*}
\begin{aligned}
& \delta x_{pred} = F \delta x
\\ & P_{pred} = FPF^T + Q
\text{where:}
\\ & Q = diag(TODO)
\end{aligned}
\end{gather*}
$$


#### GNSS Updates

While the above is the general update, a dual-RTK-GPS can have a simplified observation model.

First, a dual-RTK-GPS system can output: $y = [R_{GNSS}, P_{GNSS}]$
- $R_{GNSS}$: orientation observation of the robot
- $P_{GNSS}$: position observation of the robot

In general $y = h(x) \oplus v$, but here we think the same observation model also holds true for $\delta x$:

$$
\begin{gather*}
\begin{aligned}
& z_{\theta} = Log(I^T R_{GNSS})
\\ &
z_{\delta \theta} = Log(R_{GNSS}R^T)    \text{(Left purterbation)}
\end{aligned}
\end{gather*}
$$

See? TODO (I'm not sure the above is true)

This makes things easier, because this means our observation gives a direct observation of $\theta$. we can directly get: 

$$
\begin{gather*}
\begin{aligned}
& H_{\theta} = \frac{\partial h}{\partial \delta \theta} = I
\end{aligned}
\end{gather*}
$$

Same thing with position update:

$$
\begin{gather*}
\begin{aligned}
& H_{p} = \frac{\partial h}{\partial \delta p} = I
\end{aligned}
\end{gather*}
$$

In the meantime, innovation $y \ominus h(x_{pred}) = [\delta p, \delta \theta]$ and it is:

$$
\begin{gather*}
\begin{aligned}
& y \ominus h(x_{pred}) = [p_{GNSS} - p, Log(R^T R_{GNSS})]
\end{aligned}
\end{gather*}
$$

So: 

$$
\begin{gather*}
\begin{aligned}
& H = \begin{bmatrix}
I_3 & 0_3 & 0_3 & 0_3 & 0_3 & 0_3\\
0_3 & I_3 & 0_3 & 0_3 & 0_3 & 0_3
\end{bmatrix}
\end{aligned}
\end{gather*}
$$

Then the rest remains the same as EKF

$$
\begin{gather*}
\begin{aligned}
& K_{k+1} = P_{k+1}^{*} H_{k+1}^{T}(V^{-1} + H_{k+1} P_{k+1}^{*} H_{k+1}^T)

\\ &
P_{k+1} = P_{k+1}^{*} - K_{k+1} H P_{k+1}^{*}
\\ &
\delta x_{k+1} = K_{k+1} (z - h(x_{k+1}^{*}))
\end{aligned}
\end{gather*}
$$

#### Covariance Matrix Of Errors After Resetting

In discrete time, we approximate $p_{k+1}$ as the true value $p_t$ can define:

$$
\begin{gather*}
\begin{aligned}
& x_{k+1} = x_k \oplus \delta x_k
\\
\rightarrow
\\
& p_{k+1} = p_{k} + \delta p_{k}
\\ &
v_{k+1} = v_{k} + \delta v_{k}
\\ &
exp(\theta_{k+1}^{\land}) = exp(\delta \theta_{k}^{\land})exp(\theta_{k}^{\land})
\\ &
b_{g, k+1} = b_{g, k} + \delta b_{g, k}
\\ &
b_{a, k+1} = b_{a, k} + \delta b_{a, k}
\\ &
g_{k+1} = g_{k} + \delta g_{k}
\end{aligned}
\end{gather*}
$$

Since we have applied a correction, we can go ahead and reset $\delta x = 0$

**However, we recognize that this correction may not update with the best reset.** So, we need to adjust the error covariance before proceeding to the next step. We assume that after the reset $\delta x_{k}$, there's still an remeniscent error $\delta x^+$

The reset is to correct $x_{k+1} \sim \mathcal(\delta x, P_{k})$ to $x_{k+1} \sim \mathcal(0, P_{reset})$. **For vector space variables `p, v, b_a, b_g, g` this reset is a simple shift of distribution. The covariance matrices stay the same.** For rotation variables $\theta$ though, this shift of distribution is in the tanget space (which is a vector space). But projected on to the `SO(3)` manifold, the distribution is not only shifted, but also scaled. 

So, if we define:

- $\delta \theta^+$ is the error after reset. It is zero of course, but we are interested in the finding the Jacobian of that.

to find the new covariance matrix:

$$
\begin{gather*}
\begin{aligned}
& exp(\delta \theta^+ )exp(\delta \theta_k) R_k = exp(\delta \theta)R_k 
\\ \rightarrow
\\ & 
exp(\delta \theta) = exp(\delta \theta^+ )exp(\delta \theta_k)

\\ &
\rightarrow exp(\delta \theta^+ ) = exp(\delta \theta)exp(-\delta \theta_k )
\\&
\text{Using BCH:}
\theta^+ \approx -\delta \theta_k + \delta \theta - \frac{1}{2} \delta \theta^{\land} \delta \theta_k + o((\delta \theta_k )^2)

\\ &
= \theta^+ \approx -\delta \theta_k + \delta \theta + \frac{1}{2} \delta \theta_k ^{\land} \delta \theta+ o((\delta \theta_k )^2)

\\ &
\rightarrow \frac{\partial \theta^+}{\partial \delta \theta} = I+\frac{1}{2} \delta \theta_k^{\land}
\end{aligned}
\end{gather*}
$$

Then, the overall "Jacobian" for the covariance is:

$$
\begin{gather*}
\begin{aligned}
& J_k = [I_3, I_3, I+\frac{1}{2} \delta \theta_k^{\land}, I_3, I_3, I_3]
\end{aligned}
\end{gather*}
$$

So the covariance reset is:

$$
\begin{gather*}
\begin{aligned}
& P_{reset} = J_k P_{k+1} J_k
\end{aligned}
\end{gather*}
$$

**Usually, this is close enough to identity because the $\theta$ covariance is small**