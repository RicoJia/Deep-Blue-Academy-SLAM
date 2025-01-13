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

We can write:

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