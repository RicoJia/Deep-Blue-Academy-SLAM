<!-- To generate a pdf, do pandoc hw.md -o hw.pdf --pdf-engine=xelatex -->
<!-- 1: No empty lines 2. No begin{gather} -->
<!-- Solution: https://blog.csdn.net/Walking_roll/article/details/134310443 -->

# Homework 3

## [Question 1]

Prove:

$$
\begin{gather*}
\begin{aligned}
& \frac{\partial r_{\Delta p_{ij}}}{\partial \boldsymbol{\phi}_i} =
\left( \mathbf{R}_i^\top \left( \mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t_{ij} - \frac{1}{2} \mathbf{g} \Delta t_{ij}^2 \right) \right)^\wedge.
\end{aligned}
\end{gather*}
$$

### Proof

$$
\begin{gather*}
\begin{aligned}
& r_{\Delta p_{ij}} := R_i^{\top} \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2} g \Delta t_{ij}^2 \right) - \Delta \tilde{p}_{ij}.

\\ &
\Rightarrow \text{Applying right perturbation on } R_i

\\ &
r_{\Delta p_{ij}}(R_i^{\top} Exp(\phi)) := (R_i Exp(\phi))^{\top} \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2} g \Delta t_{ij}^2 \right) - \Delta \tilde{p}_{ij}.

\\ &
\approx (I - \delta \phi^{\land}) R_i^{\top}  \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2} g \Delta t_{ij}^2 \right) - \Delta \tilde{p}_{ij}.

\\ &
= (R_i^{\top}  \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2} g \Delta t_{ij}^2 \right) - \Delta \tilde{p}_{ij}) - \delta \phi^{\land} R_i^{\top} \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2} g \Delta t_{ij}^2 \right)

\\ &
= r_{\Delta p_{ij}} + [R_i^{\top} \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2} g \Delta t_{ij}^2 \right)]^{\land} \delta \phi

\\ &
\Rightarrow 

\\ &
\frac{\partial r_{\Delta p_{ij}}}{\partial \boldsymbol{\phi}_i} =
\left( \mathbf{R}_i^\top \left( \mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t_{ij} - \frac{1}{2} \mathbf{g} \Delta t_{ij}^2 \right) \right)^\wedge.

\end{aligned}
\end{gather*}
$$

## [Question 2]