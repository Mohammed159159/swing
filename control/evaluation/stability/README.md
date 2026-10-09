# Stability, Controllability & Margin Proofs (`control/evaluation/stability/`)

This directory contains automated analytical validation scripts for proving system stability, controllability, observability, and robust loop margins.

---

## 🎯 Mathematical Verifications Required

1. **Open-Loop Characteristic Equation & Eigenvalues:**
   - Solve $\det(\lambda I - A) = 0$.
   - Confirm presence of one strictly positive real pole ($\lambda_1 > 0$), proving open-loop instability at the apex.

2. **Controllability Proof:**
   - Controllability matrix:
     $$\mathcal{C} = \begin{bmatrix} B & AB & A^2 B & A^3 B \end{bmatrix} \in \mathbb{R}^{4 \times 4}$$
   - Script confirms $\operatorname{rank}(\mathcal{C}) = 4$ and $\det(\mathcal{C}) \ne 0$.

3. **Observability Proof:**
   - Observability matrix with position sensing only ($C = [I_2, 0_{2\times 2}]$):
     $$\mathcal{O} = \begin{bmatrix} C \\ CA \\ CA^2 \\ CA^3 \end{bmatrix} \in \mathbb{R}^{8 \times 4}$$
   - Script confirms $\operatorname{rank}(\mathcal{O}) = 4$.

4. **Closed-Loop Frequency-Domain Robustness:**
   - Return difference and loop transfer functions: $L(s) = K (sI - A)^{-1} B$.
   - Bode and Nyquist plots computing Gain Margin ($GM \ge 6\text{ dB}$ for LQR: $\infty$) and Phase Margin ($PM \ge 60^\circ$).
