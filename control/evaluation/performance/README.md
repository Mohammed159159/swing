# Quantitative Performance Benchmarking (`control/evaluation/performance/`)

This directory contains automated scripts to compute standardized quantitative performance criteria across all controllers from simulated and experimental hardware data.

---

## 🎯 Quantitative Performance Indices

1. **Transient Response Specifications:**
   - Settling time $T_s$ ($2\%$ error envelope)
   - Rise time $T_r$ ($10\% \to 90\%$)
   - Percentage Overshoot: $\%OS = \frac{\theta_{1,\max} - \theta_{1,\text{target}}}{\theta_{1,\text{target}}} \times 100$
   - Maximum Peak Deviation: $|\theta_1|_{\max}$

2. **Error Integral Metrics:**
   - Integral of Absolute Error: $\text{IAE} = \int_0^T |e(t)| dt$
   - Integral of Squared Error: $\text{ISE} = \int_0^T e^2(t) dt$
   - Integral of Time-multiplied Absolute Error: $\text{ITAE} = \int_0^T t |e(t)| dt$

3. **Actuator Effort & Energy Expenditure:**
   - Peak control voltage: $u_{\text{peak}} = \max_t |u(t)|$
   - Root Mean Square Control Voltage:
     $$u_{\text{RMS}} = \sqrt{\frac{1}{T} \int_0^T u^2(t) dt}$$
   - Actuator Saturation Percentage:
     $$\%T_{\text{sat}} = \frac{T_{\text{saturated}}}{T_{\text{total}}} \times 100$$

4. **Comparative Summary Tables:**
   - Automated scripts compiling PID, Pole Placement, LQR, and RL metrics into formatted Markdown and LaTeX tables for the final technical report.
