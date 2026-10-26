# Franka Panda — Scientific & Technical Review (Milestones R0–R5)

**Documento de Diagnóstico y Revisión Matemática Científica**  
**Fecha:** 26 de octubre de 2026  
**Rama de trabajo:** `review/s1-scientific-polish`  
**Estado de referencia:** Git `main` (`3245186`), sincronizado con `origin/main`  
**Entorno de verificación:** MATLAB R2026a Update 4 (Windows 64-bit)

---

## Parte I — Diagnóstico Inicial (Hito R0)

### 1. Resumen Ejecutivo del Diagnóstico (Hito R0)

Tras la publicación de la Versión B (dinámica geométrica global sobre $(S^1)^7$) y la finalización de los hitos fundacionales H0–H7, se realizó una auditoría diagnóstica integral del repositorio.

El código es reproducible y todas las suites de validación (`validate_panda`, `validate_panda_s1`, `test_s1_geometry_limits`, `test_s1_simulation_consistency`, `test_source_geometry`, `test_simulation_inputs`) pasan al 100 % en MATLAB R2026a.

Se localizaron **cuatro problemas metodológicos/técnicos concretos** (A, B, C, D), incompatibilidades potenciales en el renderizado de LaTeX en GitHub Flavored Markdown (GFM), y matices en las afirmaciones científicas que se abordan formalmente en la Parte II (Hito R1).

### 2. Localización de Problemas (Hito R0)

- **Problema A (Monitorización de límites vs. garantía de seguridad):** `+panda_s1/simulate.m` (Líneas 7–8, 160–182), `docs/development/HANDOFF.md` (Línea 37), `README.md` (Líneas 89, 120, 183).
- **Problema B (Validación de norma unitaria en `check_limits.m`):** `+panda_s1/check_limits.m` (Líneas 25, 33–36, 40–43) y `+panda_s1/config_matrix.m` (Líneas 12–14, 31–38).
- **Problema C (Test energético acoplado a defecto del baseline):** `tests/test_s1_simulation_consistency.m` (Líneas 41–42).
- **Problema D (Muestreo de benchmarks con gaussianas no acotadas):** `tools/benchmark_s1_comparison.m` (Líneas 42–44).
- **Auditoría editorial del `README.md`:** Entorno `\begin{bmatrix} ... \end{bmatrix}` con `\\` dentro de tabla Markdown (Línea 30), código en backticks adyacente a bloques `$ ... $` en tablas (Líneas 32, 34), y ruta heterogénea `results/benchmark_s1_comparison.png` (Línea 73) frente a `docs/assets/`.

---

## Parte II — Revisión Matemática y Científica (Hito R1)

**Responsable:** Gemini 4 Argon  
**Objetivo:** Establecer con rigor matemático qué se mantiene, qué debe corregirse obligatoriamente en código/tests/benchmarks (Hito R2) y qué afirmaciones deben precisarse en la documentación pública (Hitos R3–R4).

---

### 1. Representación Geométrica de los Límites en $(S^1)^7$, Validación de Variedad y Operador `box_diff`

#### 1.1 Exactitud de la Caracterización del Arco Admisible sobre $(S^1)^7$

Para cada articulación $i \in \{1,\dots,7\}$ del Franka Emika Panda, el intervalo articular físico es $[q_{\min,i}, q_{\max,i}] \subset \mathbb{R}$ con punto medio $m_i = \frac{1}{2}(q_{\min,i} + q_{\max,i})$ y semiamplitud $h_i = \frac{1}{2}(q_{\max,i} - q_{\min,i})$. Los valores exactos del modelo son:

| Articulación $i$ | $q_{\min,i}$ [rad] | $q_{\max,i}$ [rad] | $m_i$ [rad] | $h_i$ [rad] | $2h_i$ [rad] | $\cos h_i$ | ¿$0 < h_i < \pi$? | ¿$2h_i > \pi$? |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| $1, 3, 5, 7$ | $-2.8973$ | $+2.8973$ | $0.0000$ | $2.8973$ | $5.7946$ | $-0.970309$ | **Sí** | **Sí** |
| $2$ | $-1.7628$ | $+1.7628$ | $0.0000$ | $1.7628$ | $3.5256$ | $-0.190987$ | **Sí** | **Sí** |
| $4$ | $-3.0718$ | $-0.0698$ | $-1.5708$ | $1.5010$ | $3.0020$ | $+0.069743$ | **Sí** | No |
| $6$ | $-0.0175$ | $+3.7525$ | $+1.8675$ | $1.8850$ | $3.7700$ | $-0.309060$ | **Sí** | **Sí** |

**Proposición 1.1 (Caracterización exacta sobre la variedad):**  
*Dado que $0 < h_i \le 2.8973 < \pi$ para las 7 articulaciones, la aplicación de recubrimiento $\pi_i: \mathbb{R} \to S^1$, $q_i \mapsto (\cos q_i, \sin q_i)^\top$, es inyectiva sobre el intervalo cerrado $[q_{\min,i}, q_{\max,i}]$. Su imagen $\mathcal{A}_i = \pi_i([q_{\min,i}, q_{\max,i}]) \subset S^1$ es un arco circular propio y simplemente conexo. Para cualquier vector $x_i \in \mathbb{R}^2$, se cumple que $x_i \in \mathcal{A}_i$ si y solo si se satisfacen conjuntamente:*
$$x_i^\top x_i = 1 \qquad \text{y} \qquad u_i^\top x_i \ge \cos h_i, \qquad \text{donde } u_i = \begin{bmatrix}\cos m_i \\ \sin m_i\end{bmatrix} \in S^1.$$

*Demostración:* Si $x_i^\top x_i = 1$, existe un único desplazamiento angular $\delta_i \in (-\pi, \pi]$ respecto al centro del arco $u_i$ tal que $x_i = R_z(\delta_i) u_i$, con $u_i^\top x_i = \cos\delta_i$ y $u_i \wedge x_i = u_{i,1} x_{i,2} - u_{i,2} x_{i,1} = \sin\delta_i$. Como $\cos|\delta_i|$ es estrictamente decreciente en $|\delta_i| \in [0, \pi]$ y $h_i \in (0, \pi)$, se tiene $\cos\delta_i \ge \cos h_i \iff |\delta_i| \le h_i \iff m_i + \delta_i \in [q_{\min,i}, q_{\max,i}]$. $\blacksquare$

#### 1.2 Fallo de la Desigualdad de Arco Fuera de la Variedad ($x_i^\top x_i \ne 1$)

En la implementación actual de `+panda_s1/check_limits.m` (líneas 25–43):
```matlab
[X, max_norm_err] = panda_s1.config_matrix(X);
...
u_dot_x = c .* cm + s .* sm;
in_limits = (u_dot_x >= (m.cos_half - tol));
...
delta = atan2(u_wedge_x, u_dot_x);
margins = m.qhalf - abs(delta);
```
En $\mathbb{R}^2$, la condición $u_i^\top x_i \ge \cos h_i$ sin restringir $x_i^\top x_i = 1$ define un **semiplano afín** $\{x_i \in \mathbb{R}^2 : \|x_i\|_2 \cos\delta_i \ge \cos h_i\}$, lo que provoca tres defectos matemáticos graves para entradas mal normalizadas:

1. **Falsos positivos en articulaciones con $2h_i > \pi$ ($\cos h_i < 0$, articulaciones 1, 2, 3, 5, 6, 7):**
   - El vector nulo $x_1 = (0,0)^\top \notin S^1$ produce $u_1^\top x_1 = 0 > \cos h_1 = -0.9703$, por lo que `check_limits` devuelve `in_limits(1) = true` y `margins(1) = 2.8973 rad` (debido a que `atan2(0,0) = 0`).
   - El vector antipodal escalado $x_1 = -0.5\,u_1$ (cuya dirección angular es $\delta_1 = \pi$, exactamente en la mitad del tope mecánico prohibido) produce $u_1^\top x_1 = -0.5 > \cos h_1 = -0.9703$, devolviendo **`in_limits(1) = true`** mientras `margins(1) = -0.2443 rad < 0`!
2. **Falsos positivos y falsos negativos en la articulación 4 ($2h_4 < \pi \implies \cos h_4 = +0.0697 > 0$):**
   - Un vector fuera de los límites con $\cos\delta_4 = 0.04 < \cos h_4$ pero escalado por $\|x_4\|_2 = 2$ da $u_4^\top x_4 = 0.08 > \cos h_4$ (`in_limits(4) = true`, `margins(4) < 0`).
   - Un vector con ángulo válido en el borde ($\cos\delta_4 = \cos h_4$) pero norma $\|x_4\|_2 = 0.5$ da $u_4^\top x_4 = 0.5\cos h_4 < \cos h_4$ (`in_limits(4) = false`, `margins(4) = 0`).
3. **Inconsistencia interna entre `in_limits` y `margins`:** `u_dot_x` es homogéneo de grado 1 en $\|x_i\|_2$, mientras que `atan2(u_wedge_x, u_dot_x)` es homogéneo de grado 0 en $\|x_i\|_2 > 0$.

#### 1.3 Corrección Matemática Obligatoria para `check_limits.m` (Hito R2)

Para que `+panda_s1/check_limits.m` sea matemáticamente rigurosa sobre $\mathbb{R}^{2\times 7}$:
1. Definir la desviación de variedad por articulación:
   $$r_i = \sqrt{c_i^2 + s_i^2}, \qquad e_{\text{norm},i} = |c_i^2 + s_i^2 - 1|.$$
2. Aceptar dos tolerancias explícitas (o un struct/parámetro opcional `tol_norm`, con valor por defecto `tol_norm = max(100 * eps, tol)` = `1e-10`):
   - **Predicado de pertenencia a la variedad $S^1$:**
     $$\text{on\_manifold}_i \iff (e_{\text{norm},i} \le \text{tol}_{\text{norm}}).$$
   - **Normalización segura para evaluación angular:** Si $r_i > \text{eps}$, definir $\hat{x}_i = x_i / r_i = (\hat{c}_i, \hat{s}_i)^\top \in S^1$. Si $r_i \le \text{eps}$ (entrada degenerada), fijar $u_i^\top \hat{x}_i = -\infty$, $\delta_i = \text{NaN}$, $\mu_i = -\infty$.
   - **Predicado de pertenencia al arco sobre la dirección normalizada:**
     $$\text{in\_arc}_i \iff (u_i^\top \hat{x}_i \ge \cos h_i - \text{tol}) \;\land\; (\mu_i \ge -\text{tol}).$$
   - **Predicado global de admisibilidad (`in_limits`):**
     $$\text{in\_limits}_i \iff \text{on\_manifold}_i \;\land\; \text{in\_arc}_i.$$
3. Mantener separados en `report`: `report.in_limits`, `report.on_manifold`, `report.in_arc`, `report.unit_norm_error` ($7\times 1$), `report.max_unit_norm_error` (escalar), `report.margins_rad` ($7\times 1$), `report.min_margin_rad`, y `report.max_limit_violation_rad`.
4. En `+panda_s1/config_matrix.m`, rechazar con error explícito cualquier columna degenerada con $c_i^2 + s_i^2 == 0$ cuando se invoque desde funciones cinemáticas o de carta (`to_q`, `box_diff`, `geodesic_diff`), o cuando se especifique `tol`.

#### 1.4 Garantías Matemáticas de `box_diff` frente a Controlador e Integrador

Es imprescindible distinguir tres objetos matemáticos distintos:
1. **Desplazamiento geodésico en $SO(2) \cong S^1$ (`geodesic_diff`):**
   $$\Delta\theta_{\text{geo},i}(x_{2,i}, x_{1,i}) = \operatorname{atan2}(x_{1,i}^\top J^\top x_{2,i},\; x_{1,i}^\top x_{2,i}) \in (-\pi, \pi].$$
   Mide el arco más corto sobre el círculo completo $S^1$. Como 6 de las 7 articulaciones del Panda tienen amplitud admisible $2h_i > \pi$, el arco prohibido $S^1 \setminus \mathcal{A}_i$ mide $2\pi - 2h_i < \pi$. Por tanto, para configuraciones admisibles cerca de extremos opuestos con $|q_{2,i} - q_{1,i}| > \pi$, la geodésica más corta en $S^1$ **atraviesa el tope físico prohibido** y tiene signo opuesto al desplazamiento físico permitido.
2. **Desplazamiento angular levantado en el arco admisible (`box_diff`):**
   Como $\mathcal{A}_i \subset S^1 \setminus \{-u_i\}$ es contráctil, la carta centrada $\delta_i: S^1 \setminus \{-u_i\} \to (-\pi, \pi)$,
   $$\delta_i(x_i) = \operatorname{atan2}(s_i \cos m_i - c_i \sin m_i,\; c_i \cos m_i + s_i \sin m_i),$$
   es un difeomorfismo real analítico sobre un entorno abierto de $\mathcal{A}_i$, cuyo corte de rama está en $-u_i$ (el punto medio del tope mecánico prohibido, a distancia angular $\pi - h_i \ge 0.2443\text{ rad} \approx 14.0^\circ$ de cualquier configuración admisible). El operador `box_diff` define:
   $$\Delta q_{\mathcal{A},i}(x_{2,i}, x_{1,i}) = \delta_i(x_{2,i}) - \delta_i(x_{1,i}) \in [-2h_i, 2h_i].$$
3. **Delimitación exacta de garantías:**
   - **Garantía matemática de `box_diff` y `reference.m` (Nivel Cinemático y de Referencia):**
     Si $X_0, X_{\text{goal}} \in \mathcal{A}$ y el escalar de fase $s(t)$ verifica $s(t) \in [0, 1]$ para todo $t$ (propiedad cumplida por el polinomio quíntico monótono de `ftv5.phase_plan`), entonces $\delta_i(X_{\text{ref}}(t)) = (1 - s(t))\delta_i(X_0) + s(t)\delta_i(X_{\text{goal}}) \in [-h_i, h_i]$ por convexidad de $[-h_i, h_i]$. En consecuencia, **la trayectoria de referencia $X_{\text{ref}}(t)$ permanece idénticamente dentro del conjunto admisible $\mathcal{A}$ para todo $t$ y jamás cruza el tope prohibido $S^1 \setminus \mathcal{A}$**. Asimismo, para cualesquiera $X_{\text{ref}}, X \in \mathcal{A}$, el error proporcional `box_diff(X_ref, X)` apunta siempre en la dirección admisible sobre $\mathcal{A}$.
   - **Ausencia de garantía de invariancia dinámica en `simulate.m` (Nivel Dinámico / Viabilidad):**
     El sistema mecánico de segundo orden $M(X)\dot{\omega} + b(X,\omega) = \tau_{\text{applied}} + J_b(X)^\top F_{\text{ext}}$ integrado por RK4-MK en `simulate.m` evoluciona sobre $T(S^1)^7 \cong (S^1)^7 \times \mathbb{R}^7$. El subconjunto $\mathcal{A} \times \mathbb{R}^7$ **no es positivamente invariante** para velocidades o perturbaciones arbitrarias: si el momento cinético hacia un tope supera la capacidad de frenado del par saturado $\tau_{\max,i}$, o si actúa una fuerza externa $F_{\text{ext}}$ suficientemente grande, la trayectoria ejecutada $X(t)$ cruzará la frontera $\partial\mathcal{A}$. El simulador `simulate.m` **monitoriza y cuantifica** exactamente cualquier violación en todas las etapas RK4 (`minimum_stage_limit_margin_rad`, `maximum_stage_limit_violation_rad`), pero no implementa fuerzas de contacto unilaterales ni filtros de barrera de control (CBF).

---

### 2. Análisis Causal de los Resultados Energéticos

En la trayectoria nominal de $3.6\text{ s}$ ($h = 1.0\text{ ms}$, $3,600$ subpasos RK4), los errores máximos de balance energético registrados en `results/benchmark_s1_comparison.json` son:
- **Versión A (`+panda/`, PoE con `MatrixExp6` + RK4 en $\mathbb{R}^{14}$):** $5.6218 \times 10^{-7}\text{ J}$ (estancado en $5.62 \times 10^{-7}\text{ J}$ para $h = 2, 1, 0.5, 0.25\text{ ms}$; orden empírico $\approx 0$).
- **Variante B-alg ($q \in \mathbb{R}^7 \mapsto (\cos q, \sin q)$ + kernels algebraicos $S^1$ + RK4 en $\mathbb{R}^{14}$):** $9.9790 \times 10^{-11}\text{ J}$ (convergencia $O(h^4)$ hasta $4.29 \times 10^{-13}\text{ J}$ a $h = 0.25\text{ ms}$).
- **Versión B (`+panda_s1/`, $X \in (S^1)^7$ + kernels algebraicos $S^1$ + RK4-MK en $(S^1)^7 \times \mathbb{R}^7$):** $1.0000 \times 10^{-10}\text{ J}$ (convergencia $O(h^4)$ hasta $2.44 \times 10^{-13}\text{ J}$ a $h = 0.25\text{ ms}$).

#### 2.1 Causa Exacta del Salto Energético en la Versión A (`NearZero` en `MatrixExp6`)

En `third_party/modern_robotics/MatrixExp6.m` (líneas 22–25):
```matlab
omgtheta = so3ToVec(se3mat(1: 3, 1: 3));
if NearZero(norm(omgtheta))
    T = [eye(3), se3mat(1: 3, 4); 0, 0, 0, 1];
```
Debido a que `MatrixExp6` recibe como única entrada la matriz acoplada `se3mat = VecTose3(S_i * q_i)`, pierde la separación entre el eje del tornillo $S_i = (\omega_i, v_i)$ y el ángulo $q_i$. Para recuperar el eje unitario divide por $\theta = \|\omega_i q_i\|_2 = |q_i|$ (`omgmat = se3mat(1:3,1:3) / theta`), introduciendo una singularidad numérica artificial $0/0$ en $q_i = 0$. Para proteger esa división, `NearZero` aplica un corte duro en $|q_i| < 10^{-6}\text{ rad}$, imponiendo $R_i = I_3$ en lugar de $I_3 + q_i[\omega_i]_\times + O(q_i^2)$, mientras conserva el término lineal $q_i v_i$ en la traslación.

En la trayectoria nominal, las articulaciones $1, 3$ y $5$ parten de $q_i(0) = 0$. Al cruzar el umbral $|q_i| = 10^{-6}\text{ rad}$ en el paso $k=3$, la matriz de rotación salta discontinuamente en $\Delta R_i \approx 10^{-6}[\omega_i]_\times$, produciendo un salto artificial de $5.6218 \times 10^{-7}\text{ J}$ en la energía potencial $V(q)$ que no se reduce al disminuir el paso de integración $h$.

#### 2.2 Atribución Científica Correcta de Cada Efecto

| Factor Técnico | Efecto Cuantitativo Verificado | ¿De qué depende realmente? |
|---|---|---|
| **1. Eliminación del corte `NearZero` ($5.62\times 10^{-7}\text{ J} \to 1.00\times 10^{-10}\text{ J}$, factor $5,621\times$ a $h=1\text{ ms}$)** | Restaura la suavidad $C^\infty$ en $q_i = 0$ y la convergencia $O(h^4)$ del integrador RK4. | Depende de evaluar la exponencial de tornillo y las rotaciones directamente en función de $(\cos q_i, \sin q_i)$ sobre el eje conocido $\omega_i$ sin multiplicar primero $S_i q_i$ y dividir por $|q_i|$. **Tanto `Variant B-alg` (en $\mathbb{R}^7$) como `Version B` (en $(S^1)^7$) consiguen esta mejora.** |
| **2. Fórmulas algebraicas especializadas y adjuntos factorizados ($B^{(i)} = \operatorname{Ad}_{M_i^{-1}}$)** | Aceleración de $\sim 90\%$ de la ganancia total de tiempo en FK, frames, jacobianos y RNEA. | Depende de explotar la estructura de articulación rotacional pura ($\|\omega_i\|=1, \omega_i^\top v_i = 0$) y el homomorfismo adjunto. Aplicable tanto a `Variant B-alg` como a `Version B`. |
| **3. Representación de estado sobre la variedad $(S^1)^7$ (`Version B` vs. `Variant B-alg`)** | Ganancia adicional de $5\%\text{--}12\%$ en llamadas directas a cinemática/RNEA (al ahorrar `cos(q), sin(q)`); sobrecoste de $4.4\%$ en simulación completa por las 4 llamadas a `exp_s1` por subpaso. | Propiedad intrínseca de almacenar y propagar el estado directamente como $X \in (S^1)^7 \subset \mathbb{R}^{2\times 7}$. |
| **4. Integración geométrica RK4-MK con renormalización de 1 ulp** | Error máximo de norma unitaria de $2.22 \times 10^{-16}$ frente a $7.87 \times 10^{-9}$ en RK4 ambiente; menor piso de redondeo energético a $h=0.25\text{ ms}$ ($2.44\times 10^{-13}\text{ J}$ vs. $4.29\times 10^{-13}\text{ J}$). | Propiedad del integrador de grupo de Lie Munthe–Kaas RK4-MK. **Nota:** RK4-MK **no es un integrador variacional simpléctico**; conserva el balance energético hasta el orden de truncamiento $O(h^4)$, no de forma exacta. |

---

### 3. Estructura Algebraica: Expresiones Polinomiales, Racionales y Transcendentes

#### 3.1 Clasificación Exacta de las Operaciones en `+panda_s1/`

La afirmación *"Purely polynomial in $(c_i, s_i)$"* debe matizarse distinguiendo con precisión entre cinemática/dinámica inversa y dinámica directa/integración:

1. **Operaciones puramente polinomiales en $X = (c_1, s_1, \dots, c_7, s_7) \in (S^1)^7$:**
   - *Exponencial de tornillo (`screw_exp.m` para paso nulo):* $\exp([S_i], x_i) = I_4 + s_i[S_i] + (1-c_i)[S_i]^2$ es un **polinomio afín (grado 1)** en $(c_i, s_i)$.
   - *Transformaciones inter-eslabón y adjuntos (`frames.m`, `rnea_shared_kinematics_S1.m`):* $T_{i-1,i}(x_i) = M_i \operatorname{diag}(R_z(x_i), 1)$ y $\operatorname{Ad}_{T_{i,i-1}}(x_i) = \operatorname{diag}(R_z(x_i)^\top, R_z(x_i)^\top) B^{(i)}$ son **polinomios lineales/afines** en $(c_i, s_i)$ (sobre $S^1$, $R_z(x_i)^{-1} = R_z(x_i)^\top$ es lineal en $(c_i, s_i)$).
   - *Cinemática directa (`fk.m`), Jacobianos (`jacobian.m`), Energía (`energy.m`), Matriz de Masa $M(X)$ y Dinámica Inversa sin fricción (`inverse_dynamics.m`, `dynamics.m`):* Son composiciones de sumas y productos de las matrices afines anteriores, por lo que son **funciones polinomiales multilineales** en $(c_1, s_1, \dots, c_7, s_7)$ (y cuadráticas/lineales en $\omega, \dot{\omega}$).
2. **Operaciones racionales suaves en $X$ (NO polinomiales):**
   - *Dinámica directa (`forward_dynamics.m` y `mk_rhs` en `simulate.m`):* La aceleración angular $\dot{\omega} = M(X)^{-1}(\tau - b(X,\omega))$ requiere resolver el sistema lineal con $M(X)$. Como $M(X)$ es polinomial en $(c_i, s_i)$ y uniformemente definida positiva ($\lambda_{\min}(M(X)) \ge 3.53 \times 10^{-3}\text{ kg}\cdot\text{m}^2 > 0$ en todo $(S^1)^7$), $M(X)^{-1} = \operatorname{adj}(M(X))/\det(M(X))$ es una **función racional real analítica globalmente bien definida** sobre $(S^1)^7$, pero no un polinomio.
3. **Operaciones no algebraicas (transcendentes):**
   - *Fricción articular (`+panda/friction.m`):* Incluye el término $\tanh(\omega_i / \omega_s)$, que es trascendente en la velocidad $\omega_i$.
   - *Mapas de carta y diferencias angulares (`to_q.m`, `box_diff.m`, `geodesic_diff.m`, `check_limits.m`):* Emplean $\operatorname{atan2}$ para recuperar ángulos en radianes.
   - *Acción exponencial del integrador (`exp_s1.m`):* Evalúa $\cos(\Delta\theta_i), \sin(\Delta\theta_i)$ sobre los incrementos de etapa $\Delta\theta_i = h b_j \omega_i$.

#### 3.2 Hipótesis Exactas y Alcance del Teorema 3.1 (`screw_exp.m`)

La identidad algebraica $\exp([S], x) = I_4 + s[S] + (1-c)[S]^2$ para $S = (\omega, v) \in \mathbb{R}^6$ y $x = (\cos q, \sin q)^\top \in S^1$ requiere **dos hipótesis geométricas exactas**:
1. **Eje rotacional unitario:** $\|\omega\|_2 = 1$, lo cual garantiza la identidad de Cayley–Hamilton $[\omega]_\times^3 = -[\omega]_\times$.
2. **Paso helicoidal nulo (articulación puramente rotacional):** $h_{\text{pitch}} = \omega^\top v = 0$, lo cual garantiza que $[\omega]_\times^2 v = (\omega\omega^\top - I_3)v = -v$ y por tanto $[S]^3 = -[S]$.

**Limitaciones para otros tornillos:**
- *Tornillos helicoidales ($\|\omega\|_2 = 1$, $\omega^\top v \ne 0$):* Como $[S]^3 + [S] = \begin{bmatrix} 0 & (\omega^\top v)\omega \\ 0 & 0 \end{bmatrix} \ne 0$, la traslación contiene el término secular $(q - \sin q)(\omega^\top v)\omega$. Este término **no es periódico en $q$** y por tanto no es una función univaluada sobre $S^1$ a menos que se fije una carta $q(x)$ en un arco simplemente conexo (implementado como fallback en `screw_exp.m`, líneas 41–50).
- *Articulaciones prismáticas ($\omega = 0$, $\|v\|_2 = 1$):* El grupo de configuración es $(\mathbb{R}, +)$ y no $S^1$; aquí $[S]^2 = 0$ y $\exp([S]q) = I_4 + q[S]$ es afín en $q \in \mathbb{R}$.

---

### 4. Diseño de Pruebas y Muestreo Físico de Benchmarks

#### 4.1 Desacoplamiento del Test Energético (`tests/test_s1_simulation_consistency.m`)

La aserción actual en la línea 41:
```matlab
assert(nom_energy_err_a / nom_energy_err_b > 5000, 'panda_s1:sim', ...
    'S^1 formulation must eliminate MatrixExp6 NearZero energy jump (>5000x improvement)');
```
convierte un defecto externo de `MatrixExp6` en una condición necesaria para que el test de `+panda_s1` pase.

**Criterio matemático aprobado para R2:**
- Un test de unidad/consistencia de `+panda_s1` debe verificar **propiedades intrínsecas absolutas** de `+panda_s1`:
  1. Cota absoluta del residuo energético a paso nominal ($h = 1\text{ ms}$): `assert(nom_energy_err_b < 5e-10)` (valor medido: $1.0000 \times 10^{-10}\text{ J}$).
  2. Orden empírico de convergencia de cuarto orden al dividir el paso por 2 ($h = 1\text{ ms} \to 0.5\text{ ms}$):
     $$p_{\text{emp}} = \log_2\!\left(\frac{\text{nom\_energy\_err\_b}}{\text{nom\_energy\_err\_b\_fine}}\right) > 3.8 \qquad (\text{valor medido: } 4.115).$$
  3. Invariancia de variedad a precisión de máquina: `assert(nom_max_unit_norm_err < 1e-15)`.
- El cociente `nom_energy_err_a / nom_energy_err_b` debe conservarse en el struct `report` como métrica comparativa frente al baseline congelado, pero sin bloquear la validez intrínseca de la Versión B.

#### 4.2 Verificación Empírica del Muestreo en `tools/benchmark_s1_comparison.m`

Al inspeccionar exactamente las muestras generadas por `tools/benchmark_s1_comparison.m` con la semilla fijada `rng(13102026, 'twister')` y $N = 100$:
- `Q`: muestreada uniformemente en $[q_{\min,i} + 0.08,\; q_{\max,i} - 0.08] \implies 100\%$ dentro de los límites de posición ($\max |q_i - m_i|/h_i = 0.9698 \le 1$).
- `DQ = randn(7, N) .* (0.35 * m_a.dqmax)`: alcanza **$\max_{i,k} |\dot{q}_{i,k}| / \dot{q}_{\max,i} = 1.223076$**, con **4 muestras que exceden el límite físico de velocidad $\dot{q}_{\max}$**.
- `TAU = randn(7, N) .* (0.30 * m_a.taumax)`: alcanza $\max_{i,k} |\tau_{i,k}| / \tau_{\max,i} = 0.998293$ (al borde de violar $\tau_{\max}$, y para $N \ge 150$ excede $\tau_{\max}$).

**Criterio de muestreo aprobado para R2:**
1. **Conjunto Principal Nominal (100% Físicamente Admisible):**
   Generar los $N = 100$ estados dentro del sobre físico admisible del robot:
   - Posición: $q_i \sim \mathcal{U}(q_{\min,i} + \text{margin},\; q_{\max,i} - \text{margin})$ con `margin = 0.08` rad.
   - Velocidad: normal truncada (mediante rechazo o acotamiento estricto) o uniforme en $[-0.90\,\dot{q}_{\max,i},\; +0.90\,\dot{q}_{\max,i}]$ (por ejemplo, muestreo normal con $\sigma_i = 0.35\,\dot{q}_{\max,i}$ rechazando/re-muestreando cualquier elemento con $|\dot{q}_i| > 0.95\,\dot{q}_{\max,i}$, lo que preserva la distribución realista sin colas fuera de límites).
   - Par: normal truncada por rechazo con $\sigma_i = 0.30\,\tau_{\max,i}$ restringida a $|\tau_i| \le 0.95\,\tau_{\max,i}$.
   - Aceleración: normal truncada por rechazo restringida a $|\ddot{q}_i| \le 10.0\text{ rad/s}^2$.
   - Incluir `assert` explícito de admisibilidad física para `Q`, `DQ` y `TAU`, y registrar los máximos de utilización en `results/benchmark_s1_comparison.json`.
2. **Verificación de Estrés Separada (en `test_s1_geometry_limits.m`):**
   Mantener pruebas explícitas para estados en los bordes exactos $q_{\min}, q_{\max}$, fuera de los límites ($q_{\min} - \epsilon, q_{\max} + \epsilon$) y fuera de la variedad $S^1$ ($\|x_i\|_2 \ne 1$).

---

### 5. Síntesis de Correcciones y Criterios de Aceptación para Hito R2

#### 5.1 Correcciones Obligatorias (a implementar en R2)
1. **`+panda_s1/check_limits.m` y `+panda_s1/config_matrix.m`:**
   - Validar simultáneamente la pertenencia a la variedad $S^1$ ($|x_i^\top x_i - 1| \le \text{tol}_{\text{norm}}$) y la desigualdad del arco sobre el vector normalizado ($u_i^\top \hat{x}_i \ge \cos h_i - \text{tol}$).
   - Garantizar que entradas fuera de $S^1$ (como $x_i = (0,0)^\top$, $x_i = -0.5 u_i$, o $x_i = 2 u_i$) devuelvan `in_limits(i) = false` y reporten por separado `on_manifold`, `in_arc`, `max_unit_norm_error` y `margins_rad`.
2. **`+panda_s1/simulate.m` y `+panda_s1/box_diff.m`:**
   - Ajustar los docstrings para precisar que `box_diff` garantiza la consistencia homotópica de la referencia y del error de control en el arco admisible $\mathcal{A}$, mientras que `simulate.m` monitoriza márgenes de etapa sin imponer barreras dinámicas (CBF) ni contacto rígido.
3. **`tests/test_s1_geometry_limits.m` y `tests/test_s1_simulation_consistency.m`:**
   - Añadir tests unitarios en `test_s1_geometry_limits.m` para vectores fuera de $S^1$ (nulos, escalados $\lambda = 0.5, 2.0$, antipodales escalados) verificando que `check_limits` los rechace y reporte el error de norma exacto.
   - Reemplazar en `test_s1_simulation_consistency.m` la aserción dura `nom_energy_err_a / nom_energy_err_b > 5000` por cotas intrínsecas sobre `nom_energy_err_b < 5e-10` y orden empírico `log2(nom_energy_err_b / nom_energy_err_b_fine) > 3.8`.
4. **`tools/benchmark_s1_comparison.m` y artefactos en `results/`:**
   - Aplicar muestreo normal truncado por rechazo dentro del sobre físico admisible ($|\dot{q}_i| \le 0.95\,\dot{q}_{\max,i}$, $|\tau_i| \le 0.95\,\tau_{\max,i}$), verificar con `assert` que 0 muestras exceden los límites físicos, re-ejecutar el benchmark y actualizar `results/benchmark_s1_comparison.json`, `.mat`, `.png` y `docs/assets/benchmark_s1_comparison.png`.

#### 5.2 Correcciones Opcionales
- Añadir en `setup_panda.m` la ruta `fullfile(root, 'tests')` y `fullfile(root, 'tools')` (o mantenerlas accesibles) para que cualquier test o benchmark pueda invocarse directamente tras `setup_panda` desde la raíz.

#### 5.3 Afirmaciones que Deben Matizarse en Documentación y README (en R3–R4)
1. **Energía:** Atribuir la mejora de $5,621\times$ a la eliminación del corte discontinuo `NearZero` ($10^{-6}$) de `MatrixExp6` mediante la formulación algebraica suave en $(c_i, s_i)$ (compartida por `Variant B-alg` y `Version B`), y aclarar que RK4-MK conserva la energía hasta $O(h^4)$ (no es un integrador simpléctico).
2. **Seguridad de límites:** Distinguir entre admisibilidad cinemática de la referencia (`box_diff` evita cruzar el tope prohibido) y monitorización de etapas en `simulate.m` (sin invariancia dinámica garantizada ante saturación o perturbaciones).
3. **Carácter polinomial:** Especificar que la cinemática directa, jacobianos, matriz de masa y dinámica inversa (sin fricción hiperbólica) son polinomiales en $(c_i, s_i)$, mientras que la dinámica directa es racional en $(c_i, s_i)$ por la inversión de $M(X)$.
4. **Velocidad computacional:** Destacar que $\sim 90\%$ de la aceleración frente a la Versión A proviene de la especialización algebraica (Teorema 3.1 y adjuntos factorizados, presentes en `Variant B-alg`), que `Version B` es un $5\%\text{--}12\%$ más rápida en llamadas aisladas que `Variant B-alg`, y que `Variant B-alg` es un $4.4\%$ más rápida en simulación completa debido al coste de `exp_s1` en las etapas RK4-MK.
