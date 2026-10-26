# Franka Panda — Scientific & Technical Review (Milestones R0–R5)

**Documento de Diagnóstico, Revisión Matemática y Verificación Experimental**  
**Fecha:** 26 de octubre de 2026  
**Rama de trabajo:** `review/s1-scientific-polish`  
**Estado de referencia:** Git `main` (`3245186`), sincronizado con `origin/main`  
**Entorno de verificación:** MATLAB R2026a Update 4 (Windows 64-bit)

---

## Parte I — Diagnóstico Inicial (Hito R0)

### 1. Resumen Ejecutivo del Diagnóstico (Hito R0)

Tras la publicación de la Versión B (dinámica geométrica global sobre $(S^1)^7$) y la finalización de los hitos fundacionales H0–H7, se realizó una auditoría diagnóstica integral del repositorio.

El código es reproducible y todas las suites de validación (`validate_panda`, `validate_panda_s1`, `test_s1_geometry_limits`, `test_s1_simulation_consistency`, `test_source_geometry`, `test_simulation_inputs`) pasan al 100 % en MATLAB R2026a.

Se localizaron **cuatro problemas metodológicos/técnicos concretos** (A, B, C, D), incompatibilidades potenciales en el renderizado de LaTeX en GitHub Flavored Markdown (GFM), y matices en las afirmaciones científicas que se abordaron formalmente en la Parte II (Hito R1) y se corrigieron en la Parte III (Hito R2).

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

#### 1.2 Fallo de la Desigualdad de Arco Fuera de la Variedad ($x_i^\top x_i \ne 1$)

En $\mathbb{R}^2$, la condición $u_i^\top x_i \ge \cos h_i$ sin restringir $x_i^\top x_i = 1$ define un **semiplano afín** $\{x_i \in \mathbb{R}^2 : \|x_i\|_2 \cos\delta_i \ge \cos h_i\}$, lo que provoca defectos matemáticos para entradas mal normalizadas:
- En las 6 articulaciones con $2h_i > \pi$ ($\cos h_i < 0$), el vector nulo $x_1 = (0,0)^\top$ o el vector antipodal escalado $x_1 = -0.5\,u_1$ (que apunta a la mitad del tope mecánico prohibido $\delta_1 = \pi$) producían $u_1^\top x_1 \ge -0.9703$, devolviendo falsamente `in_limits(1) = true`.
- En la articulación 4 ($2h_4 < \pi \implies \cos h_4 = +0.0697 > 0$), un vector fuera del arco escalado por $\|x_4\|_2 = 2$ daba un falso positivo, mientras un vector en el borde con $\|x_4\|_2 = 0.5$ daba un falso negativo.

#### 1.3 Garantías Matemáticas de `box_diff` frente a Controlador e Integrador

- **Garantía cinemática de `box_diff` + `reference.m`:** Como $\mathcal{A}_i \subset S^1 \setminus \{-u_i\}$ es contráctil, la carta centrada $\delta_i(x_i) \in [-h_i, h_i]$ tiene su corte de rama en $-u_i \in S^1 \setminus \mathcal{A}_i$ (a $\ge 14.0^\circ$ de distancia de cualquier estado admisible). Toda interpolación escalar monótona $s(t) \in [0, 1]$ genera una trayectoria $X_{\text{ref}}(t) \in \mathcal{A}$ para todo $t$ sin cruzar jamás el tope prohibido, y el vector de error PD apunta a lo largo de la clase de homotopía correcta.
- **Ausencia de invariancia dinámica en `simulate.m`:** El conjunto $\mathcal{A} \times \mathbb{R}^7 \subset T(S^1)^7$ no es positivamente invariante bajo la ecuación de segundo orden integrada ante velocidades elevadas, pares saturados o perturbaciones externas. `simulate.m` monitoriza y registra márgenes en cada subpaso RK4-MK, pero no es un controlador de barrera de control (CBF) ni un modelo de impacto de contacto rígido.

### 2. Análisis Causal de los Resultados Energéticos

- **Origen del salto en la Versión A:** `MatrixExp6.m` divide por $\theta = |q_i|$; para evitar $0/0$ en $\theta = 0$ aplica `NearZero(norm(omgtheta)) < 1e-6`, truncando la rotación $R_i \to I_3$. Al arrancar de $q_i(0)=0$ y cruzar $10^{-6}\text{ rad}$ en $k=3$, la energía potencial $V(q)$ sufre un salto discontinuo de $5.6218 \times 10^{-7}\text{ J}$ que estanca el error independiente de $h$.
- **Atribución exacta:** La eliminación del salto de energía ($5.62\times 10^{-7}\text{ J} \to 1.00\times 10^{-10}\text{ J}$, $5,621\times$ a $h=1\text{ ms}$) proviene de evaluar $R_i$ de forma algebraica suave en $(\cos q_i, \sin q_i)$ sobre el eje unitario conocido $\omega_i$. Tanto `Variant B-alg` como `Version B` comparten esta ventaja. RK4-MK no es un integrador simpléctico; conserva la energía con orden $O(h^4)$.

### 3. Estructura Polinomial vs. Racional

- **Polinomiales en $(c_i, s_i)$:** Cinemática directa, jacobianos, adjuntos, energía, $M(X)$, $b(X,\omega)$ sin fricción $\tanh$, y RNEA de dinámica inversa.
- **Racionales en $(c_i, s_i)$ (no polinomiales):** Dinámica directa $\dot{\omega} = M(X)^{-1}(\tau - b)$, debido a la inversión de $M(X)$.
- **Hipótesis del Teorema 3.1 (`screw_exp.m`):** Requiere $\|\omega\|_2 = 1$ y $\omega^\top v = 0$. Para tornillos helicoidales ($\omega^\top v \ne 0$) aparece el término axial $(q-s)(\omega^\top v)\omega$, que requiere elegir una carta $q(x)$.

---

## Parte III — Implementación y Verificación de Correcciones (Hito R2)

**Responsable:** Gemini 3.8 Flash (High)  
**Fecha:** 26 de octubre de 2026

### 1. Correcciones de Código Implementadas

1. **`+panda_s1/check_limits.m` (Validación de Variedad y Arco Desacoplados):**
   - Ahora calcula explícitamente $e_{\text{norm},i} = |c_i^2 + s_i^2 - 1|$ y evalúa `on_manifold = (unit_norm_err <= tol_norm)`.
   - Normaliza radialmente la dirección $\hat{x}_i = x_i / \|x_i\|_2$ (con protección $\hat{x}_i = -\infty$ si $\|x_i\|_2 \le \text{eps}$).
   - Evalúa `in_arc` sobre la dirección unitaria normalizada: $u_i^\top \hat{x}_i \ge \cos h_i - \text{tol}$ y $\mu_i \ge -\text{tol}$.
   - Define la admisibilidad combinada estricta: `in_limits = on_manifold & in_arc`.
   - Expone en `report` los campos diagnósticos: `report.in_limits`, `report.on_manifold`, `report.in_arc`, `report.unit_norm_error`, `report.u_dot_x_normalized`.
2. **`+panda_s1/config_matrix.m`:**
   - Detecta y rechaza con error explícito `panda_s1:config_matrix_degenerate` cualquier columna degenerada con norma nula cuando se especifica `tol`.
3. **`+panda_s1/simulate.m` y `+panda_s1/box_diff.m` (Docstrings Precisados):**
   - Se ajustaron los encabezados documentales para delimitar que `box_diff` garantiza la clase de homotopía cinemática admisible de la referencia y del error PD, mientras `simulate.m` monitoriza márgenes de etapa sin asumir invariancia dinámica (viabilidad) ni contacto rígido.
4. **`setup_panda.m`:**
   - Se añadieron las carpetas `tests/` y `tools/` al path de MATLAB para permitir ejecución directa e inmediata de tests y benchmarks desde la raíz.

### 2. Correcciones en Tests de Regresión

1. **`tests/test_s1_geometry_limits.m` (Nueva Sección 3b de Pruebas Fuera de $S^1$):**
   - Comprobación (a): Vector nulo $X = 0 \implies$ `in_limits = false`, `on_manifold = false`, `in_arc = false`.
   - Comprobación (b): Vector antipodal escalado $x_1 = -0.5 u_1$ en el centro del tope prohibido $\delta_1 = \pi \implies$ `in_limits(1) = false`, `on_manifold(1) = false`, `in_arc(1) = false`.
   - Comprobación (c): Ángulo válido en el centro pero norma $\lambda = 0.5$ y $\lambda = 2.0 \implies$ `in_limits = false`, `on_manifold = false`, `in_arc = true`.
   - Comprobación (d): Ángulo fuera de límites con norma $\lambda = 2.0 \implies$ `in_limits = false`, `on_manifold = false`, `in_arc = false`.
   - Comprobación (e): Error explícito `panda_s1:config_matrix_degenerate` al pasar columnas nulas a `config_matrix`.
2. **`tests/test_s1_simulation_consistency.m` (Desacoplamiento Energético):**
   - Se reemplazó la aserción acoplada `nom_energy_err_a / nom_energy_err_b > 5000` por cotas intrínsecas absolutas sobre la Versión B:
     `assert(nom_energy_err_b < 5e-10)` y orden empírico `nom_energy_err_b / nom_energy_err_b_fine > 14`.
   - La comprobación del ratio frente a la Versión A se condicionó a `if nom_energy_err_a > 1e-7` para documentar la eliminación del artefacto `NearZero` sin romper los tests si la Versión A es optimizada en el futuro.
3. **Resultado de la Suite de Tests:**
   - `validate_panda`: **PASS**
   - `validate_panda_s1`: **PASS**
   - `test_s1_geometry_limits`: **PASS**
   - `test_s1_simulation_consistency`: **PASS**
   - `test_source_geometry`: **PASS**
   - `test_simulation_inputs`: **PASS**

### 3. Muestreo Físicamente Admisible y Resultados del Benchmark

En `tools/benchmark_s1_comparison.m`:
- Se implementó muestreo normal truncado por rechazo en $DQ$ y $TAU$, garantizando que el 100% de los $N = 100$ estados pertenecen estrictamente al sobre operativo admisible:
  - Posición: $Q \in [q_{\min} + 0.08, q_{\max} - 0.08]$ rad ($\max |q_i - m_i|/h_i = 0.9686 \le 1.0$).
  - Velocidad: $\max |\dot{q}_i| / \dot{q}_{\max,i} = 0.9459 \le 0.95$ (**0 violaciones físicas**, frente a las 4 violaciones anteriores).
  - Par articular: $\max |\tau_i| / \tau_{\max,i} = 0.9277 \le 0.95$ (**0 violaciones físicas**).
  - Aceleración: $\max |\ddot{q}_i| = 7.116\text{ rad/s}^2 \le 10.0\text{ rad/s}^2$.
- Se añadieron aserciones formales de verificación de sobre físico (`assert(all(abs(DQ) <= m_a.dqmax, 'all'))`, etc.).
- Se actualizó la persistencia en `results/benchmark_s1_comparison.json`, `.mat`, `.png` y se unificó en `docs/assets/benchmark_s1_comparison.png`.

#### Tabla Comparativa Actualizada (100 Estados Físicamente Admisibles)

| Operación | Stock MR | Versión A ($q \in \mathbb{R}^7$) | Variante B-alg ($q + S^1$) | Versión B ($X \in (S^1)^7$) | Speedup B vs Stock | Speedup B vs A | Speedup B vs B-alg |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Cinemática Directa (`fk`)** | $42.13\,\mu\text{s}$ | $42.24\,\mu\text{s}$ | $14.92\,\mu\text{s}$ | **$13.29\,\mu\text{s}$** | **$3.17\times$** | **$3.18\times$** | **$1.12\times$** |
| **Link Frames (`frames`)** | $49.28\,\mu\text{s}$ | $52.49\,\mu\text{s}$ | $9.96\,\mu\text{s}$ | **$9.08\,\mu\text{s}$** | **$5.43\times$** | **$5.78\times$** | **$1.10\times$** |
| **Space Jacobian ($J_s$)** | $48.10\,\mu\text{s}$ | $49.01\,\mu\text{s}$ | $25.89\,\mu\text{s}$ | **$25.24\,\mu\text{s}$** | **$1.91\times$** | **$1.94\times$** | **$1.03\times$** |
| **Body Jacobian ($J_b$)** | $48.07\,\mu\text{s}$ | $96.25\,\mu\text{s}$ | $33.11\,\mu\text{s}$ | **$31.73\,\mu\text{s}$** | **$1.52\times$** | **$3.03\times$** | **$1.04\times$** |
| **Dinámica Inversa (`id`, RNEA)** | $162.38\,\mu\text{s}$ | $105.27\,\mu\text{s}$ | $51.45\,\mu\text{s}$ | **$49.07\,\mu\text{s}$** | **$3.31\times$** | **$2.15\times$** | **$1.05\times$** |
| **Dinámica ($M$ y $b$)** | $1271.50\,\mu\text{s}$ | $178.47\,\mu\text{s}$ | $124.34\,\mu\text{s}$ | **$124.99\,\mu\text{s}$** | **$10.17\times$** | **$1.43\times$** | **$0.99\times$** |
| **Dinámica Directa (`fd`)** | $1297.35\,\mu\text{s}$ | $186.53\,\mu\text{s}$ | $127.72\,\mu\text{s}$ | **$124.53\,\mu\text{s}$** | **$10.42\times$** | **$1.50\times$** | **$1.03\times$** |

#### Simulación Completa de Trayectoria (3.6 s, 3,600 subpasos RK4)

| Métrica de Simulación | Versión A ($q$, PoE) | Variante B-alg ($q + S^1$) | Versión B ($(S^1)^7$, RK4-MK) |
|:---|:---:|:---:|:---:|
| **Tiempo de Simulación (Wall-clock)** | $3067.88\text{ ms}$ | **$2091.79\text{ ms}$** | **$2158.82\text{ ms}$** |
| **Latencia por Subpaso** | $852.19\,\mu\text{s}$ | **$581.05\,\mu\text{s}$** | **$599.67\,\mu\text{s}$** |
| **Factor de Tiempo Real (RTF)** | $1.17\times$ | **$1.72\times$** | **$1.67\times$** |
| **Residuo de Balance Energético** | $5.6218 \times 10^{-7}\text{ J}$ | **$9.9790 \times 10^{-11}\text{ J}$** | **$1.0000 \times 10^{-10}\text{ J}$** |
| **Paridad de Trayectoria vs. B** | $9.7797 \times 10^{-10}\text{ rad}$ | **$4.4409 \times 10^{-15}\text{ rad}$** | $0$ (Referencia) |
| **Error Máximo de Norma en $S^1$** | N/A | N/A | **$2.2204 \times 10^{-16}$ ($1\,\text{ulp}$)** |
