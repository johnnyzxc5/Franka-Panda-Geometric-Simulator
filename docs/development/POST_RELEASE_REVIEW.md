# Franka Panda — Scientific & Technical Review (Milestones R0–R5)

**Documento de Diagnóstico y Plan de Acción Científica**  
**Fecha:** 26 de octubre de 2026  
**Rama de trabajo:** `review/s1-scientific-polish`  
**Estado de referencia:** Git `main` (`3245186`), sincronizado con `origin/main`  
**Entorno de verificación:** MATLAB R2026a Update 4 (Windows 64-bit)

---

## 1. Resumen Ejecutivo del Diagnóstico (Hito R0)

Tras la publicación de la Versión B (dinámica geométrica global sobre $(S^1)^7$) y la finalización de los hitos fundacionales H0–H7, se ha realizado una auditoría diagnóstica integral del repositorio. 

El código es reproducible, matemáticamente sólido y todas las suites de validación (`validate_panda`, `validate_panda_s1`, `test_s1_geometry_limits`, `test_s1_simulation_consistency`, `test_source_geometry`, `test_simulation_inputs`) pasan al 100 % en MATLAB R2026a.

No obstante, se han detectado **cuatro problemas metodológicos/técnicos concretos**, inconsistencias en el renderizado de LaTeX en GitHub Flavored Markdown (GFM), y matices en las afirmaciones científicas que requieren corrección en las fases R1 a R5 para alcanzar los más altos estándares de rigor editorial y científico.

---

## 2. Diagnóstico de Problemas Concretos

### Problema A: Monitorización de Límites vs. Garantía de Seguridad Dinámica (Viabilidad)

* **Ubicación:** 
  - `+panda_s1/simulate.m` (Líneas 7–8, 160–182).
  - `docs/development/HANDOFF.md` (Línea 37).
  - `README.md` (Líneas 89, 120, 183).
* **Diagnóstico Técnico:**
  1. La condición algebraica $u_i^T x_i \ge \cos h_i$ define exactamente el arco admisible $\mathcal{A}_i \subset S^1$.
  2. El operador `box_diff` garantiza que la trayectoria de referencia cinemática $X_{\text{ref}}(t)$ generada por `reference.m` y el vector de error PD apunten en la dirección de la clase de homotopía admisible sin cruzar el sector prohibido $S^1 \setminus \mathcal{A}_i$.
  3. Sin embargo, el integrador `simulate.m` integra las ecuaciones dinámicas continuas no restringidas:
     $$\dot{\omega} = M(X)^{-1} (\tau_{\text{applied}} + \tau_{\text{ext}} - b(X, \omega))$$
     No existe un proyector de viabilidad dinámica, ni una barrera de control activa (CBF), ni un modelo de restitución de contacto por impacto en los topes mecánicos.
  4. En presencia de pares de frenado insuficientes, saturación severa o perturbaciones externas, el estado $(X(t), \omega(t))$ puede abandonar el conjunto admisible $\mathcal{A}$. `simulate.m` monitoriza y registra el margen de etapa (`minimum_stage_limit_margin_rad`) y la violación (`maximum_stage_limit_violation_rad`), pero **no previene dinámicamente la violación**.
* **Impacto Documental:** Expresiones en comentarios y resúmenes como *"so that trajectories never traverse forbidden stops"* o *"navegación segura sin cruzar topes físicos"* confunden la consistencia cinemática/homotópica del planificador con una garantía de invariancia dinámica hacia adelante.
* **Acción para R1:** Rediseñar la documentación y docstrings para distinguir estrictamente entre:
  - *Invariancia cinemática de la referencia / operador de error* (garantizada por `box_diff`).
  - *Monitorización de márgenes de etapa* (realizada por `simulate.m`).
  - *Ausencia de barrera dinámica o física de contacto* (delimitada formalmente como limitación).

---

### Problema B: Validación de Norma Unitaria en Verificación de Límites

* **Ubicación:** `+panda_s1/check_limits.m` (Líneas 25, 33–36, 40–43).
* **Diagnóstico Técnico:**
  1. `check_limits.m` invoca `[X, max_norm_err] = panda_s1.config_matrix(X);` sin pasar el argumento `tol`. Por lo tanto, `config_matrix` no lanza error si las columnas de $X$ no tienen norma 1.
  2. La evaluación del límite de arco se realiza mediante:
     $$u_i^T x_i \ge \cos h_i - \text{tol}$$
  3. Si la entrada $X$ no pertenece estrictamente a $(S^1)^7$ (por ejemplo, si contiene vectores escalados $x_i = \lambda [\cos q_i; \sin q_i]$ con $\lambda \ne 1$ o puntos arbitrarios de $\mathbb{R}^2$):
     $$u_i^T x_i = \lambda \cos(\delta_i)$$
     - Si $\lambda > 1$, el producto escalar se amplifica artificialmente, pudiendo cumplir $u_i^T x_i \ge \cos h_i$ aun cuando el ángulo $\delta_i$ esté fuera del arco admisible.
     - Si $\lambda \le 0$ o $\lambda \ll 1$, el producto escalar se distorsiona arbitrariamente.
  4. Mientras tanto, el cálculo del ángulo $\delta_i = \operatorname{atan2}(u_i \times x_i, u_i^T x_i)$ es homogéneo de grado cero en $\|x_i\|$, lo que genera una inconsistencia interna: `in_limits` (lineal en $\|x_i\|$) puede diferir del signo de `margins` (invariante a escala).
* **Acción para R1:**
  - Actualizar `check_limits.m` para condicionar `in_limits` a que la norma unitaria se satisfaga simultáneamente:
    $$\text{in\_limits}_i = (u_i^T x_i \ge \cos h_i - \text{tol}) \;\land\; (|x_i^T x_i - 1| \le \text{tol}_{\text{norm}})$$
  - Alternativamente, normalizar radialmente las columnas en `check_limits` antes del producto escalar o rechazar entradas con error de norma excesivo mediante una opción configurable.

---

### Problema C: Test Energético Acoplado a Defecto de Librería Externa

* **Ubicación:** `tests/test_s1_simulation_consistency.m` (Líneas 41–42).
* **Diagnóstico Técnico:**
  1. El test contiene la aserción:
     ```matlab
     assert(nom_energy_err_a / nom_energy_err_b > 5000, 'panda_s1:sim', ...
         'S^1 formulation must eliminate MatrixExp6 NearZero energy jump (>5000x improvement)');
     ```
  2. Este ratio masivo ($5,621.86\times$) se debe exclusivamente a un artefacto de implementación de la función `MatrixExp6.m` de la librería externa `third_party/modern_robotics`: cuando $\|\omega \theta\| < 10^{-6}\text{ rad}$, la función descarta la rotación e impone $R = I_3$. En la trayectoria nominal, los ejes 1, 3 y 5 arrancan de reposo y cruzan $10^{-6}\text{ rad}$ en el paso $k=3$, introduciendo un salto repentino de energía de $5.62 \times 10^{-7}\text{ J}$.
  3. La formulación sobre $(S^1)^7$ de la Versión B evalúa las rotaciones de manera puramente algebraica y polinómica ($C^\infty$), arrojando un error intrínseco de $1.00 \times 10^{-10}\text{ J}$ que escala limpiamente como $O(h^4)$ con el refinamiento del paso.
  4. Si en el futuro se corrige `MatrixExp6.m` en la Versión A (por ejemplo, usando series de Taylor de bajo orden o $\epsilon_{\text{mach}}$), el error de la Versión A caería a $\sim 10^{-10}\text{ J}$, haciendo que el ratio relativo sea $\sim 1$ y provocando que el test de la Versión B **falle**, a pesar de que la Versión B permanezca inalterada y correcta.
* **Acción para R2:** Desacoplar la prueba de regresión de la Versión B:
  - Mantener la verificación de que el error propio de la Versión B está acotado por su cota teórica ($\text{err}_B < 10^{-9}\text{ J}$ a $h = 1\text{ ms}$).
  - Mantener la prueba de refinamiento de paso donde se comprueba la convergencia de orden $O(h^4)$ ($\text{err}_B(h) / \text{err}_B(h/2) > 14$).
  - Convertir el ratio relativo $> 5000$ en un indicador diagnóstico informativo o condicionarlo a la presencia del defecto de `MatrixExp6`.

---

### Problema D: Muestreo de Benchmarks con Distribuciones Gaussianas No Acotadas

* **Ubicación:** `tools/benchmark_s1_comparison.m` (Líneas 42–44).
* **Diagnóstico Técnico:**
  1. En el script de benchmark, las velocidades articulares y pares se generan como:
     ```matlab
     DQ = randn(7, N) .* (0.35 * m_a.dqmax);
     DDQ = randn(7, N) .* 2.0;
     TAU = randn(7, N) .* (0.30 * m_a.taumax);
     ```
  2. La distribución normal estándar $z \sim \mathcal{N}(0, 1)$ tiene soporte infinito $(-\infty, +\infty)$.
  3. Para $0.35 \dot{q}_{\max}$, cualquier muestra con $|z| > 1/0.35 \approx 2.857$ excede los límites físicos reales del manipulador ($P(|z| > 2.857) \approx 0.00428$). Con $N = 100$ a $250$ estados y 7 articulaciones ($700$ a $1,750$ variables aleatorias), la probabilidad de que varias muestras superen los límites físicos máximos de velocidad o par es superior al 95 %.
  4. Esto mezcla condiciones operativas físicamente admisibles con condiciones de sobre-esfuerzo / estrés extremo sin documentar la partición.
* **Acción para R3:**
  - Sustituir `randn` por distribuciones acotadas uniformes en el rango admisible (e.g. $U(-\alpha \dot{q}_{\max}, \alpha \dot{q}_{\max})$ con $\alpha = 0.85$ o truncamiento estricto a $\pm \dot{q}_{\max}$ y $\pm \tau_{\max}$).
  - Si se desea evaluar el comportamiento bajo sobre-esfuerzo, crear explícitamente dos conjuntos de benchmark: *Régimen Operativo Admisible* vs. *Régimen de Estrés Dinámico*.

---

## 3. Auditoría de Presentación, Markdown y LaTeX (README.md)

### 3.1 Entornos LaTeX dentro de Tablas Markdown (Incompatibilidad GFM)
* **Línea 30:** `J = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}` está contenido dentro de una celda de tabla. En GitHub Flavored Markdown, las barras invertidas dobles `\\` se interpretan con frecuencia como escape de caracteres, corrompiendo la fila de la tabla o produciendo fallos en el renderizador MathJax/KaTeX de GitHub.
* **Líneas 32 y 34:** Inserción de código en backticks dentro o adyacente a expresiones matemáticas delimitadas por `$`. Esto crea ambigüedad en el parser AST de GitHub.
* **Ecuaciones en Celdas:** Las expresiones matemáticas largas en la tabla de formulaciones (líneas 27–40) dificultan la lectura en pantallas estándar y móviles.
* **Recomendación para R4:** 
  - Simplificar las expresiones dentro de tablas (utilizar notación matricial compacta como $J = [0, -1; 1, 0]$).
  - Extraer las formulaciones matemáticas ricas (como el Teorema 3.1 del screw exponential y la factorización de adjuntos) a bloques de ecuaciones en display (`$$ ... $$`) fuera de las tablas.

### 3.2 Enlaces y Rutas de Recursos Multimedia
* **Líneas 15, 19:** Utilizan `docs/assets/panda_version_a.gif`, `docs/assets/panda_version_b.gif`, `docs/assets/panda_comparison_side_by_side.gif`. Todos los archivos existen en disco y están bajo control de versiones.
* **Línea 73:** `![Benchmark Comparison](results/benchmark_s1_comparison.png)`. Aunque el archivo existe en `results/`, rompe la consistencia estructural con los demás assets visuales.
* **Recomendación para R4:** Duplicar / mover la figura del benchmark a `docs/assets/benchmark_s1_comparison.png` para consolidar todas las imágenes públicas del repositorio en `docs/assets/`.

---

## 4. Auditoría de Justificación Científica de Afirmaciones

| Afirmación en Documentación / README | Evaluación Crítica | Justificación en Datos Guardados | Acción Requerida |
|---|---|---|---|
| **Aceleración $3.43\times$ en FK, $6.58\times$ en frames, $2.21\times$ en ID** | **Completamente Válida** | Mediana confirmada en `results/benchmark_s1_comparison.json`. | Mantener. Aclarar que el 90 % proviene del Teorema 3.1 y la factorización $B^{(i)}$. |
| **Mejora de $5,621\times$ a $2.3\times 10^6\times$ en balance de energía** | **Matemáticamente Medida, pero Atribuible a Defecto de Baseline** | Confirmada numéricamente frente a Versión A con `MatrixExp6`. Sin embargo, la brecha se debe al corte abrupto `NearZero` ($10^{-6}$) de Modern Robotics, no a una limitación intrínseca de usar coordenadas $q$. | Declarar explícitamente en el texto principal que esta ventaja comparativa mide la eliminación del defecto de singularidad/corte de la librería baseline. |
| **Preservación de norma unitaria a $2.22 \times 10^{-16}$** | **Completamente Válida** | Confirmada en `benchmark_s1_comparison.json` ($1\,\text{ulp}$). | Mantener. Aclarar la contribución de la normalización radial de 1 ulp frente al drift de $10^{-13}$ de RK4-MK puro. |
| **Garantía de navegación segura sin violar límites** | **Parcialmente Exagerada en Expresión** | Cierto para la interpolación de referencia y la orientación de error de `box_diff`. Falso como garantía dinámica global ante pares saturados o perturbaciones. | Matizar redacción: cambiar "garantiza que la trayectoria nunca cruza topes" por "garantiza que la referencia y la ley de control no comandan movimiento a través del sector prohibido". |
| **Simulación más rápida en Versión B ($1.41\times$)** | **Válida frente a A, pero B-alg es $1.48\times$** | Confirmada en `benchmark_s1_comparison.json`. La integración en $(S^1)^7$ añade un 4.4 % de overhead por las llamadas a `exp_s1` frente a integrar en $q$. | Mantener la transparencia en la comparación de 3 vías (A vs. B-alg vs. B). |

---

## 5. Hoja de Ruta para los Siguientes Hitos (R1 a R5)

* **HITO R1 — Corrección Geométrica y de Restricciones (Argon / Astra):**
  - Actualizar `+panda_s1/check_limits.m` para verificar simultáneamente la norma unitaria.
  - Precisar los docstrings y comentarios en `+panda_s1/simulate.m` y `box_diff.m` para clarificar la distinción entre guía de referencia y seguridad dinámica.
* **HITO R2 — Robustez de Tests y Desacoplamiento de Energía (Flash 3.8 High):**
  - Desacoplar la aserción $> 5000$ en `tests/test_s1_simulation_consistency.m`.
  - Asegurar que `setup_panda.m` agregue `tests/` o que los scripts de prueba configuren sus rutas limpiamente.
* **HITO R3 — Muestreo Físico en Benchmarks (Argon / Astra):**
  - Refactorizar `tools/benchmark_s1_comparison.m` para muestrear dentro de límites físicos acotados.
  - Regenerar `results/benchmark_s1_comparison.json`, `.mat` y `.png`.
* **HITO R4 — Pulido Editorial de LaTeX y README en GitHub (Flash 3.8 High):**
  - Eliminar matrices multilínea dentro de tablas markdown en `README.md`.
  - Extraer display math y unificar enlaces multimedia en `docs/assets/`.
* **HITO R5 — Integración, Merge a Main y Verificación Final (Argon / Astra):**
  - Ejecutar la suite completa de pruebas y benchmarks.
  - Merge limpio de `review/s1-scientific-polish` a `main` y sincronización con GitHub.
