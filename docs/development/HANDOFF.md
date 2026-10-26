# Franka Panda — Global Geometric Dynamics: Protocolo Handoff

**Archivo de seguimiento y transferencia entre modelos de IA (Argon/Astra y Flash 3.8 High)**  
**Repositorio:** `C:\Users\Johnny\Desktop\Git\Franka-Panda-Geometric-Simulator`  
**Rama activa:** `experiment/lee-s1-dynamics`  
**Última actualización:** 26 de octubre de 2026

---

## 1. Estado de los Hitos

| Hito | Descripción | Modelo Sugerido | Estado | Fecha / Nota |
|:---:|---|:---:|:---:|---|
| **HITO 0** | **Auditoría del repositorio original y baseline técnico** | **Flash 3.8 High** | **COMPLETADO** | 26-Oct-2026. Baseline medido y verificado. Rama creada. |
| **HITO 1** | Diseño matemático de la representación global ($S^1$ / $SO(2)$) | Argon / Astra | **PENDIENTE** | Siguiente hito al recibir orden «sigue». |
| **HITO 2** | Arquitectura y preparación de código | Flash 3.8 High | PENDIENTE | Espera a Hito 1. |
| **HITO 3** | Implementación geométrica y dinámica | Argon / Astra | PENDIENTE | Espera a Hito 2. |
| **HITO 4** | Integración, restricciones y consistencia física | Argon / Astra | PENDIENTE | Espera a Hito 3. |
| **HITO 5** | Benchmarks y comparación científica (A/B/C) | Flash 3.8 High | PENDIENTE | Espera a Hito 4. |
| **HITO 6** | Revisión matemática y científica final | Argon / Astra | PENDIENTE | Espera a Hito 5. |
| **HITO 7** | GIFs, presentación visual y publicación en GitHub | Flash 3.8 High | PENDIENTE | Publicación completa en `main` y GitHub. |

---

## 2. Hitos Anteriores Completados

### HITO 0 — Auditoría del repositorio original
- **Objetivo:** Establecer una referencia técnica rigurosa y reproducible antes de modificar el código.
- **Acciones realizadas:**
  1. Inspección exhaustiva de `+panda/`, `third_party/shape_kernels/`, `third_party/modern_robotics/`, `third_party/flatness/`, `sources/`, `tests/`, `tools/`, `docs/`, `results/` y `README.md`.
  2. Verificación criptográfica de 47 fuentes en `sources/` mediante `tools/verify_sources.py` (resultado: PASS).
  3. Ejecución local de todas las pruebas en MATLAB R2026a Update 4 (`validate_panda`, `test_source_geometry`, `test_simulation_inputs`, `run_panda_demo`). Todas pasaron al 100% de tolerancia.
  4. Ejecución del benchmark de dinámica completa $M(q) + c(q,\dot{q}) + g(q) + f(\dot{q})$ en 100 estados aleatorios en 5 repeticiones: Stock = $1,277.51\ \mu\text{s}$, Optimizado = $178.81\ \mu\text{s}$ (Speedup = $7.14\times$).
  5. Creación de la rama de trabajo aislada `experiment/lee-s1-dynamics` a partir de `main` (`3e41fa131a7b839b9075e4e8723d4e21631aded4`), preservando `main` inalterada.
  6. Redacción del informe detallado de auditoría en `docs/development/AUDIT_BASELINE.md`.

---

## 3. Decisiones Matemáticas y Técnicas Tomadas

1. **Invariabilidad de la Física y Parámetros:**
   - Se mantiene estrictamente el conjunto de parámetros físicos de las Tablas VIII y IX del suplemento revisado de Gaz et al. (2019). No se utilizarán los polinomios identificados de Gaz que conducían a $M_{77} = 0.001272\text{ kg m}^2$, conservando el valor físico coherente $M_{77} = 0.00490965\text{ kg m}^2$.
   - Fricción Gaz con corrección de la junta 2 ($\phi_{1,2} = 0.87224\text{ N m}$) implementada vía $\tanh$.

2. **Convención Cinemática y de Ejes:**
   - Las 7 articulaciones son revolutas con eje local $z$ ($A_i = [0;0;1;0;0;0]^T$).
   - Twists en orden $[\omega; v]$ (6x1). Wrenches en orden $[m; f]$ (6x1).
   - Base fija en $panda\_link0$. Brida $panda\_link8$ fijada rígidamente a $0.107\text{ m}$ sobre el eje z del eslabón 7.

3. **Geometría de los Límites en $S^1$:**
   - Todas las 7 articulaciones tienen un recorrido estrictamente menor que $2\pi$ ($2h_i < 2\pi$, $h_i < \pi$).
   - El arco admisible queda bien definido por $x_i^T u_i \ge \cos h_i$, donde $u_i = (\cos m_i, \sin m_i)^T$ y $m_i = (q_{min, i} + q_{max, i})/2$.
   - La junta 6 supera $\pi$ ($q_{max,6} = 3.7525\text{ rad} > \pi$); el levantamiento $q(x)$ debe centrarse en $m_6 = 1.8675\text{ rad}$ para que la discontinuidad de rama de $\mathrm{atan2}$ quede en el sector prohibido exterior.

---

## 4. Archivos Creados o Modificados

- `docs/development/AUDIT_BASELINE.md`: Creado. Informe completo de auditoría, dependencias y baseline.
- `docs/development/HANDOFF.md`: Creado. Este documento de transferencia y fuente de verdad.
- Rama `experiment/lee-s1-dynamics`: Creada y seleccionada.
- Rama `main`: Intacta en `3e41fa131a7b839b9075e4e8723d4e21631aded4`.

---

## 5. Pruebas Ejecutadas y Resultados

| Prueba | Comando / Script | Métrica Clave | Resultado |
|---|---|---|:---:|
| Verificación fuentes | `python tools/verify_sources.py` | 47 registros SHA-256 | PASS |
| Geometría de fuentes | `matlab -batch "test_source_geometry"` | Error pose brida: $2.84 \times 10^{-16}$ | PASS |
| Validación física | `matlab -batch "validate_panda"` | Error masa: $5.67 \times 10^{-16}$; Error gravedad: $2.21 \times 10^{-14}\text{ N m}$ | PASS |
| Entradas / límites | `matlab -batch "test_simulation_inputs"` | Saturación, efectividad, wrench externo | PASS |
| Demostración nominal | `matlab -batch "run_panda_demo"` | Res. energía: $5.62 \times 10^{-7}\text{ J}$; Error pos: $0.065\text{ mm}$ | PASS |
| Refinamiento paso | `run_panda_demo` (subpasos x2) | $\Delta q_{max} = 1.46 \times 10^{-7}\text{ rad}$ | PASS |
| Benchmark dinámica | `matlab -batch "benchmark_panda"` | Stock: $1277.5\ \mu\text{s}$, Optim: $178.8\ \mu\text{s}$ ($7.14\times$) | PASS |

---

## 6. Limitaciones Conocidas

1. **Límites de posición en el simulador:** Actualmente son **monitoreados** en cada etapa RK4, no impuestos como restricciones mecánicas duras de contacto (no hay modelo de choque o fuerzas normales de tope).
2. **Caché bitwise existente:** `rnea_shared_kinematics_MR` compara `typecast(thetalist, 'uint64')` para $q \in \mathbb{R}^7$. Para una representación en $S^1$ con $x \in \mathbb{R}^{14}$ o matrices $SO(2)$, se requerirá una estrategia de caché equivalente y coherente.
3. **Potencial gravitatorio:** En coordenadas relativas de juntas, el potencial $V(q)$ no es lineal en los pares $(\cos q_i, \sin q_i)$ individuales porque aparecen productos trigonométricos a lo largo de la cadena cinemática.
4. **Cálculo de masa completa:** La matriz de masa completa $M(q)$ requiere 7 recursiones hacia atrás ($O(n^2)$), mientras que la dinámica inversa de esfuerzo articular $\tau$ es $O(n)$.

---

## 7. Control de Versiones y Git

- **Commit base original:** `3e41fa131a7b839b9075e4e8723d4e21631aded4`
- **Rama experimental:** `experiment/lee-s1-dynamics`
- **Estado de sincronización:** Rama local creada. Cambios de documentación listos para commit en la rama experimental.
- **Rama `main`:** Limpia, sincronizada con `origin/main`.

---

## 8. Próxima Acción Concreta

- **HITO 1: Diseño matemático de la representación global ($S^1$ / $SO(2)$)**
  - **Modelo recomendado:** Argon / Astra (razonamiento matemático profundo).
  - **Tareas inmediatas al recibir «sigue»:**
    1. Estudiar el Capítulo 4 del libro de Lee, Leok y McClamroch (`Global Formulations of Lagrangian and Hamiltonian Dynamics`).
    2. Formalizar el espacio de configuración $\mathsf{Q} = (S^1)^7$ y su álgebra de Lie / espacio tangente $(\mathbb{R})^7$.
    3. Resolver la formulación cinemática relativa y global conectada con las poses de eslabón $SE(3)$.
    4. Analizar rigurosamente la desigualdad de límites $x_i^T u_i \ge \cos h_i$, la singularidad del corte de $\mathrm{atan2}$ (especialmente en junta 6 con $q > \pi$), y la definición de diferencias geodésicas vs levantadas para control.
    5. Proponer el esquema de integración numérica que preserve numéricamente la restricción $\|x_i\| = 1$.
    6. Redactar el documento formal de diseño matemático `docs/development/MATHEMATICAL_FORMULATION.md`.
