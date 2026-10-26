# Franka Panda — Global Geometric Dynamics: Protocolo Handoff

**Archivo de seguimiento y transferencia entre modelos de IA (Argon/Astra y Flash 3.8 High)**  
**Repositorio:** `C:\Users\Johnny\Desktop\Git\Franka-Panda-Geometric-Simulator`  
**Rama activa:** `experiment/lee-s1-dynamics`  
**Última actualización:** 26 de octubre de 2026 (Hito 2 completado)

---

## 1. Estado de los Hitos

| Hito | Descripción | Modelo Sugerido | Estado | Fecha / Nota |
|:---:|---|:---:|:---:|---|
| **HITO 0** | **Auditoría del repositorio original y baseline técnico** | **Flash 3.8 High** | **COMPLETADO** | 26-Oct-2026. Baseline medido y verificado. Rama creada. |
| **HITO 1** | **Diseño matemático de la representación global ($S^1$ / $SO(2)$)** | **Argon / Astra** | **COMPLETADO** | 26-Oct-2026. Formulación, topología de límites, integrador RK4-MK y contratos en `MATHEMATICAL_FORMULATION.md`. |
| **HITO 2** | **Arquitectura y preparación de código** | **Flash 3.8 High** | **COMPLETADO** | 26-Oct-2026. Paquete `+panda_s1`, kernel `rnea_shared_kinematics_S1`, interfaces y tests de geometría/límites. |
| **HITO 3** | Implementación geométrica y dinámica | Argon / Astra | **PENDIENTE** | Siguiente hito al recibir orden «sigue». |
| **HITO 4** | Integración, restricciones y consistencia física | Argon / Astra | PENDIENTE | Espera a Hito 3. |
| **HITO 5** | Benchmarks y comparación científica (A/B/C) | Flash 3.8 High | PENDIENTE | Espera a Hito 4. |
| **HITO 6** | Revisión matemática y científica final | Argon / Astra | PENDIENTE | Espera a Hito 5. |
| **HITO 7** | GIFs, presentación visual y publicación en GitHub | Flash 3.8 High | PENDIENTE | Publicación completa en `main` y GitHub. |

---

## 2. Hitos Anteriores Completados

### HITO 0 — Auditoría del repositorio original (Flash 3.8 High)
- Inspección completa del repositorio base.
- Verificación criptográfica SHA-256 de las 47 fuentes (`tools/verify_sources.py` -> PASS).
- Baseline en MATLAB R2026a Update 4 (`validate_panda`, `test_source_geometry`, `test_simulation_inputs`, `run_panda_demo`, `benchmark_panda` -> PASS, speedup $7.14\times$).
- Rama `experiment/lee-s1-dynamics` inicializada.
- Documentado en `docs/development/AUDIT_BASELINE.md`. Commit `f812988`.

### HITO 1 — Diseño matemático de la representación global (Argon)
- Selección de la variedad de configuración $\mathsf{Q} = (S^1)^7$ embebida en $\mathbb{R}^{2\times 7}$, con velocidades angulares $\omega \in \mathbb{R}^7$ y cinemática $\dot{x}_i = \omega_i J x_i$.
- Demostración de exponenciales de tornillo puramente algebraicas en $S^1$ y equivalencia exacta entre las ecuaciones reducidas de Euler–Lagrange de Lee (Capítulo 4) y el RNEA espacial (`InverseDynamicsSharedKinematics_MR` y `MassMatrixZeroStateSharedKinematics_MR`).
- Análisis topológico de límites: 6 de las 7 juntas tienen amplitud admisible $2h_i > \pi$, lo que hace que la geodésica más corta en $S^1$ cruce el tope mecánico prohibido cuando $|q_2 - q_1| > \pi$. Se establecen `geodesic_diff` (con detección de cruce de tope) y `box_diff` (desplazamiento continuo sobre el arco admisible $\mathcal{A}_i$).
- Formulación del integrador Munthe–Kaas RK4 de 4 etapas (RK4-MK) sobre el grupo de Lie abeliano $(SO(2))^7 \ltimes \mathbb{R}^7$.
- Documentado en `docs/development/MATHEMATICAL_FORMULATION.md`. Commit `ef52d85`.

### HITO 2 — Arquitectura y preparación de código (Flash 3.8 High)
- Creación de la estructura del nuevo paquete `+panda_s1/` sin modificar una sola línea de `+panda/`:
  - `+panda_s1/config_matrix.m`: Validación, normalización y verificación de norma unitaria en $\mathbb{R}^{2\times 7}$ / $\mathbb{R}^{14}$.
  - `+panda_s1/model.m`: Extensión del modelo con geometría de arcos $S^1$ (`qmid`, `qhalf`, `umid`, `cos_half`, `is_pure_z`).
  - `+panda_s1/from_q.m`: Embebido $q \mapsto X = [\cos q^T; \sin q^T]$.
  - `+panda_s1/to_q.m`: Levantamiento centrado branch-cut-safe $X \mapsto q \in [q_{\min}, q_{\max}]$.
  - `+panda_s1/exp_s1.m`: Acción de grupo $\operatorname{Exp}_{S^1}(\Delta\theta, X)$ con preservación unitaria.
  - `+panda_s1/check_limits.m`: Evaluación algebraica de $u_i^T x_i \ge \cos h_i$, error de norma y márgenes directos $\mu_i(x_i)$.
  - `+panda_s1/geodesic_diff.m`: Logaritmo principal en $SO(2)^7$ con bandera `crosses_forbidden_arc` y error cordal de Lee.
  - `+panda_s1/box_diff.m`: Desplazamiento sobre el arco admisible $\mathcal{A}_i$, homotópicamente consistente para control y trayectorias.
  - `+panda_s1/fk.m`, `+panda_s1/frames.m`, `+panda_s1/jacobian.m`: Cinemática sobre $(S^1)^7$.
  - `+panda_s1/dynamics.m`, `+panda_s1/inverse_dynamics.m`, `+panda_s1/forward_dynamics.m`, `+panda_s1/energy.m`: Dinámica sobre $(S^1)^7$.
  - `+panda_s1/simulate.m`: Integrador RK4-MK de Lie con lazo ZOH y monitoreo por etapas.
- Implementación de `third_party/shape_kernels/rnea_shared_kinematics_S1.m`:
  - Invariant hoisting y caché bitwise de 14 doubles (`typecast(X(:), 'uint64')`).
  - Rotaciones locales directas $SO(2)$ para ejes $z$ puros con fallback general ante tornillos alterados.
  - Reutilización directa de `MassMatrixZeroStateSharedKinematics_MR` e `InverseDynamicsSharedKinematics_MR` sin duplicar código.
- Creación y ejecución de la suite de pruebas `tests/test_s1_geometry_limits.m`:
  - 200 muestras de ida y vuelta $q \leftrightarrow X$: error máximo $4.44 \times 10^{-16}\text{ rad}$.
  - Continuidad y límites en Junta 4 (negativa) y Junta 6 ($q > \pi$): error $< 10^{-14}\text{ rad}$.
  - Discriminación estricta de arcos en $q_{\min}, q_{\max}$ y rechazo de puntos fuera de límites.
  - Detección de cruce de tope prohibido e inversión de signo en las 6 juntas con $2h > \pi$.
  - Preservación de norma tras 100 pasos de `Exp_S1`: residuo $2.22 \times 10^{-16}$.
  - Paridad inicial con `+panda`: FK, Frames, Jacobianos, Dinámica y Energía dentro de $10^{-14}$.
- Verificación de regresión en las pruebas originales (`validate_panda`, `test_source_geometry`, `test_simulation_inputs`): **100% PASS**.

---

## 3. Decisiones Matemáticas y Técnicas Tomadas

1. **Coexistencia Limpia de Paquetes:**
   - La implementación original (`+panda/`) permanece inalterada como referencia de verdad para la Versión A.
   - La nueva implementación reside en `+panda_s1/` (**Versión B**).
2. **Cero Duplicación de Kernels RNEA:**
   - `MassMatrixZeroStateSharedKinematics_MR.m` e `InverseDynamicsSharedKinematics_MR.m` se reutilizan directamente tanto en `+panda/` como en `+panda_s1/`.
3. **Formato Canónico de Configuración en $S^1$:**
   - Matriz $2\times 7$ double donde la columna $i$ es $[c_i; s_i]$. Se aceptan de forma transparente vectores $14\times 1$ o $1\times 14$ mediante `panda_s1.config_matrix(X)`.

---

## 4. Archivos Creados o Modificados

- `+panda_s1/config_matrix.m` (Creado)
- `+panda_s1/model.m` (Creado)
- `+panda_s1/from_q.m` (Creado)
- `+panda_s1/to_q.m` (Creado)
- `+panda_s1/exp_s1.m` (Creado)
- `+panda_s1/check_limits.m` (Creado)
- `+panda_s1/geodesic_diff.m` (Creado)
- `+panda_s1/box_diff.m` (Creado)
- `+panda_s1/fk.m` (Creado)
- `+panda_s1/frames.m` (Creado)
- `+panda_s1/jacobian.m` (Creado)
- `+panda_s1/dynamics.m` (Creado)
- `+panda_s1/inverse_dynamics.m` (Creado)
- `+panda_s1/forward_dynamics.m` (Creado)
- `+panda_s1/energy.m` (Creado)
- `+panda_s1/simulate.m` (Creado)
- `third_party/shape_kernels/rnea_shared_kinematics_S1.m` (Creado)
- `tests/test_s1_geometry_limits.m` (Creado)
- `docs/development/HANDOFF.md` (Actualizado)

---

## 5. Pruebas Ejecutadas y Resultados

| Prueba | Script | Resultado | Métricas Clave |
|---|---|:---:|---|
| Geometría y Límites $S^1$ | `tests/test_s1_geometry_limits.m` | **PASS** | Error ida/vuelta: $4.44 \times 10^{-16}\text{ rad}$<br>Deriva unitaria (100 pasos): $2.22 \times 10^{-16}$<br>Paridad dinámica: $< 3.24 \times 10^{-16}$ |
| Validación Original | `tests/validate_panda.m` | **PASS** | Error masa: $5.67 \times 10^{-16}$<br>Gravedad: $2.21 \times 10^{-14}\text{ N m}$ |
| Geometría Original | `tests/test_source_geometry.m` | **PASS** | Error pose: $2.84 \times 10^{-16}$ |
| Entradas / Simulación Original | `tests/test_simulation_inputs.m` | **PASS** | Saturación y efectividad verificadas |

---

## 6. Limitaciones Conocidas

1. `+panda_s1/simulate.m` implementa la arquitectura RK4-MK de Lie con lazo ZOH; la verificación exhaustiva de convergencia de orden temporal y pasividad/conservación de energía en maniobras forzadas corresponde al Hito 4.
2. `rnea_shared_kinematics_S1.m` tiene soporte de rotación pura $SO(2)$ e incluye fallback general ante tornillos no z; su validación bajo perturbaciones extremas de tornillos se consolidará en Hito 3.

---

## 7. Control de Versiones y Git

- **Commit Hito 0:** `f812988`
- **Commit Hito 1:** `ef52d85`
- **Rama experimental activa:** `experiment/lee-s1-dynamics`
- **Rama `main`:** Intacta en `3e41fa131a7b839b9075e4e8723d4e21631aded4`.

---

## 8. Próxima Acción Concreta

- **HITO 3: Implementación geométrica y dinámica**
  - **Modelo sugerido:** Argon / Astra (razonamiento científico profundo).
  - **Tareas inmediatas al recibir «sigue»:**
    1. Revisar en detalle las implementaciones de cinemática y dinámica en `+panda_s1/` y el kernel `rnea_shared_kinematics_S1.m`.
    2. Desarrollar un banco de pruebas exhaustivo de equivalencia física y matemática profunda entre Versión A (`+panda`) y Versión B (`+panda_s1`) para todos los operadores:
       - Dinámica inversa con wrenches de contacto en la brida (`Ftip` robot $\to$ entorno).
       - Dinámica directa con wrenches del entorno (`F_env` entorno $\to$ robot).
       - Invalidación de caché bit a bit ante cambios dinámicos de geometría y tornillos helicoidales alterados (reproduciendo la prueba de línea 65 de `validate_panda.m`).
       - Propiedades de pasividad y disipación de fricción $\omega^T f(\omega) \ge 0$.
       - Verificación de la identidad de potencia de Coriolis: $\omega^T c(X, \omega) = \frac{1}{2} \omega^T \dot{M}(X, \omega) \omega$.
    3. Documentar las justificaciones computacionales y equivalencias en un informe técnico de dinámica.
    4. Realizar el commit del Hito 3, actualizar `HANDOFF.md` y detenerse.
