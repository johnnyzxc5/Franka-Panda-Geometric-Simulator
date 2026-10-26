# Auditoría Técnica y Baseline del Franka Panda Geometric Simulator

**Fecha:** 26 de octubre de 2026  
**Commit base:** `3e41fa131a7b839b9075e4e8723d4e21631aded4` (`main`, release 0.1.0)  
**Entorno validado:** MATLAB R2026a Update 4 (`26.1.0.3312084`), Windows 11, Python 3.13  
**Ejecutor:** Antigravity (Gemini 3.8 Flash High)

---

## 1. Resumen Ejecutivo

El repositorio `Franka-Panda-Geometric-Simulator` proporciona una implementación mecánica rigurosa y autocontenida (base MATLAB puro, sin toolboxes de pago ni dependencias MEX/Simulink/Python en tiempo de ejecución) del manipulador Franka Emika Panda de 7 grados de libertad (DoF).

La implementación original (**Versión A**) modela el estado de configuración en coordenadas articulares euclidianas $q \in \mathbb{R}^7$, apoyándose en el formalismo de Lie $SE(3)$, Product of Exponentials (PoE), transformaciones adjuntas y el algoritmo recursivo de Newton-Euler (RNEA) espacial con optimizaciones de hoisting de invariantes y caché de estados.

La auditoría confirma que el código original está íntegro, todas las pruebas numéricas pasan con errores residuales en el rango de precisión de máquina (`double`), los 47 registros de procedencia SHA-256 coinciden exactamente, y el benchmark local reproduce una aceleración de **7.14x** frente a la implementación de referencia canónica de Modern Robotics.

---

## 2. Mapa Arquitectural y Dependencias

```
Franka-Panda-Geometric-Simulator/
├── +panda/                                 # Paquete de espacio de nombres MATLAB principal
│   ├── config.m                            # Configuración de simulación, controlador y callbacks
│   ├── draw_robot.m                        # Renderizado 3D de eslabones y mallas STL
│   ├── dynamics.m                          # Matriz M(q) completa y bias b = c(q,dq) + g(q) + f(dq)
│   ├── energy.m                            # Hamiltoniano mecánico H = 1/2 dq'*M*dq + V(q)
│   ├── fk.m                                # Cinemática directa PoE espacial (SE(3))
│   ├── forward_dynamics.m                  # Dinámica directa ddq = M \ (tau + Jb'*F_env - b)
│   ├── frames.m                            # Cinemática directa recursiva de eslabones sucesivos
│   ├── friction.m                          # Modelo de fricción sigmoide Gaz 2019 con corrección
│   ├── inverse_dynamics.m                  # Dinámica inversa RNEA: esfuerzo para aceleración deseada
│   ├── jacobian.m                          # Jacobiano de twist espacial y corporal (6x7, [omega; v])
│   ├── joint_vector.m                      # Validación y coercitivo de vector articular (7x1 double)
│   ├── lagrange_reference.m                # Cálculo independiente COM-Jacobiano/Lagrange (oráculo)
│   ├── model.m                             # Construcción del modelo, DH modificado, inercias y tornillos
│   ├── plot_results.m                      # Generación de diagnósticos gráficos y figuras PNG
│   ├── read_stl.m                          # Lector binario de mallas STL para visualización
│   ├── reference.m                         # Generador de trayectorias quintic a partir de phase_plan
│   ├── rpy_rotation.m                      # Matriz SO(3) roll-pitch-yaw estándar
│   ├── simulate.m                          # Integrador RK4 (1 ms) con lazo de control ZOH (4 ms)
│   ├── wrench_vector.m                     # Validación de wrench [moment; force] (6x1 double)
│   └── write_json.m                        # Serializador de métricas a JSON
├── third_party/
│   ├── modern_robotics/                    # Librería de Lynch & Park (MIT) para SE(3)
│   ├── shape_kernels/                      # Kernels RNEA optimizados con hoisting y caché (MIT)
│   │   ├── rnea_shared_kinematics_MR.m     # Precálculo de cinemática compartida y caché bitwise
│   │   ├── InverseDynamicsSharedKinematics_MR.m # RNEA hacia adelante y atrás con ad(V) directo
│   │   └── MassMatrixZeroStateSharedKinematics_MR.m # Matriz de masa P3 sin bias de gravedad
│   └── flatness/
│       └── +ftv5/                          # Planificador de fase quintic diferencialmente plano
├── sources/                                # Metadatos de ingeniería y mallas oficiales
│   ├── parameters.json                     # Parámetros físicos (Gaz 2019 Tablas VIII y IX)
│   ├── joint_limits.yaml / origins.json    # Límites articulares y orígenes URDF/Xacro
│   └── meshes/                             # Archivos DAE y STL de los eslabones 0 a 7
├── tests/                                  # Banco de pruebas de regresión y validación física
│   ├── validate_panda.m                    # 40 configuraciones aleatorias contra oráculo de Lagrange
│   ├── test_source_geometry.m              # Verificación contra orígenes de fabricante Xacro
│   └── test_simulation_inputs.m            # Verificación de efectividad, saturación y wrench externo
├── tools/
│   ├── benchmark_panda.m                   # Benchmark comparativo stock vs optimizado (100 estados)
│   └── verify_sources.py                   # Verificador criptográfico SHA-256 de archivos fuente
├── run_panda_demo.m                        # Script maestro de demostración y refinamiento temporal
└── setup_panda.m                           # Inicializador de paths de MATLAB
```

### Grafo de Flujo de Datos y Dependencias

```
[parameters.json] -> panda.model() -> Struct 'm' (Mlist, Slist, Glist, friction, etc.)
                          |
    +---------------------+---------------------+
    |                     |                     |
panda.fk()           panda.frames()       panda.jacobian()
(PoE espacial)      (Local chain)         (Space & Body)
    |                     |                     |
    +---------------------+---------------------+
                          |
             rnea_shared_kinematics_MR
             (Invariant hoisting & bitwise cache)
                          |
        +-----------------+-----------------+
        |                                   |
MassMatrixZeroState...             InverseDynamicsShared...
(Zero-state P3 mass)               (Forward & backward RNEA)
        |                                   |
        +-----------------+-----------------+
                          |
                   panda.dynamics()
                   [M, b = c + g + f]
                          |
        +-----------------+-----------------+
        |                                   |
panda.forward_dynamics()            panda.inverse_dynamics()
(ddq = M \ (tau - b))               (tau = RNEA(q,dq,ddq) + f)
        |                                   |
        +-----------------+-----------------+
                          |
                   panda.simulate()
                   (RK4 plant, ZOH control, energy monitor)
```

---

## 3. Inspección Rigurosa de Componentes

### 3.1. Convenciones Geométricas y de Ejes
- **Base y Brida:** Base fija $panda\_link0$. Brida $panda\_link8$ fijada rígidamente a $0.107\text{ m}$ sobre el eje z del último marco articular (link 7).
- **Tornillos Espaciales ($S_i$):** $S_i = [w_i; -w_i \times p_i] \in \mathbb{R}^6$, orden $[\omega; v]$. $w_i = R_{home, i} \cdot [0;0;1]^T$.
- **Tornillos Corporales ($B_i$):** $B_i = \mathrm{Ad}_{T_{home}^{-1}} S_i$.
- **Ejes Locales:** Todas las 7 articulaciones son revolutas puras alrededor del eje local $z$: $A_i = \mathrm{Ad}_{(M_1\dots M_i)^{-1}} S_i = [0;0;1;0;0;0]^T$ (hasta precisión de redondeo numérico).
- **Twists y Wrenches:**
  - Twist: $V = [\omega; v] \in \mathbb{R}^6$ (velocidad angular sobre velocidad lineal).
  - Wrench: $F = [m; f] \in \mathbb{R}^6$ (momento sobre fuerza).
  - Dualidad de potencia: $P = F^T V = m^T \omega + f^T v$.
  - Signo de interacción: En dinámica directa `panda.forward_dynamics`, $F_{env}$ es aplicado **por el entorno sobre el robot** ($+J_b^T F_{env}$). En dinámica inversa `panda.inverse_dynamics`, $F_{tip}$ es el wrench aplicado **por el robot sobre el entorno** ($F_{tip} = -F_{env}$).

### 3.2. Parámetros Físicos y Rigor de Inercias
- **Origen de Parámetros:** Tabla VIII (masas y centros de masa) y Tabla IX (tensores de inercia baricéntricos) del suplemento revisado de Gaz et al. (15 de octubre de 2019).
- **Inercia Espacial:** Sobre el origen de cada eslabón:
  $$G_i = \begin{bmatrix} I_{com, i} - m_i [c_i]_\times [c_i]_\times & m_i [c_i]_\times \\ -m_i [c_i]_\times & m_i I_{3\times 3} \end{bmatrix}$$
- **Propiedades físicas verificadas:** Todos los tensores $I_{com, i}$ son simétricos, estrictamente definidos positivos y satisfacen las desigualdades triangulares de momentos principales ($I_{xx} + I_{yy} > I_{zz}$, etc.).
- **Diferencia crítica con código generado de Gaz:** El valor $M_{77} = 0.00490965\text{ kg m}^2$ se deriva rigurosamente de $I_{7,zz} + m_7 (c_{7x}^2 + c_{7y}^2)$. El código simbólico generado por Gaz reporta $0.00127224\text{ kg m}^2$. El simulador mantiene estrictamente la coherencia física de las tablas y no mezcla ambos conjuntos.

### 3.3. Modelo de Fricción
- Ley sigmoide de Gaz (2019), ec. (25), implementada vectorialmente con $\tanh$:
  $$f_i(\dot{q}_i) = \frac{1}{2} \phi_{1,i} \left( \tanh\left(\frac{1}{2} \phi_{2,i}(\dot{q}_i + \phi_{3,i})\right) - \tanh\left(\frac{1}{2} \phi_{2,i}\phi_{3,i}\right) \right)$$
- Parámetros: $\phi_{1}, \phi_{2} > 0$. Incluye el erratum publicado para la articulación 2: $\phi_{1,2} = 0.87224\text{ N m}$.
- Propiedad de disipación: $\dot{q}^T f(\dot{q}) \ge 0$ estrictamente, con $f(0) = 0$.

### 3.4. Kernels RNEA Optimizados y Caché
- **`rnea_shared_kinematics_MR.m`:**
  - *Invariant Hoisting:* Precomputa y almacena $M_i^{-1}$, $A_i$, y $\mathrm{Ad}_{M_{8}^{-1}}$. Solo se recalculan si las matrices estructurales $Mlist$ o $Slist$ cambian (invalidación segura bit a bit).
  - *Bitwise State Cache:* Utiliza `typecast(thetalist, 'uint64')` para comparación exacta en 64 bits. Si el vector $q$ es idéntico al evaluado previamente, reutiliza la cinemática ya proyectada.
- **`InverseDynamicsSharedKinematics_MR.m`:**
  - Implementa RNEA espacial recursivo evaluando el corchete de Lie $\mathrm{ad}_V$ directamente con indexación sin matrices dispersas intermedias.
- **`MassMatrixZeroStateSharedKinematics_MR.m`:**
  - Evalúa la matriz de masa columna a columna imponiendo $\dot{q} = 0$ y aceleración nula de gravedad en la base, propagando la aceleración articular unitaria solo a partir del eslabón $j$.
  - Complejidad: $O(n^2)$ para la matriz completa $7 \times 7$, mientras que la dinámica inversa escalar es $O(n)$.

### 3.5. Simulación, Integración y Balance Energético
- Controlador computed-torque con ZOH a $\Delta t_c = 4\text{ ms}$ ($250\text{ Hz}$).
- Integrador RK4 continuo para la planta con paso $h = 1\text{ ms}$ (4 subpasos por ciclo de control).
- Monitoreo de balance de energía mecánica:
  $$\Delta E(t) = H(t) - H(0) - W_{joint}(t) - W_{env}(t) + E_{fric}(t) \approx 0$$
  donde $H = \frac{1}{2} \dot{q}^T M(q) \dot{q} + V(q)$.
- Monitoreo continuo de márgenes articulares y velocidad máxima en todas las 4 etapas intermedias de RK4.

---

## 4. Límites Articulares y Geometría en $S^1$

Los límites físicos registrados en `sources/parameters.json` y `sources/joint_limits.yaml` son:

| Articulación | $q_{min}$ [rad] | $q_{max}$ [rad] | Amplitud $2h_i$ [rad] | Centro $m_i$ [rad] | Semiamplitud $h_i$ [rad] | $\cos(h_i)$ | Observaciones críticas |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| 1 | -2.8973 | 2.8973 | 5.7946 | 0.0000 | 2.8973 | -0.970309 | Simétrico, $2h < 2\pi$ |
| 2 | -1.7628 | 1.7628 | 3.5256 | 0.0000 | 1.7628 | -0.190826 | Simétrico |
| 3 | -2.8973 | 2.8973 | 5.7946 | 0.0000 | 2.8973 | -0.970309 | Simétrico |
| 4 | -3.0718 | -0.0698 | 3.0020 | -1.5708 | 1.5010 | +0.069740 | **Asimétrico, totalmente negativo** |
| 5 | -2.8973 | 2.8973 | 5.7946 | 0.0000 | 2.8973 | -0.970309 | Simétrico |
| 6 | -0.0175 | 3.7525 | 3.7700 | +1.8675 | 1.8850 | -0.309059 | **Asimétrico, atraviesa $\pi$ ($3.7525 > \pi$)** |
| 7 | -2.8973 | 2.8973 | 5.7946 | 0.0000 | 2.8973 | -0.970309 | Simétrico |

### Hallazgo Decisivo para el Hito 1:
1. **Amplitud estrictamente menor que $2\pi$:** Para las 7 articulaciones, $2h_i < 2\pi$ ($h_i < \pi$). Por tanto, el arco permitido no cubre el círculo completo; existe un sector prohibido no vacío en cada articulación.
2. **Desigualdad del arco:** Definiendo $u_i = (\cos m_i, \sin m_i)^T$ y $x_i = (\cos q_i, \sin q_i)^T \in S^1$, la condición física admisible es:
   $$x_i^T x_i = 1, \qquad x_i^T u_i \ge \cos h_i$$
3. **Corte angular y levantamiento:** Para la articulación 6, el ángulo físico alcanza $3.7525\text{ rad} \approx 215^\circ$. Un `atan2(s,c)` directo estándar produce valores en $(-\pi, \pi]$, rompiendo la continuidad en $\pi$. La reconstrucción debe centrarse en $m_i$:
   $$q_i = m_i + \mathrm{atan2}(s_i \cos m_i - c_i \sin m_i, \; c_i \cos m_i + s_i \sin m_i)$$
   Como $h_i < \pi$, el corte de este `atan2` centrado cae en el arco prohibido exterior, garantizando continuidad matemática y numérica en toda la zona admisible.

---

## 5. Medición del Baseline de Pruebas y Rendimiento

Ejecutado localmente en MATLAB R2026a Update 4:

### 5.1. Validación Física (`validate_panda.m`)
- Semilla RNG: `13102026`, 40 estados aleatorios en el interior de límites articulares.
- Discrepancia relativa en matriz de masa ($M$ vs Lagrange $M_L$): **$5.67 \times 10^{-16}$**
- Discrepancia en gravedad ($g$ vs Lagrange $g_L$): **$2.21 \times 10^{-14}\text{ N m}$**
- Discrepancia en Coriolis ($c$ vs Lagrange $\dot{M} \dot{q} - \frac{1}{2} \nabla K$): **$5.76 \times 10^{-10}\text{ N m}$**
- Discrepancia cinemática PoE vs cadena relativa: **$7.47 \times 10^{-16}\text{ m}$**
- Discrepancia Jacobiano vs diferencias finitas: **$6.32 \times 10^{-11}$**
- Asimetría de matriz de masa ($\|M - M^T\|_F$): **$9.38 \times 10^{-16}$**
- Autovalor mínimo de masa: **$0.003563\text{ kg m}^2 > 0$** (estrictamente definida positiva)
- Identidad de potencia de Coriolis ($\dot{q}^T c - \frac{1}{2} \dot{q}^T \dot{M} \dot{q}$): **$5.05 \times 10^{-10}\text{ W}$**
- Error de ida y vuelta dinámica inversa / directa: **$3.91 \times 10^{-13}\text{ rad/s}^2$**
- Discrepancia con Modern Robotics stock: **$0.00$ (exactitud bit a bit)**
- Invalidación de caché ante cambio de geometría/tornillos: **Superada ($< 10^{-10}$)**
- Inercia aparente diagonal opcional añadida una sola vez: **Superada**
- Resultado global: **PASS**

### 5.2. Geometría de Fuentes (`test_source_geometry.m`)
- Discrepancia en transformaciones relativas frente a Xacro: **$2.35 \times 10^{-17}$**
- Discrepancia en ejes de giro: **$0.00$**
- Discrepancia de pose en brida: **$2.84 \times 10^{-16}$**
- Resultado: **PASS**

### 5.3. Entradas y Límites de Simulación (`test_simulation_inputs.m`)
- Trabajo ambiental no nulo registrado: $3.6333 \times 10^{-4}\text{ J}$
- Pérdida de efectividad del 10% en junta 2: Comprobada exactamente
- Saturación de torque: Comprobada en 100% de muestras forzadas
- Pose en bordes articulares: Discrepancia $< 3.52 \times 10^{-16}$
- Resultado: **PASS**

### 5.4. Demostración y Refinamiento Temporal (`run_panda_demo.m`)
- Duración nominal: $3.6\text{ s}$, control ZOH a $4\text{ ms}$, planta a $1\text{ ms}$.
- Error máximo de seguimiento articular: **$0.087^\circ$** ($0.0015\text{ rad}$)
- Error RMS articular: **$0.021^\circ$**
- Error máximo cartesiano de posición de efector: **$0.0653\text{ mm}$**
- Residual máximo del balance de energía: **$5.6218 \times 10^{-7}\text{ J}$**
- Refinamiento de paso de planta a $0.5\text{ ms}$ (paso mitad):
  - Diferencia máxima de posición articular: **$1.4597 \times 10^{-7}\text{ rad}$**
  - Diferencia máxima de velocidad articular: **$3.8874 \times 10^{-7}\text{ rad/s}$**
- Violación de límites articulares en etapas RK4: **$0.0000\text{ rad}$** (margen mínimo: $0.9718\text{ rad}$)
- Resultado: **PASS**

### 5.5. Rendimiento y Benchmark de Dinámica (`benchmark_panda.m`)
- 100 estados aleatorios distintos $(q, \dot{q})$ dentro de límites, 5 repeticiones calentadas.
- Tarea medida: Evaluación conjunta de matriz de masa completa $M(q)$ y vector de bias $b = c(q,\dot{q}) + g(q) + f(\dot{q})$.
- Tiempo mediano Modern Robotics stock: **$1,277.51\ \mu\text{s}$**
- Tiempo mediano Kernels optimizados (SHAPE): **$178.81\ \mu\text{s}$**
- Factor de aceleración (Speedup): **$7.14\times$**

---

## 6. Conclusión de la Auditoría

El repositorio en el commit `3e41fa131a7b839b9075e4e8723d4e21631aded4` constituye una base de referencia limpia, reproducible y de alta precisión. Todos los componentes mecánicos, cinemáticos y de simulación están formalmente vinculados y verificados.

Queda establecido el entorno para abordar el **HITO 1: Diseño matemático de la representación global en $S^1$ / $SO(2)$**.
