

# 🔐 Warden: Sistema de Reconocimiento Facial para Control de Acceso

## Descripción

**Warden** es un sistema de seguridad biométrica diseñado para controlar el acceso a áreas sensibles utilizando reconocimiento facial. Está enfocado en contextos donde se requiere una protección estricta, como:

- Instalaciones gubernamentales
- Instituciones bancarias
- Cárceles
- Infraestructura crítica

El sistema combina hardware embebido con visión computacional, implementando un prototipo funcional de **puerta inteligente** que se abre solo tras un reconocimiento facial exitoso.

---

## 🧠 Componentes Clave

### ⚙️ Electrónica
- **Microcontrolador:** [NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)
- **Sensor de proximidad:** Sensor ultrasónico HC-SR04
- **Motor de apertura:** Servomotor **MOT110 (3V)**

### 🎥 Visión por Computadora
- **Cámara:** USB estándar
- **Procesamiento:** Computadora con Python 3 instalado
- **Modelo de detección:** [MediaPipe FaceMesh](https://google.github.io/mediapipe/solutions/face_mesh)

---

## 🔧 Funcionamiento

1. **Detección de proximidad** mediante el sensor ultrasónico.
2. Activación de la **cámara USB** al detectar una persona.
3. Captura de imagen y detección de rostro con **MediaPipe FaceMesh**.
4. Si la identificación facial coincide con el patrón autorizado:
   - Se activa el servomotor para abrir la puerta.

---
