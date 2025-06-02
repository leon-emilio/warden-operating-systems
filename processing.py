import math
import numpy as np


def get_face_center(landmarks):
    # Puntos clave para el centro facial
    # Esquinas internas de los ojos
    left_eye_inner = landmarks[133]  # Esquina interna ojo izquierdo
    right_eye_inner = landmarks[362]  # Esquina interna ojo derecho

    # Punta de la nariz
    nose_tip = landmarks[1]

    # Centro entre las cejas
    between_eyebrows = landmarks[9]

    # Promedio ponderado de estos puntos
    face_center = [
        (left_eye_inner[0] + right_eye_inner[0] + nose_tip[0] + between_eyebrows[0]) / 4,
        (left_eye_inner[1] + right_eye_inner[1] + nose_tip[1] + between_eyebrows[1]) / 4
    ]

    return face_center


def get_face_scale(landmarks):

    # Esquinas externas de los ojos
    left_eye_outer = landmarks[33]  # Esquina externa ojo izquierdo
    right_eye_outer = landmarks[263]  # Esquina externa ojo derecho

    # Distancia euclidiana entre esquinas externas
    dx = right_eye_outer[0] - left_eye_outer[0]
    dy = right_eye_outer[1] - left_eye_outer[1]
    eye_distance = math.sqrt(dx ** 2 + dy ** 2)

    return eye_distance


def rotation_angle_func(landmarks):
    # Esquinas externas de los ojos (más estables que las internas)
    left_eye_outer = landmarks[33]
    right_eye_outer = landmarks[263]

    dx = right_eye_outer[0] - left_eye_outer[0]
    dy = right_eye_outer[1] - left_eye_outer[1]
    angle = math.atan2(dy, dx)

    return angle


def rotate_point(point, angle, center=(0, 0)):

    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)

    # Trasladar al origen
    x = point[0] - center[0]
    y = point[1] - center[1]

    # Rotar
    rotated_x = cos_angle * x - sin_angle * y
    rotated_y = sin_angle * x + cos_angle * y

    # Trasladar de vuelta
    rotated_x += center[0]
    rotated_y += center[1]

    return [rotated_x, rotated_y]


def normalize_landmarks(landmarks):
    # 1. Obtener centro facial robusto
    face_center = get_face_center(landmarks)

    # 2. Obtener escala facial
    face_scale = get_face_scale(landmarks)

    # 3. Calcular ángulo de rotación
    rotation_angle = rotation_angle_func(landmarks)

    # 4. Convertir a coordenadas relativas al centro
    relative_landmarks = []
    for landmark in landmarks:
        relative_x = landmark[0] - face_center[0]
        relative_y = landmark[1] - face_center[1]
        relative_landmarks.append([relative_x, relative_y])

    # 5. Normalizar por escala (independiente de distancia a cámara)
    scaled_landmarks = []
    for landmark in relative_landmarks:
        scaled_x = landmark[0] / face_scale
        scaled_y = landmark[1] / face_scale
        scaled_landmarks.append([scaled_x, scaled_y])

    # 6. Rotar para corregir inclinación
    normalized_landmarks = []
    for landmark in scaled_landmarks:
        rotated_point = rotate_point(landmark, -rotation_angle)
        normalized_landmarks.append(rotated_point)

    return normalized_landmarks, face_center, face_scale, rotation_angle


def process_landmarks(landmarks):

    if len(landmarks) == 0:
        return [], None, None, None

    # Convertir a formato de lista si es necesario
    if hasattr(landmarks[0], 'x'):  # Si son objetos de MediaPipe
        landmarks_array = [[lm.x, lm.y] for lm in landmarks]
    else:
        landmarks_array = landmarks

    return normalize_landmarks(landmarks_array)


def get_specific_landmarks(normalized_landmarks, indices):

    if not normalized_landmarks:
        return []

    return [normalized_landmarks[i] for i in indices if i < len(normalized_landmarks)]


# Índices útiles para diferentes partes del rostro
FACE_OVAL = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379, 378, 400, 377, 152, 148, 176,
             149, 150, 136, 172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109]
LEFT_EYE = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]
RIGHT_EYE = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398]
LIPS = [61, 84, 17, 314, 405, 320, 307, 375, 321, 308, 324, 318]
NOSE = [1, 2, 5, 4, 19, 94, 125, 141, 235, 236, 3, 51, 48, 115, 131, 134, 102, 49, 220, 305, 281, 363, 355, 279, 331,
        294, 455]


# Ejemplo de uso con landmarks específicas
def get_facial_features_normalized(landmarks):
    
    normalized_landmarks, center, scale, angle = process_landmarks(landmarks)

    if not normalized_landmarks:
        return None

    features = {
        'face_oval': get_specific_landmarks(normalized_landmarks, FACE_OVAL),
        'left_eye': get_specific_landmarks(normalized_landmarks, LEFT_EYE),
        'right_eye': get_specific_landmarks(normalized_landmarks, RIGHT_EYE),
        'lips': get_specific_landmarks(normalized_landmarks, LIPS),
        'nose': get_specific_landmarks(normalized_landmarks, NOSE),
        'metadata': {
            'center': center,
            'scale': scale,
            'rotation_angle': math.degrees(angle)
        }
    }

    return features