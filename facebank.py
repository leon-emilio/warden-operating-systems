import csv
import numpy as np


def append_matrix_to_csv(matrix, filename='matrix.csv'):
    # Guarda matriz de landmarks (ahora soporta flotantes)

    # Se pasa a una matriz de numpy
    matrix = np.array(matrix)

    # Aplanar la matriz para guardar como una sola fila
    flattened = matrix.flatten()

    # Se hace append en el documento con formato flotante
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(flattened)


def append_normalized_matrix_to_csv(normalized_landmarks, metadata=None, filename='normalized_matrix.csv'):

    # Guarda landmarks normalizadas

    matrix = np.array(normalized_landmarks)
    flattened = matrix.flatten()
    # metadata del ajuste
    if metadata:
        row_data = list(flattened) + [metadata.get('scale', 0), metadata.get('rotation_angle', 0)]
    else:
        row_data = list(flattened)

    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(row_data)


def calculate_error(matrix1, matrix2):

    #Calcula error entre dos matrices usando distancia euclidiana entre los puntos

    matrix1 = np.array(matrix1, dtype=float)
    matrix2 = np.array(matrix2, dtype=float)

    # Asegurar que ambas matrices tienen la misma forma
    if matrix1.shape != matrix2.shape:
        print(f"Warning: Shape mismatch {matrix1.shape} vs {matrix2.shape}")
        return float('inf')

    # Aplanar las matrices si son 2D
    if len(matrix1.shape) > 1:
        matrix1 = matrix1.flatten()
        matrix2 = matrix2.flatten()

    # Calcular distancia euclidiana
    euclidean_distance = np.sqrt(np.sum((matrix1 - matrix2) ** 2))

    # Normalizar por el número de puntos para tener error promedio por landmark
    normalized_error = euclidean_distance / len(matrix1)

    return normalized_error


def find_lowest_error(current_matrix, filename='matrix.csv', use_normalized=True):
    #    Encuentra el menor error comparando con matrices almacenadas

    current_matrix = np.array(current_matrix, dtype=float)

    # Aplanar si es necesario
    if len(current_matrix.shape) > 1:
        current_matrix = current_matrix.flatten()

    # Usar archivo normalizado por defecto si existe
    if use_normalized:
        import os
        normalized_file = 'normalized_matrix.csv'
        if os.path.exists(normalized_file):
            filename = normalized_file

    try:
        with open(filename, 'r') as file:
            reader = csv.reader(file)
            lowest_error = float('inf')
            row_count = 0

            for row in reader:
                if not row:
                    continue

                try:
                    # Convertir a flotantes
                    stored_matrix = np.array([float(x) for x in row], dtype=float)

                    # Si hay metadata al final (scale, rotation), removerla para comparación
                    if use_normalized and len(stored_matrix) > len(current_matrix):
                        stored_matrix = stored_matrix[:len(current_matrix)]

                    # Asegurar que tienen el mismo tamaño
                    min_len = min(len(current_matrix), len(stored_matrix))
                    current_truncated = current_matrix[:min_len]
                    stored_truncated = stored_matrix[:min_len]

                    error = calculate_error(current_truncated, stored_truncated)

                    if error < lowest_error:
                        lowest_error = error

                    row_count += 1

                except (ValueError, IndexError) as e:
                    print(f"Error parsing row {row_count}: {e}")
                    continue

            print(f"Compared with {row_count} stored faces")
            print(f"Lowest error: {lowest_error:.4f}")
            return lowest_error

    except FileNotFoundError:
        print(f"File {filename} not found")
        return float('inf')
    except Exception as e:
        print(f"Error reading file: {e}")
        return float('inf')


def calculate_similarity_percentage(error, max_reasonable_error=1.0):
#    Convierte error a porcentaje de similitud

    if error >= max_reasonable_error:
        return 0.0

    similarity = (1 - (error / max_reasonable_error)) * 100
    return max(0.0, similarity)


def find_best_matches(current_matrix, filename='matrix.csv', top_n=3, use_normalized=True):

    #Encuentra las mejores coincidencias y sus similitudes

    current_matrix = np.array(current_matrix, dtype=float)

    if len(current_matrix.shape) > 1:
        current_matrix = current_matrix.flatten()

    if use_normalized:
        import os
        normalized_file = 'normalized_matrix.csv'
        if os.path.exists(normalized_file):
            filename = normalized_file

    matches = []

    try:
        with open(filename, 'r') as file:
            reader = csv.reader(file)

            for row_idx, row in enumerate(reader):
                if not row:
                    continue

                try:
                    stored_matrix = np.array([float(x) for x in row], dtype=float)

                    if use_normalized and len(stored_matrix) > len(current_matrix):
                        stored_matrix = stored_matrix[:len(current_matrix)]

                    min_len = min(len(current_matrix), len(stored_matrix))
                    current_truncated = current_matrix[:min_len]
                    stored_truncated = stored_matrix[:min_len]

                    error = calculate_error(current_truncated, stored_truncated)
                    similarity = calculate_similarity_percentage(error)

                    matches.append({
                        'row': row_idx,
                        'error': error,
                        'similarity': similarity
                    })

                except (ValueError, IndexError):
                    continue

        # Ordenar por menor error
        matches.sort(key=lambda x: x['error'])

        return matches[:top_n]

    except FileNotFoundError:
        return []

