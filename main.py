from cvzone.FaceMeshModule import FaceMeshDetector
import cv2
import serial
import time
import processing as pro
import facebank as fb


# Comunicacion serial con timeout
ser = serial.Serial('COM6', 115200, timeout=0.01)  # timeout muy pequeño para no bloquear

# Inicializa la camara
cap = cv2.VideoCapture(1)

# Modelo de rostro
detector = FaceMeshDetector(staticMode=False, maxFaces=1, minDetectionCon=0.5, minTrackCon=0.5)

# Temporizador
last_send_time = time.time()
last_activation_time = 0  # Tiempo de la última activación
process_face = False  # Flag para controlar cuándo procesar rostros
faces = []  # Inicializar faces
activation_timeout = 2.0  # Timeout en segundos sin señal
# NUEVA VARIABLE para controlar guardado manual con tecla 'a'
save_next_face = False

print("Starting - Waiting for activation signal...")

while True:
    success, img = cap.read()


    if ser.in_waiting > 0:  # Solo lee si hay datos disponibles
        try:
            received_data = ser.read(1)
            if received_data == b'a':
                process_face = True
                print("Received activation signal - Starting face detection")
        except:
            print("Error reading serial data")
    else:
        process_face = False
        print("No signal - Stopping face detection")

    # Solo procesar rostros si está activado
    if process_face:
        img, faces = detector.findFaceMesh(img, draw=True)
    else:
        faces = []

    current_time = time.time()

    if current_time - last_send_time >= 1 and process_face:

        if faces:
            try:
                # Procesar landmarks con normalización
                normalized_landmarks, face_center, face_scale, rotation_angle = pro.process_landmarks(faces[0])

                if normalized_landmarks:

                    error = fb.find_lowest_error(normalized_landmarks)

                    if error * 10000 < 10:
                        ser.write(b'a')
                        print("a - Rostro reconocido")
                        print(f"Error: {error:.6f}")

                        if face_center:
                            print(f"Rotación: {rotation_angle * 180 / 3.14159:.1f}°")
                            print(f"Escala: {face_scale:.4f}")
                    else:
                        print("b - Rostro no reconocido")
                        print(f"Error: {error:.6f}")
                        ser.write(b'b')

                        # SI SE SOLICITÓ GUARDAR ROSTRO DESCONOCIDO
                        if save_next_face:
                            print("Guardando rostro no reconocido...")
                            cv2.putText(img, "Guardando rostro...", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                        (255, 255, 0), 2)

                            metadata = {
                                'scale': face_scale,
                                'rotation_angle': rotation_angle
                            }
                            fb.append_normalized_matrix_to_csv(normalized_landmarks, metadata=metadata)
                            save_next_face = False  # Resetear bandera

                else:
                    print("Error al normalizar landmarks")
                    ser.write(b'b')

            except Exception as e:
                print(f"Error procesando rostro: {e}")
                ser.write(b'b')
        else:
            print("b - No se detectó rostro")
            ser.write(b'b')

        last_send_time = current_time

    # Mostrar estado en la ventana
    status_text = "ACTIVO" if process_face else "INACTIVO"
    cv2.putText(img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (0, 255, 0) if process_face else (0, 0, 255), 2)

    cv2.imshow("Image", img)
    # Checar si el usuario presiona 'a' para guardar el próximo rostro no reconocido
    key = cv2.waitKey(1) & 0xFF
    if key == ord('a'):
        print("Se presionó 'a' - Próximo rostro no reconocido será guardado")
        save_next_face = True
    elif key == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
ser.close()
print("Program ended")