import RPi.GPIO as GPIO
import time
import socket
import threading
import struct
import cv2

# =============================================================================
# CONFIGURACIÓN DE PINES
# =============================================================================
# Pines para el Motor A
ENA, IN1, IN2 = 12, 17, 16  # ENA (PWM), IN1, IN2 (Control)
# Pines para el Motor B
ENB, IN3, IN4 = 13, 22, 23  # ENB (PWM), IN3, IN4 (Control)

# Pines de los Sensores Infrarrojos
E1A, E1B = 4, 5
E2A, E2B = 6, 26
IR_IZQ = 19
IR_DER = 20

# Pines del Sensor Ultrasónico
TRIG = 24
ECHO = 25

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configuración de los pines como Salida/Entrada
for pin in [IN1, IN2, IN3, IN4, ENA, ENB, TRIG]:
    GPIO.setup(pin, GPIO.OUT)
for pin in [E1A, E1B, E2A, E2B, ECHO, IR_IZQ, IR_DER]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Configuración de PWM para control de velocidad
pwmA = GPIO.PWM(ENA, 100)  # Pin ENA, Frecuencia 100 Hz
pwmB = GPIO.PWM(ENB, 100)  # Pin ENB, Frecuencia 100 Hz
pwmA.start(0)
pwmB.start(0)

# =============================================================================
# VARIABLES
# =============================================================================
VELOCIDAD = 30  # % duty cycle inicial
OFFSET_B = 0    # Offset para el motor B (para corrección de trayectoria)
piloto_automatico = False

# =============================================================================
# MOVIMIENTO
# =============================================================================
def def adelante(v=VELOCIDAD):
    # Motor A: Adelante
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    # Motor B: Adelante
    GPIO.output(IN3, 0); GPIO.output(IN4, 1) # NOTA: La polaridad de IN3/IN4 de B está al revés respecto a A
    pwmA.ChangeDutyCycle(v + OFFSET_B)
    pwmB.ChangeDutyCycle(v)
    print(f"** Adelante | Vel {v}%")

def atras(v=VELOCIDAD):
    # Motor A: Atrás
    GPIO.output(IN1, 0); GPIO.output(IN2, 1)
    # Motor B: Atrás
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)
    pwmA.ChangeDutyCycle(v + OFFSET_B)
    pwmB.ChangeDutyCycle(v)
    print(f"** Atrás | Vel {v}%")

def izquierda(v=VELOCIDAD):
    # Motor A: Atrás (o Detenido)
    GPIO.output(IN1, 0); GPIO.output(IN2, 1) # Gira sobre su eje
    # Motor B: Adelante
    GPIO.output(IN3, 0); GPIO.output(IN4, 1)
    pwmA.ChangeDutyCycle(v)
    pwmB.ChangeDutyCycle(v)
    print(f"** Izquierda | Vel {v}%")

def derecha(v=VELOCIDAD):
    # Motor A: Adelante
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    # Motor B: Atrás (o Detenido)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0) # Gira sobre su eje
    pwmA.ChangeDutyCycle(v)
    pwmB.ChangeDutyCycle(v)
    print(f"** Derecha | Vel {v}%")

def detener():
    # Motor A: Detener
    GPIO.output(IN1, 0); GPIO.output(IN2, 0)
    # Motor B: Detener
    GPIO.output(IN3, 0); GPIO.output(IN4, 0)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    print(f"** Detenido")

# =============================================================================
# SENSORES
# =============================================================================
def medir_distancia():
    # Enviar pulso TRIG
    GPIO.output(TRIG, False)
    time.sleep(0.05)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Medir tiempo del pulso ECHO
    pulse_start = 0
    pulse_end = 0
    timeout = 0.05  # Tiempo máximo de espera
    time_start = time.time()

    # Esperar el inicio del pulso
    while GPIO.input(ECHO) == 0 and time.time() < time_start + timeout:
        pulse_start = time.time()

    # Esperar el final del pulso
    time_end = time.time()
    while GPIO.input(ECHO) == 1 and time.time() < time_end + timeout:
        pulse_end = time.time()

    # Calcular duración y distancia
    duracion = pulse_end - pulse_start
    # Distancia = Duración * Velocidad del Sonido (34300 cm/s) / 2
    # 34300 / 2 = 17150
    distancia = duracion * 17150
    return round(distancia, 2)

# =============================================================================
# PILOTO AUTOMÁTICO
# =============================================================================
def piloto():
    global global_velocidad
    while True:
        if piloto_automatico:
            ir_izq = GPIO.input(IR_IZQ)
            ir_der = GPIO.input(IR_DER)
            dist = medir_distancia()

            v_auto = max(10, global_velocidad)  # Mínimo 10% en automático

            if ir_izq == 0:
                derecha(v_auto)
            elif ir_der == 0:
                izquierda(v_auto)
            elif dist > 10:
                adelante(v_auto)
            else:
                detener()
            time.sleep(0.1)

# =============================================================================
# COMANDOS
# =============================================================================
def manejar_comando(c: str, global_piloto_automatico, velocidad):
    global piloto_automatico, VELOCIDAD

    c = c.strip().upper()

    if c == 'W':
        adelante(velocidad)
    elif c == 'S':
        atras(velocidad)
    elif c == 'A':
        izquierda(velocidad)
    elif c == 'D':
        derecha(velocidad)
    elif c == 'X':
        detener()
    elif c == 'P':
        piloto_automatico = not piloto_automatico
        estado = "ACTIVADO" if piloto_automatico else "DESACTIVADO"
        print(f"** Piloto automático {estado}")
    elif c == '+':
        VELOCIDAD = min(100, VELOCIDAD + 5)
        print(f"** Velocidad ↑ + {VELOCIDAD}%")
    elif c == '-':
        VELOCIDAD = max(10, VELOCIDAD - 5)
        print(f"** Velocidad ↓ - {VELOCIDAD}%")

# =============================================================================
# SERVIDOR DE COMANDOS
# =============================================================================
def servidor_comandos():
    host = "0.0.0.0"  # Escuchar en todas las interfaces
    port = 8000
    server_socket = socket.socket(AF_INET, SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"[CMD] Escuchando en {host}:{port}")

    try:
        conn, addr = server_socket.accept()
        print(f"[CMD] Conectado desde {addr}")

        while True:
            try:
                data = conn.recv(1024).decode('utf-8')
                if not data:
                    break
                for c in data.split(';'):
                    manejar_comando(c, piloto_automatico, VELOCIDAD)
            except:
                break # Error o conexión cerrada

    except:
        pass  # Manejo de error general (e.g., al iniciar el socket)
    finally:
        detener()
        conn.close()
        server_socket.close()

# =============================================================================
# SERVIDOR DE VIDEO
# =============================================================================
def servidor_video():
    # Captura de video
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Configuración del socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("0.0.0.0", 8001)) # Puerto diferente para video
    s.listen(1)
    print("[CAM] Esperando conexión de video...")

    try:
        conn, addr = s.accept()
        print(f"[CAM] Conectado desde {addr}")

        while True:
            ret, frame = cam.read()
            if not ret:
                time.sleep(0.05)
                continue

            # Convertir a JPEG para enviar menos datos
            # Parámetros: (.jpg, frame, [parámetro, valor_calidad])
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            data = buffer.tobytes()
            
            # Empaquetar el tamaño del mensaje (4 bytes, little-endian)
            message = struct.pack('<L', len(data)) + data
            
            # Enviar el mensaje completo (tamaño + datos)
            conn.sendall(message)

            time.sleep(0.01) # Pequeña pausa

    except Exception as e:
        print(f"[CAM] Error: {e}")
    finally:
        cam.release()
        conn.close()
        s.close()
        server_close() # Cerrar el socket del servidor de video

# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    hilo_cmd = threading.Thread(target=servidor_comandos)
    hilo_cam = threading.Thread(target=servidor_video)
    hilo_piloto = threading.Thread(target=piloto, daemon=True) # daemon=True para que termine con el main

    try:
        # Iniciar los hilos de ejecución
        hilo_cmd.start()
        hilo_cam.start()
        hilo_piloto.start()

        # Esperar a que los hilos principales terminen
        hilo_cmd.join()
        hilo_cam.join()
        hilo_piloto.join()

    except KeyboardInterrupt:
        print(f"\n[N] STOP | Interrupción manual")
    finally:
        detener()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
        print("Apagando robot y limpiando pines...")