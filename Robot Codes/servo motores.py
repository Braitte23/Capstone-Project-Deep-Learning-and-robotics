import RPi.GPIO as GPIO
import time
import socket
import threading

# =============================================================================
# PINES
# =============================================================================
# Pines para el Motor A
ENA = 10
IN1 = 8
IN2 = 9
# Pines para el Motor B
IN3 = 11
IN4 = 12
ENB = 6

# Pines del Servomotor
SERVO_PIN = 7
servo = None # Inicializado más adelante
servo_pos = 0 # Valor del GPIO.PWM (-1 a 1 aprox o 0° a 180°)

# =============================================================================
# VARIABLES
# =============================================================================
VELOCIDAD = 50 # % duty cycle
GPIO_LOW = 0
GPIO_HIGH = 1

# =============================================================================
# CONFIGURACIÓN
# =============================================================================
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
GPIO.setup([ENA, ENB], GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Configuración de PWM para control de velocidad
pwmA = GPIO.PWM(ENA, 1000) # 1 KHz
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(VELOCIDAD)
pwmB.start(VELOCIDAD)

# Configuración del Servomotor
servo = GPIO.PWM(SERVO_PIN, 50) # Frecuencia de 50 Hz para servos
servo.start(0)

# =============================================================================
# FUNCIONES DE MOVIMIENTO
# =============================================================================
print("Robot listo (servidor Wi-Fi activo).")

def adelante(velocidad):
    # Motor A: Adelante
    GPIO.output(IN1, GPIO_HIGH)
    GPIO.output(IN2, GPIO_LOW)
    # Motor B: Adelante
    GPIO.output(IN3, GPIO_HIGH)
    GPIO.output(IN4, GPIO_LOW)
    pwmA.ChangeDutyCycle(velocidad)
    pwmB.ChangeDutyCycle(velocidad)

def atras(velocidad):
    # Motor A: Atrás
    GPIO.output(IN1, GPIO_LOW)
    GPIO.output(IN2, GPIO_HIGH)
    # Motor B: Atrás
    GPIO.output(IN3, GPIO_LOW)
    GPIO.output(IN4, GPIO_HIGH)
    pwmA.ChangeDutyCycle(velocidad)
    pwmB.ChangeDutyCycle(velocidad)

def izquierda(velocidad):
    # Motor A: Atrás
    GPIO.output(IN1, GPIO_LOW)
    GPIO.output(IN2, GPIO_HIGH)
    # Motor B: Adelante
    GPIO.output(IN3, GPIO_HIGH)
    GPIO.output(IN4, GPIO_LOW)
    pwmA.ChangeDutyCycle(velocidad)
    pwmB.ChangeDutyCycle(velocidad)

def derecha(velocidad):
    # Motor A: Adelante
    GPIO.output(IN1, GPIO_HIGH)
    GPIO.output(IN2, GPIO_LOW)
    # Motor B: Atrás
    GPIO.output(IN3, GPIO_LOW)
    GPIO.output(IN4, GPIO_HIGH)
    pwmA.ChangeDutyCycle(velocidad)
    pwmB.ChangeDutyCycle(velocidad)

def detener():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO_LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

# =============================================================================
# CONTROL DE SERVO
# =============================================================================
global servo_pos

def servo_izquierda():
    global servo_pos
    servo_pos = max(-1, servo_pos - 0.1) # Limita el valor mínimo
    # Nota: servo_pos se usa para controlar la posición del servo
    servo.ChangeDutyCycle(servo_pos) 

def servo_derecha():
    global servo_pos
    servo_pos = min(1, servo_pos + 0.1) # Limita el valor máximo
    servo.ChangeDutyCycle(servo_pos)

def servo_valor(valor):
    global servo_pos
    servo_pos = valor # Asume que 'valor' es un valor de ciclo de trabajo PWM válido (e.g., 2.5 a 12.5 para 0° a 180° en 50Hz)
    servo.ChangeDutyCycle(servo_pos)

# =============================================================================
# CONTROL DE VELOCIDAD
# =============================================================================
def aumentar_velocidad():
    global VELOCIDAD
    VELOCIDAD = min(100, VELOCIDAD + 10)
    print(f"Velocidad: {VELOCIDAD}")

def disminuir_velocidad():
    global VELOCIDAD
    VELOCIDAD = max(0, VELOCIDAD - 10)
    print(f"Velocidad: {VELOCIDAD}")

# =============================================================================
# SERVIDOR TCP
# =============================================================================
HOST = '0.0.0.0' # Escucha en todas las interfaces
PORT = 5000

def servidor():
    global VELOCIDAD

    # Configuración del Socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Esperando conexión desde la PC...")

    try:
        conn, addr = server.accept()
        print(f"Conectado por {addr}")

        while True:
            # Recibir datos del cliente
            data = conn.recv(1024).decode().strip()
            if not data:
                break
            
            print(f"Comando recibido: {data}")

            # PASO DE DEPURACIÓN: mueve el servo a 0° al recibir cualquier comando
            # servo.value = -1 
            # sleep(0.5)

            # Ejecuta el comando
            if data == 'w':
                adelante(VELOCIDAD)
            elif data == 's':
                atras(VELOCIDAD)
            elif data == 'a':
                izquierda(VELOCIDAD)
            elif data == 'd':
                derecha(VELOCIDAD)
            elif data == 'x':
                detener()
            elif data == 'e':
                servo_derecha() # Control de servo
            elif data == 'q':
                servo_izquierda() # Control de servo
            elif data == 'h':
                aumentar_velocidad()
            elif data == 'l':
                disminuir_velocidad()
            
    except:
        pass
    finally:
        print("Cerrando conexión...")
        conn.close()
        server.close()
        detener()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()

# =============================================================================
# INICIO
# =============================================================================
if __name__ == "__main__":
    servidor()