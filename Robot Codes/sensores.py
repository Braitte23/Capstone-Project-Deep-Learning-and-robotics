import RPi.GPIO as GPIO
import time
import busio # Para la comunicación SPI
import board # Para los pines de la placa
# Importa las librerías del ADC MCP3008 y la interfaz analógica
from adafruit_mcp3xxx.mcp3008 import MCP3008
from adafruit_mcp3xxx.analog_in import AnalogIn

# =============================================================================
# CONFIGURACIÓN DE GPIOS Y SPI
# =============================================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pines del HC-SR04 (Ultrasónico)
TRIG = 24
ECHO = 23

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Configuración del MCP3008 (ADC)
# spi (busio.SPI): busio.SPI(clock=board.SCK, miso=board.MISO, mosi=board.MOSI)
spi = busio.SPI(clock=board.D18, MISO=board.MISO, MOSI=board.MOSI) # Chip Select en GPIO
cs = digitalio.DigitalInOut(board.D8) # Chip Select en GPIO8
mcp = MCP3008(spi, cs) # Inicializa el ADC

# Canales del MCP3008 donde conectas los Sharp
# Los sensores Sharp son analógicos y requieren el MCP3008 para ser leídos
chan_ir_izq = AnalogIn(mcp, board.D0) # MCP CH0
chan_ir_der = AnalogIn(mcp, board.D1) # MCP CH1

# =============================================================================
# FUNCIONES DE LECTURA
# =============================================================================
def leer_ultrasonico():
    """Mide la distancia usando el sensor ultrasónico HC-SR04."""
    # Enviar pulso TRIG
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    
    # Variables para el cálculo del tiempo
    pulso_inicio = time.time()
    pulso_fin = time.time()

    # Esperar el inicio del pulso
    while GPIO.input(ECHO) == 0:
        pulso_inicio = time.time()
        
    # Esperar el final del pulso
    while GPIO.input(ECHO) == 1:
        pulso_fin = time.time()

    # Cálculo de la distancia
    duracion = pulso_fin - pulso_inicio
    # Distancia = (Duración * Velocidad del Sonido (34300 cm/s)) / 2
    distancia = (duracion * 34300) / 2 # Distancia en cm
    return distancia

def voltaje_a_distancia_sharp(v):
    """
    Convierte la lectura de voltaje (V) de un sensor Sharp GP2Y0A21YK0F 
    a distancia en cm.
    """
    # Aproximación empírica del GP2Y0A21YK0F: V ~ 1/(distancia en cm)
    # Rango de medición es ~10cm a ~80cm

    if v <= 0.1:
        # Si el voltaje es muy bajo, está fuera de rango (muy lejos)
        return 80 # Retorna el máximo teórico
    
    # FÓRMULA AJUSTADA TÍPICA: Distancia = K1 / (V - K2)
    distancia = 27.86 / (v - 0.42)
    
    # Limitar el rango de distancia
    if distancia > 80:
        return 80
    elif distancia < 10:
        return 10
    else:
        return distancia
        
# =============================================================================
# LOOP PRINCIPAL
# =============================================================================
try:
    print("Iniciando prueba de sensores... (Ctrl+C para salir)\n")
    time.sleep(1)

    while True:
        # --- Lectura Ultrasónico ---
        dist_ultra = leer_ultrasonico()

        # --- Lectura Sensores Sharp (IR vía ADC) ---
        # 1. Leer el voltaje del canal analógico
        # .voltage retorna el valor de voltaje (0.0 a V_ref)
        volt_izq = chan_ir_izq.voltage
        volt_der = chan_ir_der.voltage
        
        # 2. Convertir el voltaje a distancia en cm
        dist_izq = voltaje_a_distancia_sharp(volt_izq)
        dist_der = voltaje_a_distancia_sharp(volt_der)
        
        # 3. Imprimir resultados
        print(f"Frente: {dist_ultra:5.1f} cm | Izq (V:{volt_izq:.2f} | D:{dist_izq:5.1f} cm) | Der (V:{volt_der:.2f} | D:{dist_der:5.1f} cm)")
        
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("\n[N] Finalizando prueba...")
finally:
    GPIO.cleanup()