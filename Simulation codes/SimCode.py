from controller import Robot, Motor, Keyboard, DistanceSensor, Camera
import time
import os
import torch
import clip
import numpy as np
from PIL import Image
import cv2

# ==============================
# CONSTANTES
# ==============================
TIME_STEP = 64
MAX_ROBOT_SPEED = 5.0
OBSTACLE_THRESHOLD_CM = 20  

# Nombres de dispositivos
LEFT_MOTOR_NAME = "motor1"
RIGHT_MOTOR_NAME = "motor2"
IR1 = "sensorIR1"
IR2 = "sensorIR2"
ULTRASONIC = "ultrasonico2"
CAMERA_NAME = "camara"

# Carpeta para guardar imágenes
SAVE_PATH = r"C:\Users\bryan\Desktop\codigos robot\prototipo"
os.makedirs(SAVE_PATH, exist_ok=True)

# Procesar CLIP cada N frames
CLIP_INTERVAL = 10

class RobotController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = TIME_STEP
        self.autopilot_enabled = False
        self.capture_count = 0
        self.frame_count = 0
        self.FORWARD_SPEED = -MAX_ROBOT_SPEED 

        self._initialize_motors()
        self._initialize_sensors()
        self._initialize_camera()
        self._initialize_keyboard()

        # CLIP
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32", device=self.device)
        self.text_labels = ["cubo verde", "nada"]
        self.text_tokens = clip.tokenize(self.text_labels).to(self.device)

        print("CLIP cargado correctamente")
        print("Controlador iniciado")

    # Inicialización de motores
    def _initialize_motors(self):
        self.left_motor = self.robot.getDevice(LEFT_MOTOR_NAME)
        self.right_motor = self.robot.getDevice(RIGHT_MOTOR_NAME)
        for motor in [self.left_motor, self.right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

    # Inicialización de sensores
    def _initialize_sensors(self):
        self.sensorIR1 = self.robot.getDevice(IR1)
        self.sensorIR2 = self.robot.getDevice(IR2)
        self.sensorUl = self.robot.getDevice(ULTRASONIC)
        for s in [self.sensorIR1, self.sensorIR2, self.sensorUl]:
            s.enable(self.timestep)
        print("Sensores habilitados")

    # Inicialización de cámara
    def _initialize_camera(self):
        self.camera = self.robot.getDevice(CAMERA_NAME)
        self.camera.enable(self.timestep)
        print("Cámara habilitada")

    # Inicialización del teclado
    def _initialize_keyboard(self):
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

    # Guardar imagen
    def _capture_image(self):
        filename = os.path.join(SAVE_PATH, f"capture_{self.capture_count:04d}.jpg")
        self.camera.saveImage(filename, 100)
        print(f"Imagen guardada: {filename}")
        self.capture_count += 1

    # Lectura de sensores
    def _read_sensors(self):
        ir1_val = self.sensorIR1.getValue()
        ir2_val = self.sensorIR2.getValue()
        ultra_val_m = self.sensorUl.getValue()

        ir1_cm = max(0, 100 - ir1_val * 0.09)
        ir2_cm = max(0, 100 - ir2_val * 0.09)
        ultra_cm = ultra_val_m * 100  
        return ir1_cm, ir2_cm, ultra_cm

    # Evitación de obstáculos
    def _avoid_obstacles(self, ir1_cm, ir2_cm, ultra_cm):
        REVERSE_SPEED = MAX_ROBOT_SPEED
        FORWARD_SPEED = self.FORWARD_SPEED
        
        left_speed = FORWARD_SPEED
        right_speed = FORWARD_SPEED

        if ultra_cm < OBSTACLE_THRESHOLD_CM:
            left_speed = REVERSE_SPEED * 0.5 
            right_speed = REVERSE_SPEED * 0.5
            if ir1_cm > ir2_cm:
                left_speed = REVERSE_SPEED
                right_speed = REVERSE_SPEED * 0.5
            else:
                left_speed = REVERSE_SPEED * 0.5
                right_speed = REVERSE_SPEED
        elif ir1_cm < OBSTACLE_THRESHOLD_CM:
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED * 0.5
        elif ir2_cm < OBSTACLE_THRESHOLD_CM:
            left_speed = FORWARD_SPEED * 0.5
            right_speed = FORWARD_SPEED
        else:
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    # Control manual
    def _manual_control(self, key):
        left_speed = 0.0
        right_speed = 0.0

        if key == ord('W'):
            left_speed = self.FORWARD_SPEED 
            right_speed = self.FORWARD_SPEED
        elif key == ord('S'):
            left_speed = MAX_ROBOT_SPEED
            right_speed = MAX_ROBOT_SPEED
        elif key == ord('A'):
            left_speed = MAX_ROBOT_SPEED / 2
            right_speed = -MAX_ROBOT_SPEED / 2
        elif key == ord('D'):
            left_speed = -MAX_ROBOT_SPEED / 2
            right_speed = MAX_ROBOT_SPEED / 2
            
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    # Procesamiento CLIP
    def _process_clip(self, frame):
        w, h = self.camera.getWidth(), self.camera.getHeight()
        image = np.frombuffer(frame, np.uint8).reshape((h, w, 4))[:,:,:3]
        img_pil = Image.fromarray(image)
        image_input = self.clip_preprocess(img_pil).unsqueeze(0).to(self.device)

        with torch.no_grad():
            logits_per_image, _ = self.clip_model(image_input, self.text_tokens)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()[0]

        etiqueta = "OBJETO" if probs[0] > probs[1] else "NO OBJETO"
        return etiqueta, probs[0]

    # Mostrar resultado CLIP
    def _display_clip_result(self, frame, etiqueta, prob):
        w, h = self.camera.getWidth(), self.camera.getHeight()
        img = np.frombuffer(frame, np.uint8).reshape((h, w, 4))[:,:,:3]
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        if etiqueta == "OBJETO":
            cv2.rectangle(img_bgr, (50,50), (w-50,h-50), (0,255,0), 3)
            texto = f"{etiqueta} Conf: {prob*100:.1f}%"
            cv2.putText(img_bgr, texto, (10,50), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
        else:
            cv2.putText(img_bgr, "No object", (20,40), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),3)

        cv2.imshow("CLIP Detection", img_bgr)
        cv2.waitKey(1)

    # Bucle principal
    def run(self):
        while self.robot.step(self.timestep) != -1:
            key = self.keyboard.getKey()

            if key == ord('P'):
                self.autopilot_enabled = not self.autopilot_enabled
                mode = "AUTOMÁTICO" if self.autopilot_enabled else "MANUAL"
                print(f"Modo cambiado → {mode}")

            if key == ord('Q'):
                self._capture_image()

            if self.autopilot_enabled:
                ir1_cm, ir2_cm, ultra_cm = self._read_sensors()
                self._avoid_obstacles(ir1_cm, ir2_cm, ultra_cm)
            else:
                self._manual_control(key)

            self.frame_count += 1
            frame = self.camera.getImage()
            if frame and self.frame_count % CLIP_INTERVAL == 0:
                etiqueta, prob = self._process_clip(frame)
                self._display_clip_result(frame, etiqueta, prob)

# Ejecución
if __name__ == "__main__":
    controller = RobotController()
    controller.run()
