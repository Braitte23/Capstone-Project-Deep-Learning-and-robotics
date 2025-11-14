import torch
import clip
from PIL import Image
import cv2
import numpy as np

# ========================
# Configuración inicial
# ========================
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)
print("CLIP cargado correctamente")

# Etiquetas
text = clip.tokenize(["un bebé de plástico", "nada"]).to(device)

# Cámara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir frame a PIL para CLIP
    img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    image = preprocess(img_pil).unsqueeze(0).to(device)

    with torch.no_grad():
        logits_per_image, _ = model(image, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()[0]

    prob_bebe, prob_nada = probs

    # Decidir etiqueta y color
    if prob_bebe > prob_nada:
        label = "OBJETO DETECTADO"
        color = (0, 255, 0)  # verde
        # Dibujar cuadro alrededor de toda la imagen
        cv2.rectangle(frame, (5, 5), (frame.shape[1]-5, frame.shape[0]-5), color, 4)
    else:
        label = "NO OBJETO"
        color = (0, 0, 255)  # rojo

    # Escribir texto en pantalla
    cv2.putText(frame, f"{label} ({prob_bebe*100:.1f}%)", (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # Mostrar frame
    cv2.imshow("CLIP detector", frame)

    # Salir con Q
    if cv2.waitKey(1) & 0xFF == ord('o'):
        break

cap.release()
cv2.destroyAllWindows()
