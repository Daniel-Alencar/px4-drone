import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import cv2
import numpy as np
import time

def identify_shape(approx):
    sides = len(approx)
    if sides == 3:
        return "Triangulo"
    elif sides == 4:
        return "Retangulo/Quadrado"
    elif sides == 5:
        return "Pentagono"
    elif sides == 6:
      return "Hexagono"
    elif sides == 10:
      return "Estrela"
    elif sides == 12:
      return "Cruz"
    elif sides > 12:
        return "Circulo"
    return "Desconhecido"

def setLimitsOfTrackbar():
    hue = {}
    hue["min"] = cv2.getTrackbarPos("Min Hue", trackbarWindow)
    hue["max"] = cv2.getTrackbarPos("Max Hue", trackbarWindow)
    
    if hue["min"] > hue["max"]:
        cv2.setTrackbarPos("Max Hue", trackbarWindow, hue["min"])
        hue["max"] = cv2.getTrackbarPos("Max Hue", trackbarWindow)
    
    sat = {}
    sat["min"] = cv2.getTrackbarPos("Min Saturation", trackbarWindow)
    sat["max"] = cv2.getTrackbarPos("Max Saturation", trackbarWindow)
    
    if sat["min"] > sat["max"]:
        cv2.setTrackbarPos("Max Saturation", trackbarWindow, sat["min"])
        sat["max"] = cv2.getTrackbarPos("Max Saturation", trackbarWindow)

    val = {}
    val["min"] = cv2.getTrackbarPos("Min Value", trackbarWindow)
    val["max"] = cv2.getTrackbarPos("Max Value", trackbarWindow)
    
    if val["min"] > val["max"]:
        cv2.setTrackbarPos("Max Value", trackbarWindow, val["min"])
        val["max"] = cv2.getTrackbarPos("Max Value", trackbarWindow)
        
    return hue, sat, val

def computeTracking(frame, hue, sat, val):
    
    #transforma a imagem de RGB para HSV
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #definir os intervalos de cores que vão aparecer na imagem final
    lowerColor = np.array([hue['min'], sat["min"], val["min"]])
    upperColor = np.array([hue['max'], sat["max"], val["max"]])
    
    #marcador pra saber se o pixel pertence ao intervalo ou não
    mask = cv2.inRange(hsvImage, lowerColor, upperColor)
    
    #aplica máscara que "deixa passar" pixels pertencentes ao intervalo, como filtro
    result = cv2.bitwise_and(frame, frame, mask = mask)
    
    #aplica limiarização
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _,gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    
    #encontra pontos que circundam regiões conexas (contour)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Filtra contornos pequenos
    min_area = 500  # Ajuste conforme necessário
    contours = [c for c in contours if cv2.contourArea(c) > min_area]
    
    #se existir contornos 
    maxArea = 0.0   
    cntMaxArea = None
    vertices = 0
    if contours:
      #retornando a área do primeiro grupo de pixels brancos
      maxArea = cv2.contourArea(contours[0])
      contourMaxAreaId = 0
      i = 0
      
      #para cada grupo de pixels branco
      for cnt in contours:
        #procura o grupo com a maior área
        if maxArea < cv2.contourArea(cnt):
          maxArea = cv2.contourArea(cnt)
          contourMaxAreaId = i
        i += 1

      approx = cv2.approxPolyDP(
        contours[contourMaxAreaId], 
        0.02 * cv2.arcLength(contours[contourMaxAreaId], True), 
        True
      )

      # Identifica a forma com base no número de vértices
      vertices = len(approx)

      # Desenha o contorno e a forma aproximada
      cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
          
      #achei o contorno com maior área em pixels
      cntMaxArea = contours[contourMaxAreaId]
    
    return frame, gray, maxArea, cntMaxArea, vertices

origin = "Triangulo"
destiny = "Circulo"

# Definir faixa de cores HSV para segmentação
name_values = [
  "Triangulo",
  "Circulo",
  "Estrela",
  "Hexagono",
  "Quadrado",
  "Casa",
  "Cruz",
  "Pentagono"
]

index_values = -1
for i in range(len(name_values)):
  if(name_values[i] == destiny):
    index_values = i

values = [
  # 0 Triângulo: 101, 95, 179
  {
    "hue_min": 82, "hue_max": 143, 
    "sat_min": 103, "sat_max": 255, 
    "val_min": 72, "val_max": 171
  },
  # 1 Círculo: 88, 77, 230
  {
    "hue_min": 88, "hue_max": 134, 
    "sat_min": 237, "sat_max": 255, 
    "val_min": 207, "val_max": 255
  },
  # 2 Estrela: 40, 60, 157
  {
    "hue_min": 46, "hue_max": 97, 
    "sat_min": 39, "sat_max": 174, 
    "val_min": 72, "val_max": 157
  },
  # 3 Hexagono: 0, 143, 208
  {
    "hue_min": 118, "hue_max": 191, 
    "sat_min": 113, "sat_max": 255, 
    "val_min": 82, "val_max": 255
  },
  # 4 Quadrado: 98, 20, 23
  {
    "hue_min": 101, "hue_max": 137, 
    "sat_min": 46, "sat_max": 131, 
    "val_min": 44, "val_max": 122
  },
  # 5 Casa: 0, 121, 153
  {
    "hue_min": 123, "hue_max": 181, 
    "sat_min": 74, "sat_max": 207, 
    "val_min": 95, "val_max": 193
  },
  # 6 Cruz: 133, 43, 168
  {
    "hue_min": 141, "hue_max": 184, 
    "sat_min": 26, "sat_max": 189, 
    "val_min": 136, "val_max": 255
  },
  # 7 Pentagono: 133, 105, 67
  {
    "hue_min": 123, "hue_max": 181, 
    "sat_min": 74, "sat_max": 207, 
    "val_min": 95, "val_max": 193
  },
]

# Inicializa as trackbars com valores do SET_1
def setTrackbarValues(values):
    cv2.setTrackbarPos("Min Hue", trackbarWindow, values["hue_min"])
    cv2.setTrackbarPos("Max Hue", trackbarWindow, values["hue_max"])
    cv2.setTrackbarPos("Min Saturation", trackbarWindow, values["sat_min"])
    cv2.setTrackbarPos("Max Saturation", trackbarWindow, values["sat_max"])
    cv2.setTrackbarPos("Min Value", trackbarWindow, values["val_min"])
    cv2.setTrackbarPos("Max Value", trackbarWindow, values["val_max"])

trackbarWindow = "trackbar window"
cv2.namedWindow(trackbarWindow)

def onChange(val):
    return

cv2.createTrackbar("Min Hue", trackbarWindow, 0, 255, onChange)
cv2.createTrackbar("Max Hue", trackbarWindow, 255, 255, onChange)

cv2.createTrackbar("Min Saturation", trackbarWindow, 0, 255, onChange)
cv2.createTrackbar("Max Saturation", trackbarWindow, 255, 255, onChange)

cv2.createTrackbar("Min Value", trackbarWindow, 0, 255, onChange)
cv2.createTrackbar("Max Value", trackbarWindow, 255, 255, onChange)

min_hue = cv2.getTrackbarPos("Min Hue", trackbarWindow)
max_hue = cv2.getTrackbarPos("Max Hue", trackbarWindow)

min_sat = cv2.getTrackbarPos("Min Saturation", trackbarWindow)
max_sat = cv2.getTrackbarPos("Max Saturation", trackbarWindow)

min_val = cv2.getTrackbarPos("Min Value", trackbarWindow)
max_val = cv2.getTrackbarPos("Max Value", trackbarWindow)


debug = False

cap = cv2.VideoCapture(0)
# Cria uma única janela nomeada antes do loop
cv2.namedWindow('Segmentação', cv2.WINDOW_NORMAL)
cv2.namedWindow('Detecção de Formas', cv2.WINDOW_NORMAL)

def detect_shape(frame):
    
    allMaxArea = 0.0
    allFrame = None
    allGray = None
    allIndex = 0
    allCntMaxArea = None
    allVertices = 0

    setTrackbarValues(values[index_values])
    
    hue, sat, val = setLimitsOfTrackbar()
    frame, gray, maxArea, cntMaxArea, vertices = computeTracking(
      frame, hue, sat, val
    )

    if allMaxArea <= maxArea:
      allMaxArea = maxArea

      allFrame = frame
      allGray = gray
      allIndex = index_values
      allCntMaxArea = cntMaxArea
      allVertices = vertices

    #retorna um retângulo que envolve o contorno em questão
    xRect, yRect, wRect, hRect = cv2.boundingRect(allCntMaxArea)

    #desenha caixa envolvente com espessura 3
    cv2.rectangle(
      allFrame, (xRect, yRect), 
      (xRect + wRect, yRect + hRect), 
      (0, 0, 255), 2
    )
    cv2.putText(
      allFrame, f"{name_values[allIndex]} - vertices: {allVertices}", 
      (xRect, yRect - 10), 
      cv2.FONT_HERSHEY_SIMPLEX, 
      0.6, (0, 0, 255), 2
    )
  
    cv2.imshow("Segmentação", allGray)
    cv2.imshow("Detecção de Formas", allFrame)

    return xRect, yRect, wRect, hRect

def destroy_windows():
  cap.release()
  cv2.destroyAllWindows()