import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def nothing(x):
    pass

class ColorTunerNode(Node):
    def __init__(self):
        super().__init__('color_tuner_node')
        
        # --- 1. CONFIGURAÇÃO ROS ---
        
        # Assinante (Lê a câmera do Gazebo)
        # Tópico original longo que você estava usando
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
            
        # Publicador (Envia a imagem processada de volta ao ROS)
        # Tópico novo: /camera/processed
        self.publisher_ = self.create_publisher(Image, '/camera/processed', 10)
        
        self.br = CvBridge()

        # --- 2. CONFIGURAÇÃO DA GUI (TRACKBARS) ---
        # Criamos uma janela específica para os controles
        cv2.namedWindow("Ajustes HSV")
        
        # Criar barras deslizantes para ajustar Min/Max de H, S e V
        # H (Matiz): 0-179, S (Saturação): 0-255, V (Valor/Brilho): 0-255
        cv2.createTrackbar("Low H", "Ajustes HSV", 20, 179, nothing)
        cv2.createTrackbar("High H", "Ajustes HSV", 40, 179, nothing)
        
        cv2.createTrackbar("Low S", "Ajustes HSV", 100, 255, nothing)
        cv2.createTrackbar("High S", "Ajustes HSV", 255, 255, nothing)
        
        cv2.createTrackbar("Low V", "Ajustes HSV", 100, 255, nothing)
        cv2.createTrackbar("High V", "Ajustes HSV", 255, 255, nothing)

        self.get_logger().info('Nó de Calibração iniciado! Use as janelas para ajustar a cor.')

    def image_callback(self, msg):
        try:
            # Converter ROS -> OpenCV
            current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f'Erro conversão: {e}')
            return

        # Converter BGR para HSV
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # --- 3. LEITURA DOS VALORES EM TEMPO REAL ---
        # Ler a posição atual das barras deslizantes
        l_h = cv2.getTrackbarPos("Low H", "Ajustes HSV")
        h_h = cv2.getTrackbarPos("High H", "Ajustes HSV")
        l_s = cv2.getTrackbarPos("Low S", "Ajustes HSV")
        h_s = cv2.getTrackbarPos("High S", "Ajustes HSV")
        l_v = cv2.getTrackbarPos("Low V", "Ajustes HSV")
        h_v = cv2.getTrackbarPos("High V", "Ajustes HSV")

        # Criar os arrays de limite com os valores lidos
        lower_bound = np.array([l_h, l_s, l_v])
        upper_bound = np.array([h_h, h_s, h_v])

        # Criar a máscara
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # --- 4. DETECÇÃO E DESENHO ---
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Vamos desenhar na imagem original
        processed_frame = current_frame.copy()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500: 
                x, y, w, h = cv2.boundingRect(contour)
                # Desenha o retângulo
                cv2.rectangle(processed_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Escreve os valores HSV na tela para facilitar sua anotação
                info_text = f"H:{l_h}-{h_h} S:{l_s} V:{l_v}"
                cv2.putText(processed_frame, info_text, (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --- 5. VISUALIZAÇÃO E PUBLICAÇÃO ---
        
        # Mostrar as janelas
        cv2.imshow("Camera Processada (Detectada)", processed_frame)
        cv2.imshow("Mascara (Binaria)", mask)
        cv2.imshow("Ajustes HSV", np.zeros((1, 400), np.uint8)) # Janela só para os sliders
        
        cv2.waitKey(1)

        # Converter OpenCV -> ROS e Publicar
        try:
            # Publica a imagem com o retângulo desenhado
            processed_msg = self.br.cv2_to_imgmsg(processed_frame, encoding="bgr8")
            # Copiar o header da mensagem original para manter o tempo sincronizado
            processed_msg.header = msg.header 
            
            self.publisher_.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Erro publicação: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()