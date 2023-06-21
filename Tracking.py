import jetson.inference
import jetson.utils
import cv2
import time
import serial

# Mở kết nối với port ttyACM0, với baudrate là 115200 và timeout là 1 giây
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# khởi tạo model ssd-mobilenet-v2
net = jetson.inference.detectNet('ssd-mobilenet-v2', threshold=0.5)

# khởi tạo tracker CSRT
tracker = cv2.TrackerCSRT_create()

# mở file video
cap = cv2.VideoCapture('LC.mp4')

# Tắt commment đoạn bên dưới nếu dùng camera
'''
cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
'''

# đọc khung hình đầu tiên
ret, frame = cap.read()
bbox = None
# prev_cen_x = 0

# Chờ cho đến khi có người xuất hiện trong khung hình
while bbox is None:
    frame = cv2.resize(frame, (640, 360))
    cuda_img = jetson.utils.cudaFromNumpy(frame)

    # Xác định điểm giữa của chiều dài khung hình
    midpt_width = frame.shape[:1]
    # tìm kiếm người đầu tiên
    detections = net.Detect(cuda_img)

    # Nếu nhận dạng được người
    if len(detections) > 0:
        # lấy bounding box của người đầu tiên nhận dạng được
        bbox = detections[0].Left, detections[0].Top, detections[0].Width, detections[0].Height
        bbox = tuple(int(i) for i in bbox)
    else:
        # Đọc khung hình tiếp theo
        ret, frame = cap.read()

# khởi tạo tracker bằng bounding box của người đầu tiên
tracker.init(frame, bbox)


# Hàm tính toán màu áo của đối tượng
def get_color(h, s, v):
    if h >= 0 and h <= 9 and s >= 50 and v >= 50:
        return "RED"
    elif h >= 10 and h <= 21 and s >= 50 and v >= 50:
        return "ORANGE"
    elif h >= 22 and h <= 38 and s >= 50 and v >= 50:
        return "YELLOW"
    elif h >= 39 and h <= 50 and s >= 50 and v >= 50:
        return "BRIGHT GREEN"
    elif h >= 51 and h <= 90 and s >= 50 and v >= 50:
        return "GREEN"
    elif h >= 91 and h <= 135 and s >= 50 and v >= 50:
        return "BLUE"
    elif h >= 136 and h <= 160 and s >= 50 and v >= 50:
        return "SKY BLUE"
    elif h >= 161 and h <= 179 and s >= 50 and v >= 50:
        return "PURPLE"
    elif h >= 0 and h <= 9 and s >= 20 and s < 50 and v >= 50:
        return "BROWNISH RED"
    elif h >= 10 and h <= 21 and s >= 20 and s < 50 and v >= 50:
        return "BROWNISH ORANGE"
    elif h >= 22 and h <= 38 and s >= 20 and s < 50 and v >= 50:
        return "BROWNISH YELLOW"
    elif h >= 39 and h <= 50 and s >= 20 and s < 50 and v >= 50:
        return "BROWNISH GREEN"
    elif h >= 51 and h <= 90 and s >= 20 and s < 50 and v >= 50:
        return "OLIVE GREEN"
    elif h >= 91 and h <= 135 and s >= 20 and s < 50 and v >= 50:
        return "BROWNISH BLUE"
    elif h >= 136 and h <= 160 and s >= 20 and s < 50 and v >= 50:
        return "BROWNISH GRAYISH BLUE"
    elif h >= 161 and h <= 179 and s >= 20 and s < 50 and v >= 50:
        return "BROWNISH PURPLE"
    elif h >= 0 and h <= 179 and s >= 0 and s < 20 and v >= 50:
        return "GRAY"
    elif h >= 0 and h <= 179 and s >= 0 and s < 20 and v < 50:
        return "DARK GRAY"
    elif h >= 0 and h <= 9 and s >= 50 and v < 50:
        return "DARK RED"
    elif h >= 10 and h <= 21 and s >= 50 and v < 50:
        return "DARK ORANGE"
    elif h >= 22 and h <= 38 and s >= 50 and v < 50:
        return "DARK YELLOW"
    elif h >= 39 and h <= 50 and s >= 50 and v < 50:
        return "DARK GREEN"
    elif h >= 51 and h <= 90 and s >= 50 and v < 50:
        return "DARK OLIVE GREEN"
    elif h >= 91 and h <= 135 and s >= 50 and v < 50:
        return "DARK BLUE"
    elif h >= 136 and h <= 160 and s >= 50 and v < 50:
        return "DARK GRAYISH BLUE"
    elif h >= 161 and h <= 179 and s >= 50 and v < 50:
        return "DARK PURPLE"
    elif h >= 0 and h <= 179 and s >= 0 and s < 20 and v < 50:
        return "BLACK"
    elif h >= 0 and h <= 179 and s >= 0 and s < 20 and v >= 50:
        return "LIGHT GRAY"
    elif h >= 161 and h <= 179 and s >= 20 and s < 50 and v < 50:
        return "DARK BROWNISH PURPLE"
    elif h >= 161 and h <= 179 and s >= 20 and s < 50 and v >= 50:
        return "LIGHT BROWNISH PURPLE"
    elif h >= 136 and h <= 160 and s >= 20 and s < 50 and v < 50:
        return "DARK BROWNISH GRAYISH BLUE"
    elif h >= 136 and h <= 160 and s >= 20 and s < 50 and v >= 50:
        return "LIGHT BROWNISH GRAYISH BLUE"
    else:
        return "UNKNOWN COLOR"


# prev_time = 0

# Vòng lặp chương trình tracking
while cap.isOpened():
    # đọc khung hình tiếp theo
    ret, frame = cap.read()

    # Chuyển sang hệ HSV để nhận diện màu
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Resize khung hình để tăng tốc độ xử lí
    frame = cv2.resize(frame, (640, 360))
    if not ret:
        break

    # sử dụng tracker để ước tính vị trí người trong khung hình
    ok, bbox = tracker.update(frame)

    # Tọa độ trọng tâm của bbox
    cen_x = int(bbox[0] + bbox[2] / 2)
    cen_y = int(bbox[1] + bbox[3] / 2)

    # Tọa độ điểm trên áo để lấy pixel nhận diện màu
    cen_x_hsv = int(bbox[0] + bbox[2] / 3)
    cen_y_hsv = int(bbox[1] + bbox[3] / 3)

    # Pick pixel value
    pixel_center = hsv_frame[cen_y_hsv, cen_x_hsv]
    H = pixel_center[0]
    S = pixel_center[1]
    V = pixel_center[2]
    color = get_color(H, S, V)
    cv2.putText(frame, color, (10, 150), 0, 1, (0, 255, 255), 2)
    cv2.circle(frame, (cen_x_hsv, cen_y_hsv), 5, (0, 255, 255), 1)

    # khoảng lệch giữa tâm frame và tâm bbox (theo trục x) (đv: pixel)
    offset = midpt_width[0] - cen_x

    # nếu tracker hoạt động tốt, vẽ bounding box lên khung hình
    if ok:
        cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])),
                      (0, 255, 0), 2)
        cv2.circle(frame, (cen_x, cen_y), 5, (255, 0, 0), cv2.FILLED)
        if offset > 40:
            print("Turn Left: ", abs(offset))
            cv2.putText(frame, "Turn Left " + str(abs(offset)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2)
            # ser.write(b'L')
        elif offset < -40:
            print("Turn Right: ", abs(offset))
            cv2.putText(frame, "Turn Right " + str(abs(offset)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2)
            # ser.write(b'R')
        else:
            print("Go straight")
            cv2.putText(frame, "Go straight", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2)
            # ser.write(b'T')

        # Tinh huong di chuyen
        # if bbox == 0 and (cen_x-prev_cen_x) > 0:
        #     print("Moving to the Left")
        # elif bbox == 0 and (cen_x-prev_cen_x) < 0:
        #     print("Moving to the Right")

        # prev_cen_x = cen_x
        # current_time = time.time()
    # fps = 1 / (current_time - prev_time)
    # cv2.putText(frame, "FPS: "+str(int(fps)), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    # prev_time = current_time
    # hiển thị khung hình
    cv2.imshow('frame', frame)

    # thoát nếu nhấn phím 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# ser.close()
# giải phóng các tài nguyên
cap.release()
cv2.destroyAllWindows()
