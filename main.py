import progress
import cv2
from yolov5 import YOLOv5
import keyboard

mod_path = 'Demo/runs/train/exp9_ok/weights/best.pt'
Yolo_v5 = YOLOv5(mod_path)

def detect(cv2img, show_results = False):
    __img__ = cv2.cvtColor(cv2img, cv2.COLOR_BGR2RGB)
    __results__ = Yolo_v5.predict(__img__)
    __names__ = __results__.names
    predictions = __results__.pred[0]
    if 'cuda' in str(predictions.device):
        predictions = predictions.cpu()
    __boxes__ = predictions[:, :4].numpy().tolist()  # x1, x2, y1, y2
    __scores__ = predictions[:, 4].numpy().tolist()
    __categories__ = predictions[:, 5].numpy().tolist()
    if show_results:
        __results__.show()
        print(__results__)
        print(__boxes__)
        print(__scores__)
        print(__categories__)

    return __boxes__, __scores__, __categories__, __names__

def fruit(result):
    while not keyboard.is_pressed('esc'):
        img = progress.get_camera().read_cv2_image(strategy = "newest", timeout = 0.5)
        boxes, scores, categories, names = detect(img, show_results = False)
        flag = False
        for index, item in enumerate(boxes):
            pts = item
            t_class = int(categories[index])
            x1, y1, x2, y2 = list(map(int, pts))
            print(names[t_class], x1, y1, x2, y2)
            if names[t_class] == result:
                progress.seek(pos = x1, speed = 30)
                flag = True
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 3)
            cv2.putText(img, names[t_class], (x1, y1), cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale=2, color=(0, 0, 0),
                        thickness=1)
        cv2.imshow("EPCamera", img)
        if not flag:
            progress.seek(pos = 0, speed = 30)
        cv2.waitKey(1)

if __name__ == '__main__':
    progress.setup(kp = 30, ki = 10, kd = 50)
    fruit(progress.get_result())
    # progress.move(target_color = 'blue', base_speed = 70, start_angle = 180)
    # progress.recognize()
    # img = progress.get_image()
    # boxes, scores, categories, names = detect(img, show_results=False)
    # progress.seek(boxes, scores, categories, names, speed = 30)
    # for i in range(2):
    #     progress.move(base_speed = 20, start_angle = 0)
    #     progress.seek()
    #     progress.grab()
    #     progress.move(base_speed = 20, start_angle = 0)
    #     progress.place()
    # while not keyboard.is_pressed('esc'):
    #     pass
    progress.end()