import cv2
import numpy as np
import time


def convert_hsv_gimp_cv2(hsv_range):
    h, s, v = hsv_range
    h = ((h - 240) % 360) // 2 % 179
    s = int(s / 100 * 255)
    v = int(v / 100 * 255)
    return h, s, v


def find_target(frame):
    cv2.imshow('input', frame)
    h, w, _ = frame.shape
    hsv_frame = cv2.cvtColor(frame.copy(), cv2.COLOR_RGB2HSV)

    # minS, minV = 20, 60
    # maxS, maxV = 100, 100
    # deltaH = 10
    # colorH = 250

    # красный цвет
    minS, minV = 50, 50
    maxS, maxV = 100, 100
    deltaH = 20
    colorH = 120
    color_min = convert_hsv_gimp_cv2((colorH - deltaH, minS, minV))
    color_max = convert_hsv_gimp_cv2((colorH + deltaH, maxS, maxV))
    # print(color_max, color_min)

    # синий
    # hsv_min = (0, 0, 240)
    # hsv_max = (179, 20, 250)

    # красный
    # hsv_min = (190, 0, 240)
    # hsv_max = (255, 20, 250)

    # target_mask_hsv = cv2.inRange(hsv_frame, hsv_min, hsv_max)
    # cv2.imshow('target hsv', target_mask_hsv)

    target_mask_gimp = cv2.inRange(hsv_frame, color_min, color_max)
    cv2.imshow('target mask', target_mask_gimp)

    target = cv2.bitwise_and(frame, frame, mask=target_mask_gimp)
    # target = cv2.cvtColor(target, cv2.COLOR_RGB2BGR)
    cv2.imshow('target', target)

    return target


if __name__ == '__main__':
    """
    Скрипт для калибровки и поиска цвета для детектирования целей
    """
    frame = cv2.imread('D://Github//cat.jpg')
    is_video = True

    if not is_video:
        while True:
            frame = cv2.resize(frame, (600, 400))
            find_target(frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            if key == ord('r'):
                roi = cv2.selectROI(frame)
                imCrop = frame[int(roi[1]):int(roi[1] + roi[3]), int(roi[0]):int(roi[0] + roi[2])]
                hsv_imgCrop = cv2.cvtColor(imCrop, cv2.COLOR_RGB2HSV)
                print(np.sort(np.unique(hsv_imgCrop[:,:,0])))

    if is_video:
        video_stream = cv2.VideoCapture(0)

        start_time = time.time()
        num_frames = 0
        tracker = cv2.TrackerBoosting_create()
        while True:
            szX, szY = 10, 10
            ret, frame = video_stream.read()
            # frame = cv2.resize(frame, (700, 530), interpolation=cv2.INTER_CUBIC)
            frameRet = frame.copy()
            h, w, _ = frame.shape
            cWidth, cHeight = w // 2, h // 2
            frame_center = cWidth, cHeight

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            if key == ord('r'):
                roi = cv2.selectROI(frame)
                imCrop = frame[int(roi[1]):int(roi[1] + roi[3]), int(roi[0]):int(roi[0] + roi[2])]
                hsv_imgCrop = cv2.cvtColor(imCrop, cv2.COLOR_RGB2HSV)
                print('H', np.sort(np.unique(hsv_imgCrop[:, :, 0])))
                print('S', np.sort(np.unique(hsv_imgCrop[:, :, 1])))
                print('V', np.sort(np.unique(hsv_imgCrop[:, :, 2])))

            target_frame = find_target(frameRet)
            cv2.imshow('input', frameRet)

            num_frames = num_frames + 1 if num_frames < 10000 else 0

        seconds = time.time() - start_time
        fps = num_frames / seconds
        print("[INFO] elapsed time: {:.2f}".format(seconds))
        print("[INFO] approx. FPS: {:.2f}".format(fps))

        cv2.destroyAllWindows()
        video_stream.release()
