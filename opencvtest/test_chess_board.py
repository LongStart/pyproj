import cv2 as cv
import sys
import glob

if __name__ == "__main__":
    if(len(sys.argv) < 2):
        print("Please input bag file name")
        # print("Example: python {} path/*.pgm".format(sys.argv[0]))
        quit()
    imgs = sys.argv[1:]
    # board_size = (12,20)
    board_size = (20, 12)
    for img_name in imgs:

        img = cv.imread(img_name)

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = cv.bitwise_not(gray)

        flags = (cv.CALIB_CB_FAST_CHECK |
                cv.CALIB_CB_ADAPTIVE_THRESH |
                cv.CALIB_CB_FILTER_QUADS |
                cv.CALIB_CB_NORMALIZE_IMAGE)
        found, corners = cv.findChessboardCorners(gray, board_size, flags=flags)
        print(found)
        # print(len(corners))
        for c in corners:
            print(c)
        ret = 1
        cv.drawChessboardCorners(img, board_size, corners, ret)
        cv.imshow('asdf', img)
        cv.waitKey(0)