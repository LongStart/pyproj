import apriltag
import cv2

if __name__ == "__main__":
    from sys import argv
    if len(argv) < 2:
        print("example: python tag_recognition.py path/to/img.png")
        quit()

    img_filename = argv[1]
    img = cv2.imread(img_filename, cv2.IMREAD_GRAYSCALE)
    options = apriltag.DetectorOptions(families='tag36h11',  
                                border=2,
                                nthreads=4,
                                quad_decimate=1.0,
                                quad_blur=0.0,
                                refine_edges=True,
                                refine_decode=False,
                                refine_pose=False,
                                debug=True,
                                quad_contours=True)
    detector = apriltag.Detector()
    result = detector.detect(img)
    print(result)

    