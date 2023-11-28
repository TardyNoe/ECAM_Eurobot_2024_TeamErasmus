import cv2
import numpy as np

class Straightener:
    def findCorrespondance(self,imgPoint,objpoint):
        imgPoint = np.array(imgPoint)
        objpoint = np.array(objpoint)
        points1 = np.array([imgPoint[:, 0], imgPoint[:, 1]])
        w1 = np.array([objpoint[:, 0], objpoint[:, 1]])
        Mat1 = np.array([
            [w1[0][0],w1[1][0],1,0,0,0,-points1[0][0]*w1[0][0],-points1[0][0]*w1[1][0]],
            [0,0,0,w1[0][0],w1[1][0],1,-points1[1][0]*w1[0][0],-points1[1][0]*w1[1][0]],
            [w1[0][1],w1[1][1],1,0,0,0,-points1[0][1]*w1[0][1],-points1[0][1]*w1[1][1]],
            [0,0,0,w1[0][1],w1[1][1],1,-points1[1][1]*w1[0][1],-points1[1][1]*w1[1][1]],
            [w1[0][2],w1[1][2],1,0,0,0,-points1[0][2]*w1[0][2],-points1[0][2]*w1[1][2]],
            [0,0,0,w1[0][2],w1[1][2],1,-points1[1][2]*w1[0][2],-points1[1][2]*w1[1][2]],
            [w1[0][3],w1[1][3],1,0,0,0,-points1[0][3]*w1[0][3],-points1[0][3]*w1[1][3]],
            [0,0,0,w1[0][3],w1[1][3],1,-points1[1][3]*w1[0][3],-points1[1][3]*w1[1][3]],
        ])
        vect1 = np.array([points1[0][0],points1[1][0],points1[0][1],points1[1][1],points1[0][2],points1[1][2],points1[0][3],points1[1][3]])
        invMat1 = np.linalg.inv(np.matrix(Mat1))
        M1 = invMat1 @ vect1
        M1 = np.array(M1)[0]

        M1_matrix = np.array([[M1[0], M1[1], M1[2]],
                            [M1[3], M1[4], M1[5]],
                            [M1[6], M1[7], 1]])

        correspondance1 = np.zeros((600, 400,2), dtype=np.uint32)

        for y in range(600):
            for x in range(400):
                pixel = np.array([x / 200, y / 200, 1])
                transformed_pixel = np.dot(M1_matrix, pixel)
                transformed_pixel /= transformed_pixel[2]
                correspondance1[y, x] = [int(transformed_pixel[1]), int(transformed_pixel[0])]
        return correspondance1
    def computeImage(self,frame,correspondance):
        y_indices1 = correspondance[:, :, 0]
        x_indices1 = correspondance[:, :, 1]
        y_indices1 = np.clip(y_indices1, 0, frame.shape[0] - 1)
        x_indices1 = np.clip(x_indices1, 0, frame.shape[1] - 1)
        return frame[y_indices1, x_indices1]