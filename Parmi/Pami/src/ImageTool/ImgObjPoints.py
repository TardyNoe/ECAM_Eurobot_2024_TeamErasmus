import numpy as np
import cv2
from itertools import combinations
                 
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

class ObjImg:
    def __init__(self, refImage, realHeight):
        refImage = cv2.cvtColor(refImage,cv2.COLOR_BGR2GRAY)
        self.refImage = refImage
        self.realHeight = realHeight
        scale = realHeight/refImage.shape[0]
        self.realCorners, self.realIds, rejected_img_points1 = detector.detectMarkers(refImage)
        self.realCorners = np.array(self.realCorners)*scale
    def getImgpointsAndObjpoints(self,image):
        corners, ids, rejected_img_points1 = detector.detectMarkers(image)
        self.detectedtags = len(ids)
        points = []
        for c in corners:
            for i in c[0]:
                points.append(i)
        best4Points = self.find_combination(points)
        objPoint = []
        imgPoint = []
        for i in best4Points:
            imgPoint.append([int(i[0]),int(i[1])])
            id,order = self.find_point_id_and_order(corners,ids,i)
            objPoint.append(self.get_coordinate(self.realCorners, self.realIds,id,order-1).tolist())
        return imgPoint,objPoint

    def calculate_center(self,points):
        return np.mean(points, axis=0)

    def distance(self,point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def find_combination(self,points):
        center = self.calculate_center(points)
        points_sorted = sorted(points, key=lambda point: self.distance(point, center), reverse=True)

        best_combination = None
        max_distance_sum = 0

        for combination in combinations(points_sorted[:10], 4):
            distance_sum = sum(self.distance(p1, p2) for p1, p2 in combinations(combination, 2))

            if distance_sum > max_distance_sum:
                max_distance_sum = distance_sum
                best_combination = combination

        return best_combination

    def find_point_id_and_order(self,points_groups, ids, coordinate):
        for i, group in enumerate(points_groups):
            for j, point in enumerate(group[0]):
                if np.all(point == coordinate):
                    return ids[i], j + 1 
        return None, None
    def get_coordinate(self,points_groups,ids,group_id, order):
        group_index = np.where(ids == group_id)[0]
        if len(group_index) == 0:
            return None
        group = points_groups[group_index[0]]
        if order < 0 or order >= group.shape[1]:
            return None
        return group[0, order]
