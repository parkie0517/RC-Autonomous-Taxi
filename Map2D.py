import numpy as np

listRoad = []

class Roads:
    img_mapY = None
    img_mapX = None
    road_mapY = None
    road_mapX = None
    direction = None
    roadID = None

    def __init__(self, img_mapY, img_mapX, road_mapY, road_mapX, direction, roadID):
        Roads.img_mapY = img_mapY
        Roads.img_mapX = img_mapX
        Roads.road_mapY = road_mapY
        Roads.road_mapX = road_mapX
        Roads.direction = direction
        Roads.roadID = roadID

    def getImg_mapY(void):
        return Roads.img_mapY
    
    def getImg_mapX(void):
        return Roads.img_mapX
    
    def getRoad_mapY(void):
        return Roads.road_mapY
    
    def getRoad_mapX(void):
        return Roads.road_mapX
    
    def getDirection(void):
        return Roads.direction
    
    def getRoadID(void):
        return Roads.roadID

class RoadInfo:
    listRoad = []

    def __init__(void):
        listRoad.append(Roads(80, 530, 0, 2, 4, 1)) # 1
        listRoad.append(Roads(155, 225, 1, 2, 2, 2)) # 2
        listRoad.append(Roads(80, 1060, 0, 5, 4, 3)) # 3
        listRoad.append(Roads(155, 755, 1, 5, 2, 4)) # 4
        listRoad.append(Roads(230, 80, 2, 0, 3, 5)) # 5
        listRoad.append(Roads(530, 155, 2, 1, 1, 6)) # 6
        listRoad.append(Roads(230, 610, 2, 3, 3, 7)) # 7
        listRoad.append(Roads(530, 685, 2, 4, 1, 8)) # 8
        listRoad.append(Roads(230, 1140, 2, 6, 3, 9)) # 9
        listRoad.append(Roads(530, 1215, 2, 7, 1, 10)) # 10
        listRoad.append(Roads(610, 530, 3, 2, 4, 11)) # 11
        listRoad.append(Roads(685, 225, 4, 2, 2, 12)) # 12
        listRoad.append(Roads(610, 1060, 3, 5, 4, 13)) # 13
        listRoad.append(Roads(685, 755, 4, 5, 2, 14)) # 14
        listRoad.append(Roads(760, 80, 5, 0, 3, 15)) # 15
        listRoad.append(Roads(1065, 155, 5, 1, 1, 16)) # 16
        listRoad.append(Roads(760, 610, 5, 3, 3, 17)) # 17
        listRoad.append(Roads(1065, 685, 5, 4, 1, 18)) # 18
        listRoad.append(Roads(760, 1140, 5, 6, 3, 19)) # 19
        listRoad.append(Roads(1065, 1215, 5, 7, 1, 20)) # 20
        listRoad.append(Roads(1140, 530, 6, 2, 4, 21)) # 21
        listRoad.append(Roads(1215, 225, 7, 2, 2, 22)) # 22
        listRoad.append(Roads(1140, 1060, 6, 5, 4, 23)) # 23
        listRoad.append(Roads(1215, 755, 7, 5, 2, 24)) # 24

    def getInfo(self, num):
        return listRoad[num].getRoad_mapY()

roadInfo = RoadInfo()

print(roadInfo.getInfo(13))