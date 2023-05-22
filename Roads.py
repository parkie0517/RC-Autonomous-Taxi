virtual_long_road_length = 310 # Constant number. Measurement of the 2d map's long road length in pixel
virtual_short_road_length = 310 # Constant number. Measurement of the 2d map's short road length in pixel
real_long_road_length = 150 # Constant number. Measurement of the real map's long road length in cm
real_short_road_length = 70 # Constant number. Measurement of the real map's short road length in cm

class Roads:
    virtualRoads = [
        # Y, X, direction, ID, virtual_road_length
        (80, 530, 4, 1, virtual_long_road_length), # 0 A
        (155, 225, 2, 2, virtual_long_road_length), # 1 B
        (80, 1060, 4, 3, virtual_long_road_length), # 2 C
        (155, 755, 2, 4, virtual_long_road_length), # 3 D
        (230, 80,  3, 5, virtual_short_road_length), # 4 E
        (530, 155, 1, 6, virtual_short_road_length), # 5 F
        (230, 610, 3, 7, virtual_short_road_length), # 6 G 
        (530, 685, 1, 8, virtual_short_road_length), # 7 H
        (230, 1140, 3, 9, virtual_short_road_length), # 8 I
        (530, 1215, 1, 10, virtual_short_road_length), # 9 J
        (610, 530, 4, 11, virtual_long_road_length), # 10 K
        (685, 225, 2, 12, virtual_long_road_length), # 11 L
        (610, 1060, 4, 13, virtual_long_road_length), # 12 M
        (685, 755, 2, 14, virtual_long_road_length), # 13 N
        (760, 80, 3, 15, virtual_short_road_length), # 14 O
        (1065, 155, 1, 16, virtual_short_road_length), # 15 P
        (760, 610, 3, 17, virtual_short_road_length), # 16 Q
        (1065, 685, 1, 18, virtual_short_road_length), # 17 R 
        (760, 1140, 3, 19, virtual_short_road_length), # 18 S
        (1065, 1215, 1, 20, virtual_short_road_length), # 19 T
        (1140, 530, 4, 21, virtual_long_road_length), # 20 U
        (1215, 225, 2, 22, virtual_long_road_length), # 21 V
        (1140, 1060, 4, 23, virtual_long_road_length), # 22 W
        (1215, 755, 2, 24, virtual_long_road_length) # 23 X
    ]

    realRoads = [
        # Y, X, direction, ID, real_road_length
        (80, 530, 4, 1, real_long_road_length), # 0 A
        (155, 225, 2, 2, real_long_road_length), # 1 B
        (80, 1060, 4, 3, real_long_road_length), # 2 C
        (155, 755, 2, 4, real_long_road_length), # 3 D
        (230, 80,  3, 5, real_short_road_length), # 4 E
        (530, 155, 1, 6, real_short_road_length), # 5 F
        (230, 610, 3, 7, real_short_road_length), # 6 G 
        (530, 685, 1, 8, real_short_road_length), # 7 H
        (230, 1140, 3, 9, real_short_road_length), # 8 I
        (530, 1215, 1, 10, real_short_road_length), # 9 J
        (610, 530, 4, 11, real_long_road_length), # 10 K
        (685, 225, 2, 12, real_long_road_length), # 11 L
        (610, 1060, 4, 13, real_long_road_length), # 12 M
        (685, 755, 2, 14, real_long_road_length), # 13 N
        (760, 80, 3, 15, real_short_road_length), # 14 O
        (1065, 155, 1, 16, real_short_road_length), # 15 P
        (760, 610, 3, 17, real_short_road_length), # 16 Q
        (1065, 685, 1, 18, real_short_road_length), # 17 R 
        (760, 1140, 3, 19, real_short_road_length), # 18 S
        (1065, 1215, 1, 20, real_short_road_length), # 19 T
        (1140, 530, 4, 21, real_long_road_length), # 20 U
        (1215, 225, 2, 22, real_long_road_length), # 21 V
        (1140, 1060, 4, 23, real_long_road_length), # 22 W
        (1215, 755, 2, 24, real_long_road_length) # 23 X
    ]
    
    def getVirtualRoads(self, num):
        return Roads.virtualRoads[num]
    
    def getRealRoads(self, num):
        return Roads.realRoads[num]