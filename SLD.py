import numpy as np
import cv2
# implement SLD_planning in state 4 (1->1)
class SLD: #SLD means Stop Line Detecting
    lower_orange = np.array([0, 100, 100], dtype=np.uint8)
    upper_orange = np.array([22, 255, 255], dtype=np.uint8)
    lower_orange2 = np.array([160, 100, 100], dtype=np.uint8)
    upper_orange2 = np.array([180, 255, 255], dtype=np.uint8)
    """ delete these lines if they are unnecessary
    line_stop_threshold = 3 # const
    line_stop_num = 5 # const
    line_stop_iterate = 0
    line_stop_count = 0
    """
    threshold_line_stop_y = 550 # const
    flag_StopLine = False
    y = 0
    h = 0

    def SLD_perception(self, frame, hsv):
        orange_mask = cv2.inRange(hsv, SLD.lower_orange, SLD.upper_orange) + cv2.inRange(hsv, SLD.lower_orange2, SLD.upper_orange2)
        boxes, hierarchy=cv2.findContours(orange_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        SLD.y = 0
        SLD.h = 0
        SLD.flag_StopLine = False

        for idx, box in enumerate(boxes):
            if idx==0: # First box that is detected is always the full size of the frame. So ignore idx == 0
                continue
            x, SLD.y, w, SLD.h = cv2.boundingRect(box)
            if w > 100 and SLD.h < 50:
                cv2.rectangle(frame, (x, SLD.y), (x + w, SLD.y + SLD.h), (0, 0, 255), 2)
                cv2.putText(frame, 'Stop Line', (x, SLD.y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                if SLD.y + SLD.h > 630:
                    SLD.flag_StopLine = True 
        return frame
    
    def SLD_planning():
        if SLD.flag_StopLine == True and SLD.y > SLD.threshold_line_stop_y:
            return True # this will make the car stop
        else:
            return False # this won't make the car stop