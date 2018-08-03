#!/usr/bin/env python

import cv2
import numpy as np
from sensor:msgs.msg import Image
from ackermann_msgs.msg impor AckermannDriveStamped
from cv_bridge import CvBridge
class Sign:
  def __init__( self ):
    rospy.Subscriber("/camera/zed/rgb/image_rect_color",Image, self.callback, queue_size = 1)
    self.bridge=CvBridge()
    rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
   
    # publish to Ackermann
    self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)
    
    self.sign_left=False
    self.sign_right=False
    
    def call_back(self,msg):
        self.msg=msg
        frame = self.bridge.imgmsg_to_cv2(msg)
        detector=cv2.SIFT()
        FLANN_INDEX_KDITREE=0
        flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
        flann=cv2.FlannBasedMatcher(flannParam,{})

        trainImg=cv2.imread("sign_right.jpeg",0)
        trainKP,trainDesc=detector.detectAndCompute(trainImg,None)

        while True:
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            queryKP,queryDesc=detector.detectAndCompute(gray,None)
            matches=flann.knnMatch(queryDesc,trainDesc,k=2)

            goodMatch=[]
            for m,n in matches:
                if(m.distance<0.75*n.distance):
                    goodMatch.append(m)
            if(len(goodMatch)>MIN_MATCH_COUNT):
                tp=[]
                qp=[]
                for m in goodMatch:
                    tp.append(trainKP[m.trainIdx].pt)
                    qp.append(queryKP[m.queryIdx].pt)
                tp,qp=np.float32((tp,qp))
                H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
                h,w=trainImg.shape
                trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
                queryBorder=cv2.perspectiveTransform(trainBorder,H)
                cv2.polylines(QueryImgBGR,[np.int32(queryBorder)],True,(0,255,0),5)
                self.sign_left=True
                self.sign_right=True
                print "Turn now"
            else:
                print "Not Enough match found- %d/%d"%(len(goodMatch),MIN_MATCH_COUNT)
            cv2.imshow('result',frame)
            
            if cv2.waitKey(10)==ord('q'):
                break
            
if __name__ == "__main__":
    rospy.init_node(Sign", anonymous = True)
    node =Sign()
    rospy.spin()   
