
import argparse
import xml.etree.ElementTree as ET
import math
import cv2
import numpy as np

import json
from scipy.spatial.transform import Rotation as RR
import os 

###############################################################################

def parse_args():
	parser = argparse.ArgumentParser(description="convert Agisoft XML export to nerf format transforms.json")

	parser.add_argument("--xml", default="", help="specify xml file location")
	parser.add_argument("--out", default="", help="output path")
	parser.add_argument("--ext", default="jpg", help="type of images (ex. jpg, png, ...)")
	args = parser.parse_args()
	return args
if __name__ == "__main__":
	args = parse_args()
	XML_LOCATION = args.xml
	IMGTYPE = args.ext
	os.makedirs(args.out,exist_ok=True)
	OUTPATH = args.out
	with open(XML_LOCATION, "r") as f:
		root = ET.parse(f).getroot()
		#print(root[0][0][0].tag)
		width = float(root[0][0][0][0].get("width"))
		height = float(root[0][0][0][0].get("height"))
		
		frames = list()
		image_id = 0 
		cameras = open(os.path.join(OUTPATH,"cameras.txt"),"w+")
		images = open(os.path.join(OUTPATH,"images.txt"),"w+")
		points3D = open(os.path.join(OUTPATH, "points3D.txt"), "w+")

		for frame in root[0][2]:
			current_frame = dict()
			if not len(frame):
				continue
			if(frame[0].tag != "transform"):
				continue
			
			imagename = frame.get("label")+"." + IMGTYPE

			matrix_elements = [float(i) for i in frame[0].text.split()]
			transform_matrix = np.array([[matrix_elements[0], matrix_elements[1], matrix_elements[2], matrix_elements[3]], [matrix_elements[4], matrix_elements[5], matrix_elements[6], matrix_elements[7]], [matrix_elements[8], matrix_elements[9], matrix_elements[10], matrix_elements[11]], [matrix_elements[12], matrix_elements[13], matrix_elements[14], matrix_elements[15]]])
			
			# #swap axes
			# transform_matrix = transform_matrix[[2,0,1,3],:]
			# #reflect z and Y axes
			# transform_matrix_ = matrixMultiply(matrixMultiply(transform_matrix, reflectZ()), reflectY())
			Rwc = transform_matrix[:3,:3]
			# center = transform_matrix[:3,-1]
			# tvec =  -np.dot(Rwc,center)
			tvec = transform_matrix[:3,-1]
			r = RR.from_matrix(Rwc)
			qvec = r.as_quat()
			qvec = qvec.tolist()
			w = qvec.pop(-1)
			qvec.insert(0,w)
			qvec_str = " ".join(map(str,qvec))
			tvec_str = " ".join(map(str,tvec.tolist())) 

			# qvec = " ".join(map(str,information[image_name][0]["qvec"]))
        	# tvec = " ".join(map(str,information[image_name][0]["tvec"]))

			images.writelines(f"{image_id+1} {qvec_str} {tvec_str} {1} {imagename}\n")
			images.writelines("\n")
			image_id += 1
		cameras.writelines(f"1 PANORAMA {width} {height} 1 {width / 2} {height / 2}\n")
		cameras.close()
		images.close()
		points3D.close()



