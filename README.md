# colmap-fusion-gps
colmap fusion gps ,reduce drift !
---
this is original colmap run forward-facing data,scale drift !!!!!!!!!!!!!!!!!!    
实现需要补充的代码部分（用仓库里面的代码replace下面目录即可，注意代码里面的变量要自己找个地方定义）：    
bundle_adjustment.cc 是在src/optim 目录下    
incremental_mapper.cc 是在src/sfm 目录下    
cost_functions.h 是在src/base 目录下   
![image](https://github.com/yuancaimaiyi/colmap-fusion-gps/blob/main/1.png)

I add gps constraint in global bundle adjustment,obviously，the drift disappear！
![image](https://github.com/yuancaimaiyi/colmap-fusion-gps/blob/main/3.png)

I have simulated tunnels or no GPS, and my integrated GPS still performs well，like this!
![image](https://github.com/yuancaimaiyi/colmap-fusion-gps/blob/main/4.png)
# 常用数据集  
1、ISPRS   
https://www.isprs.org/education/benchmarks.aspx   
2、sensefly   
https://www.sensefly.com/education/datasets/   
3、Kagaru  
https://michaelwarren.info/docs/datasets/kagaru-airborne-stereo/   
4、WHU  
http://gpcv.whu.edu.cn/data/WHU_MVS_Stereo_dataset.html   
5、KITTI  
http://www.cvlibs.net/datasets/kitti/  
6、ETH  
https://www.eth3d.net/  
7、realitycapture  
https://www.capturingreality.com/SampleDatasets  
8、视觉定位数据集  
https://www.visuallocalization.net/datasets/  
9、ODM  
https://www.opendronemap.org/odm/datasets/  
10、mve 官网的数据  
https://www.gcc.tu-darmstadt.de/home/proj/ambient_point_clouds/apc.en.jsp  
11、奥地利维也纳理工大学的Harvest4D项目(有GT）  
https://harvest4d.org/?page_id=1367  
12、自动化所偏古建筑  
http://vision.ia.ac.cn/zh/data/index.html  
13、blendedmvs  
https://github.com/YoYo000/BlendedMVS  
14、Drone Mapping  
https://dronemapper.com/sample_data/  
16、https://www.cs.cornell.edu/projects/bigsfm/#code bigsfm  
17、http://brian.harrison.org/Photogrammetry/JemezHistoricalSite/BodyJemez.html  
Last, thank you very much for the excellent work of the author(@ahojnnes) of colmap.Later,I will add feature scale constraint in bundle adjustment!!!!!
