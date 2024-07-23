# colmap-fusion-gps   
## <span style="color: red;">Completion and Annotations : Improved COLMAP</span>  
- [x] <span style="color: red;">colmap fusion gps ,reduce drift (Submitted in 2021)</span>    
- [x] <span style="color: red;">feature scale determines the weight of BA</span>   
- [x] <span style="color: red;">multi-data, such as depth and distance constrained BA</span>   
- [x] <span style="color: red;">IDWM for two-view triangulation</span>     
- [x] <span style="color: red;">Panoramic data splitting code (branch:postprocess_panorama)</span>     
- [x] <span style="color: red;">Added panoramic camera model</span>   
- [x] <span style="color: red;">learned-feature sfm builds long-term map</span>
- [x] <span style="color: red;">Map localization based on SIFT</span>   
- [ ] <span style="color: red;">**There will be more improvements later, and these are just the tip of the iceberg I showed (my motto is that explaining the source code is meaningless, only improvements are valuable work, long live photogrammetry!**)</span>  
![image](https://github.com/user-attachments/assets/9048cd7b-ad1c-4988-b9bf-3d0c366e99ca)

3年多前colmap fusion gps 完整代码，二期课程代码会基于这个branch   
（1） feature scale 定权加入BA   
![image](https://github.com/user-attachments/assets/0ee4f4e2-1ad9-4e41-b4be-d8e8e238cbbc)
![image](https://github.com/user-attachments/assets/70597480-f717-476e-a1c3-745d5bf39556)


（2） 多模态数据，如深度、距离约束BA    
![image](https://github.com/user-attachments/assets/360de70c-cf86-4ed5-90f6-279dc786bc58)

测试数据：https://drive.google.com/file/d/1GSZ9oPpWhCJ1ycLcv47_ZeWBXhFYUO8Z/view  
## 见fusion-gps branch 分支    
## colmap fusion gps ,reduce drift !    
---
this is original colmap run forward-facing data,scale drift !!!!!!!!!!!!!!!!!!    
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
