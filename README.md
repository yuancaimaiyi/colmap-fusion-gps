# colmap fusion gps   
## <span style="color: red;">Completion and Annotations : Improved COLMAP</span>  
- [x] <span style="color: red;">colmap fusion gps ,reduce drift (Submitted in 2021)</span>    
- [x] <span style="color: red;">feature scale determines the weight of BA</span>   
- [x] <span style="color: red;">multi-data, such as depth and distance constrained BA</span>   
- [x] <span style="color: red;">IDWM for two-view triangulation</span>     
- [x] <span style="color: red;">Panoramic data splitting code (branch:postprocess_panorama)</span>     
- [x] <span style="color: red;">Added panoramic camera model</span>   
- [x] <span style="color: red;">learned-feature sfm builds long-term map</span>
- [x] <span style="color: red;">Map localization based on SIFT</span>   
- [ ] <span style="color: red;">There will be more improvements later, and these are just the tip of the iceberg I showed (my motto is that explaining the source code is meaningless, only improvements are valuable work, long live photogrammetry!)</span>  
![image](https://github.com/user-attachments/assets/9048cd7b-ad1c-4988-b9bf-3d0c366e99ca)

3年多前colmap fusion gps 完整代码，二期课程代码会基于这个branch   
（1） feature scale 定权加入BA   
![image](https://github.com/user-attachments/assets/0ee4f4e2-1ad9-4e41-b4be-d8e8e238cbbc)
![image](https://github.com/user-attachments/assets/70597480-f717-476e-a1c3-745d5bf39556)


（2） 多模态数据，如深度、距离约束BA    
![image](https://github.com/user-attachments/assets/360de70c-cf86-4ed5-90f6-279dc786bc58)

测试数据：https://drive.google.com/file/d/1GSZ9oPpWhCJ1ycLcv47_ZeWBXhFYUO8Z/view  

