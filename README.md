# colmap-fusion-gps
colmap fusion gps ,reduce drift !
---
this is original colmap run forward-facing data,scale drift !!!!!!!!!!!!!!!!!!    
实现需要补充的代码部分：  
bundle_adjustment.cc 是在src/optim 目录下    
incremental_mapper.cc 是在src/sfm 目录下    
cost_functions.h 是在src/base 目录下   
![image](https://github.com/yuancaimaiyi/colmap-fusion-gps/blob/main/1.png)

I add gps constraint in global bundle adjustment,obviously，the drift disappear！
![image](https://github.com/yuancaimaiyi/colmap-fusion-gps/blob/main/3.png)

I have simulated tunnels or no GPS, and my integrated GPS still performs well，like this!
![image](https://github.com/yuancaimaiyi/colmap-fusion-gps/blob/main/4.png)

Last, thank you very much for the excellent work of the author(@ahojnnes) of colmap.Later,I will add feature scale constraint in bundle adjustment!!!!!
