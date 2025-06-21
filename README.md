# vismap 1.0
### <span style="color: yellow;">由于colmap代码已经在我心里是滚瓜烂熟了，整个SfM开源的框架(openmvg、opensfm、alicevision、odm etc)没有一个不熟透的，后面会慢慢放出来改进的版本，一直到vismap2.4 版本。目前已经换个方向去做更大更广泛的，任务多，时间紧。另外舒适区不能太久了，就这样吧 ，祝大家学习colmap课程愉快！</span>   
### roadmap  
<img width="880" alt="image" src="https://github.com/user-attachments/assets/c37c89b6-6826-4ffa-a7a4-568fd78da38f" />




# vismap system review
vismap 做的工作有如下：

1. 特征匹配分为两个部分

    image pairs 获取和描述子匹配，vismap 的image pairs 通过learning的方式，如netvlad 获取而不是词汇树

1. 匹配完毕后，会形成一个view graph ,那么对于view graph 有如下操作
    
    （1） 重新定义边的权，源代码是直接用内点个数定权/定得分 ，而没有考虑内点的分布，vismap 支持内点个数和分布的线性组合来确定边的权，graph edge 的权重对聚类分组很重要，这种对无人机数据有用
    
    （2）view graph 的优化或者说冗余剔除，利用rotation averaging 来剔除一些不稳定的边。
    
2. view graph 的聚类
    
    （1） Ncut
    
    （2）Expand 
    
    Ncut 后会形成 lost edge ,那么对于lost edge ，分别判断边的端点属于哪一个cluster ，如果两个cluster 的超过一定的重叠图像，则不再进行扩展，避免形成过大的聚类;同时检查两个cluster的完整性，如果超过一定的阈值，也不需要扩展。下来对于所有的 lost edge 进行从大到小排序，排序的依据是边权重，优先选择权重较高的边进行扩展，因为这些边可能表示更强的图像关联性（例如更高的相似度）。优先选择小的聚类进行扩展，保障各个聚类的大小均衡。
    
3. local sfm 
    
    对所有类别进行图像数量排序，优先运行数量最多的cluster,同时使用omp 而非线程池，omp比thread 快15%。
    
    local sfm 改进方面（大多数都是四年前的工作）
    
    （1）位置约束的sfm 
    
    （2）feature scale 定权和约束BA的sfm
    
    （3）两视图三角化用IDWM实现，替换原来的线性解法
    
    （4）多视图三角化增加IGG 算法，可以剔除一些outliers
    
    （5）深度值约束的sfm ，深度值可以是激光的
    
    （6）p3p算法的替换
    
    （7）重复纹理下，view selection 算法的改进，用AAM 替换了原来基于金字塔得分的view selection 
    
    （8）更多的相机模型的支持，如全景设备，相机模型、几何验证、pnp、三角化、BA算法的修改支持球形  
    
     （9） sfm 结果的自动化评估，重投影误差、register number、还有3D点个数不应该是评判的metric
    
4. merge local sfm 结果
    
     vismap 存在两个方案
    
    （1) 有精确的位置约束
    
    那么对于所有的local sfm 进行误差排序，误差是对齐位置的误差，选择误差最小的cluster 作为anchor,不需要做sim3 变换，所有的cluster 都往anchor 靠即可
    
    （2） 位置不是那么精确
    
    i . 构建reconstruction graph 
    
     准则是：节点是每个local sfm ，边是sim3 的对齐误差，利用公共3D点对齐的误差作为edge weight 
    
    ii. 最小生成树构建，得到最准确的n-1个边候选对齐
    
    iii. 通过合并叶节点，得到anchor ,其他的cluster都要与其对齐
    
    iv. 计算其他cluster 到anchor 的变换矩阵
    
    v. 合并 
