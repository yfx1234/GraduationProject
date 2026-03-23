# 基于机器视觉的低慢小目标拦截技术研究

## 1. 课题定位

结合当前毕设项目代码，论文工作可以明确为一条完整技术链：

- 基于 UE/AirSim 仿真环境进行场景构建、图像采集与目标状态获取
- 基于自动标注与 YOLO 模型完成低慢小目标检测
- 基于目标图像特征和卡尔曼预测完成目标状态估计
- 基于视觉误差、面积约束和 PID/视觉伺服思想完成拦截控制
- 通过仿真实验验证检测、跟踪、预测与拦截效果

项目对应代码位置包括：

- `Source/GraduationProject/Guidance/VisualInterceptController.h`
- `Source/GraduationProject/Guidance/KalmanPredictor.h`
- `Source/GraduationProject/Guidance/GuidanceActor.h`
- `PythonClient/Run/run.py`
- `PythonClient/Run/Collect.py`
- `PythonClient/Run/Train.py`

## 2. 任务书草稿

以下内容可直接填入你截图中的任务书表单。

### 2.1 设计（论文）的主要任务及目标

本课题围绕“基于机器视觉的低慢小目标拦截技术研究”展开，面向无人机等低空、慢速、小尺寸目标在复杂背景下难检测、难跟踪、难稳定拦截的问题，研究一套集“仿真建模、视觉感知、目标预测、控制决策与效果验证”于一体的技术方案。课题以 Unreal Engine/AirSim 仿真平台和 Python 感知控制链路为基础，构建低慢小目标拦截实验环境，完成目标图像采集、数据集构建、目标检测模型训练、目标运动状态估计和视觉引导拦截控制，并通过仿真实验对系统性能进行验证。

本课题的主要目标如下：

1. 搭建面向低慢小目标拦截的仿真测试平台，实现目标与拦截器的场景生成、运动控制和图像采集。
2. 完成低慢小目标图像数据采集与自动标注，构建可用于检测训练的数据集。
3. 研究适用于低慢小目标的视觉检测方法，训练并优化 YOLO 检测模型，提高复杂背景下的小目标识别能力。
4. 研究基于视觉特征的目标状态估计方法，引入卡尔曼预测提升目标短时运动预测与丢失恢复能力。
5. 设计基于图像误差和面积反馈的视觉引导拦截控制方法，实现目标搜索、跟踪、逼近与拦截过程控制。
6. 通过多组仿真实验对系统在检测精度、跟踪稳定性、响应速度和拦截成功率等方面进行测试与分析。
7. 完成毕业论文撰写，形成较为完整的低慢小目标视觉拦截技术研究成果。

### 2.2 设计（论文）的主要内容

本课题主要研究内容包括以下几个方面：

1. 低慢小目标拦截仿真环境构建。基于 Unreal Engine 与 AirSim 建立无人机拦截仿真平台，完成拦截器、目标机与引导控制对象的场景配置，为后续感知与控制算法验证提供实验基础。
2. 数据采集与数据集构建。设计目标机与拦截器的相对运动方式、视角变化和距离变化策略，利用仿真平台采集低慢小目标图像，并结合自动投影与自动标注方法生成训练标签，构建检测数据集。
3. 低慢小目标视觉检测研究。针对低慢小目标尺寸小、纹理弱、背景复杂等特点，研究基于 YOLO 的目标检测方法，完成模型训练、参数调整和效果评估，并分析不同场景下检测性能变化规律。
4. 目标跟踪与运动状态预测研究。根据检测框中心、面积等视觉观测量，设计目标状态估计与短时预测方法，引入卡尔曼滤波提高对目标位置、速度和运动趋势的估计能力，缓解瞬时漏检带来的控制不稳定问题。
5. 视觉引导拦截控制研究。结合图像平面误差、目标面积变化和预测结果，设计搜索、跟踪、逼近和捕获等控制状态切换机制，并利用 PID 控制实现无人机的航向、前向速度和高度控制。
6. 系统集成与实验分析。将仿真环境、检测模型、状态预测模块和拦截控制模块集成为完整系统，设置不同目标运动模式、不同检测条件和不同控制参数开展实验，对拦截过程和结果进行分析总结。
7. 毕业论文撰写。系统梳理国内外相关研究现状，完成任务书、文献翻译、文献综述、论文正文及相关材料整理。

### 2.3 设计（论文）的基本要求

1. 系统应能够在仿真环境下完成低慢小目标的搜索、检测、跟踪和拦截过程验证。
2. 应完成低慢小目标相关数据采集与数据集构建，数据集具有一定数量和场景多样性，能够支持检测模型训练与评估。
3. 检测模型应能够输出目标位置、置信度和目标框尺度等信息，并具备一定的实时性与稳定性。
4. 应完成基于视觉观测的目标状态预测，能够在短时漏检情况下保持较稳定的控制输出。
5. 应设计较完整的视觉引导拦截控制流程，包含目标搜索、目标跟踪、逼近拦截及状态切换机制。
6. 应建立实验评价指标，对检测效果、跟踪精度、控制响应和拦截成功率进行分析。
7. 论文撰写应符合本科毕业设计规范，结构完整、逻辑清晰、图表规范、参考文献引用准确。
8. 文献综述应重点覆盖低慢小目标检测、无人机视觉跟踪、状态预测、视觉伺服与拦截控制等方向，参考文献不少于 40 篇，以高质量 SCI、EI 和顶会论文为主。

### 2.4 主要参考文献（任务书内可先填写 10 篇）

1. T.-Y. Lin et al., "Feature Pyramid Networks for Object Detection," CVPR, 2017. [DOI](https://doi.org/10.1109/cvpr.2017.106)
2. T.-Y. Lin et al., "Focal Loss for Dense Object Detection," ICCV, 2017. [DOI](https://doi.org/10.1109/iccv.2017.324)
3. D. Du et al., "The Unmanned Aerial Vehicle Benchmark: Object Detection, Tracking and Baseline," IJCV, 2019. [DOI](https://doi.org/10.1007/s11263-019-01266-1)
4. D. Du et al., "Detection and Tracking Meet Drones Challenge," TPAMI, 2021. [DOI](https://doi.org/10.1109/tpami.2021.3119563)
5. Q. Liu et al., "Vision-Based Anti-UAV Detection and Tracking," IEEE TITS, 2022. [DOI](https://doi.org/10.1109/tits.2022.3177627)
6. H. Li et al., "Anti-UAV: A Large-Scale Benchmark for Vision-Based UAV Tracking," IEEE TMM, 2021. [DOI](https://doi.org/10.1109/tmm.2021.3128047)
7. M. Boodagh et al., "Toward Visibility Guaranteed Visual Servoing Control of Quadrotor UAVs," IEEE/ASME TMech, 2019. [DOI](https://doi.org/10.1109/tmech.2019.2906430)
8. S. Song and J. Xiao, "Tracking Objects as Points," ECCV, 2020. [DOI](https://doi.org/10.1007/978-3-030-58548-8_28)
9. Y. Sun et al., "ByteTrack: Multi-Object Tracking by Associating Every Detection Box," ECCV, 2022. [DOI](https://doi.org/10.1007/978-3-031-20047-2_1)
10. S. Shah et al., "AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles," Springer FSR, 2017. [DOI](https://doi.org/10.1007/978-3-319-67361-5_40)

### 2.5 进度安排

1. 第 1-2 周：明确课题需求，阅读相关文献，完成任务书与开题准备。
2. 第 3-5 周：完成仿真环境梳理、场景配置和数据采集流程设计。
3. 第 6-8 周：完成低慢小目标数据采集、数据清洗与数据集构建。
4. 第 9-11 周：完成目标检测模型训练、调参与检测实验分析。
5. 第 12-13 周：完成目标状态预测与视觉引导拦截控制算法设计与调试。
6. 第 14-15 周：完成系统联调与拦截实验，整理实验结果和图表。
7. 第 16 周：完成文献综述、论文初稿撰写。
8. 第 17 周：完成论文修改、定稿及答辩准备。

## 3. 文献检索方案

### 3.1 检索主题

围绕本课题，将综述文献分为四条主线：

1. 低慢小目标与小目标检测
2. 无人机/反无人机目标检测与跟踪基准
3. 目标跟踪、状态估计与多目标关联
4. 视觉伺服、视觉引导拦截与仿真验证

### 3.2 推荐检索库

- Google Scholar
- IEEE Xplore
- ACM Digital Library
- CVF Open Access
- SpringerLink
- ScienceDirect
- arXiv

### 3.3 推荐检索式

可组合使用以下英文检索式：

- `"low slow small target" OR "small aerial target" OR "small object detection" AND UAV`
- `"UAV object detection" AND tracking AND benchmark`
- `"anti-UAV" AND detection AND tracking`
- `"visual servoing" AND quadrotor AND target tracking`
- `"moving target interception" AND UAV`
- `"AirSim" AND aerial AND vision`

### 3.4 纳入标准

1. 与低慢小目标检测、无人机视觉跟踪、状态预测、视觉伺服、拦截控制或仿真验证直接相关。
2. 以 SCI、EI、CCF 推荐会议、IEEE/CVF/AAAI/ECCV/ICCV/CVPR 等高质量来源为主。
3. 时间上优先 2019-2025 年文献，并适当保留高被引经典基础文献。
4. 对项目实现具有直接参考价值，如检测模型、数据集、跟踪方法、视觉控制方法、仿真平台。

### 3.5 排除标准

1. 与地面目标、纯雷达对抗或纯通信感知无直接关联的文献。
2. 仅有工程介绍但缺乏方法细节或实验验证的文献。
3. 与本科毕设技术链路关联度过弱的泛化综述。

## 4. 40 篇优质文献清单

说明：以下清单以正式期刊/顶会/高价值 benchmark 论文为主，少量综述用于搭建综述背景。链接优先给 DOI 或官方会议页面。

### A. 低慢小目标与小目标检测

1. Feature Pyramid Networks for Object Detection. CVPR 2017. [链接](https://doi.org/10.1109/cvpr.2017.106)
2. Focal Loss for Dense Object Detection. ICCV 2017. [链接](https://doi.org/10.1109/iccv.2017.324)
3. EfficientDet: Scalable and Efficient Object Detection. CVPR 2020. [链接](https://doi.org/10.1109/cvpr42600.2020.01079)
4. End-to-End Object Detection with Transformers. ECCV 2020. [链接](https://doi.org/10.1007/978-3-030-58452-8_13)
5. QueryDet: Cascaded Sparse Query for Accelerating High-Resolution Small Object Detection. CVPR 2022. [链接](https://doi.org/10.1109/cvpr52688.2022.01330)
6. Small Object Detection in Unmanned Aerial Vehicle Images Using Feature Fusion and Scaling-Based Single Shot Detector With Spatial Context Analysis. IEEE TCSVT 2019. [链接](https://doi.org/10.1109/tcsvt.2019.2905881)
7. Focus-and-Detect: A Small Object Detection Framework for Aerial Images. Signal Processing: Image Communication 2022. [链接](https://doi.org/10.1016/j.image.2022.116675)
8. MFFSODNet: Multiscale Feature Fusion Small Object Detection Network for UAV Aerial Images. IEEE TIM 2024. [链接](https://doi.org/10.1109/tim.2024.3381272)
9. A Context-Scale-Aware Detector and a New Benchmark for Remote Sensing Small Weak Object Detection in Unmanned Aerial Vehicle Images. JAG 2022. [链接](https://doi.org/10.1016/j.jag.2022.102966)
10. Small Object Detection in Unmanned Aerial Vehicle Images Using Multi-Scale Hybrid Attention. Engineering Applications of AI 2023. [链接](https://doi.org/10.1016/j.engappai.2023.107455)
11. Tiny Object Detection in Aerial Images. ICPR 2021. [链接](https://doi.org/10.1109/icpr48806.2021.9413340)

### B. 无人机/反无人机检测与跟踪基准

12. Detection and Tracking Meet Drones Challenge. IEEE TPAMI 2021. [链接](https://doi.org/10.1109/tpami.2021.3119563)
13. The Unmanned Aerial Vehicle Benchmark: Object Detection, Tracking and Baseline. IJCV 2019. [链接](https://doi.org/10.1007/s11263-019-01266-1)
14. Vision-Based Anti-UAV Detection and Tracking. IEEE TITS 2022. [链接](https://doi.org/10.1109/tits.2022.3177627)
15. Anti-UAV: A Large-Scale Benchmark for Vision-Based UAV Tracking. IEEE TMM 2021. [链接](https://doi.org/10.1109/tmm.2021.3128047)
16. A Unified Transformer-Based Tracker for Anti-UAV Tracking. CVPRW 2023. [链接](https://doi.org/10.1109/cvprw59228.2023.00305)
17. SeaDronesSee: A Maritime Benchmark for Detecting Humans in Open Water. WACV 2022. [链接](https://doi.org/10.1109/wacv51458.2022.00374)
18. BIRDSAI: A Dataset for Detection and Tracking in Aerial Thermal Infrared Videos. WACV 2020. [链接](https://doi.org/10.1109/wacv45572.2020.9093284)
19. WebUAV-3M: A Benchmark for Unveiling the Power of Million-Scale Deep UAV Tracking. IEEE TPAMI 2022. [链接](https://doi.org/10.1109/tpami.2022.3232854)
20. Unmanned Aerial Vehicle Visual Detection and Tracking Using Deep Neural Networks: A Performance Benchmark. ICCVW 2021. [链接](https://doi.org/10.1109/iccvw54120.2021.00142)
21. VisDrone-DET2019: The Vision Meets Drone Object Detection in Image Challenge Results. ICCVW 2019. [链接](https://doi.org/10.1109/iccvw.2019.00030)
22. VisDrone-DET2021: The Vision Meets Drone Object Detection Challenge Results. ICCVW 2021. [链接](https://doi.org/10.1109/iccvw54120.2021.00319)
23. On the Detection of Unauthorized Drones—Techniques and Future Perspectives: A Review. IEEE Sensors Journal 2022. [链接](https://doi.org/10.1109/jsen.2022.3171293)

### C. 跟踪、状态估计与运动关联

24. Simple Online and Realtime Tracking with a Deep Association Metric. ICIP 2017. [链接](https://doi.org/10.1109/icip.2017.8296962)
25. ByteTrack: Multi-Object Tracking by Associating Every Detection Box. ECCV 2022. [链接](https://doi.org/10.1007/978-3-031-20047-2_1)
26. Tracking Objects as Points. ECCV 2020. [链接](https://doi.org/10.1007/978-3-030-58548-8_28)
27. SiamRPN++: Evolution of Siamese Visual Tracking With Very Deep Networks. CVPR 2019. [链接](https://doi.org/10.1109/cvpr.2019.00441)
28. Learning Discriminative Model Prediction for Tracking. ICCV 2019. [链接](https://doi.org/10.1109/iccv.2019.00628)
29. Improving Multiple Object Tracking with Single Object Tracking. CVPR 2021. [链接](https://doi.org/10.1109/cvpr46437.2021.00248)
30. Observation-Centric SORT: Rethinking SORT for Robust Multi-Object Tracking. CVPR 2023. [链接](https://doi.org/10.1109/cvpr52729.2023.00934)
31. HOTA: A Higher Order Metric for Evaluating Multi-Object Tracking. IJCV 2020. [链接](https://doi.org/10.1007/s11263-020-01375-2)

### D. 视觉伺服、引导拦截与仿真验证

32. Toward Visibility Guaranteed Visual Servoing Control of Quadrotor UAVs. IEEE/ASME TMech 2019. [链接](https://doi.org/10.1109/tmech.2019.2906430)
33. Image-Based Visual Servoing of Quadrotors to Arbitrary Flight Targets. IEEE RA-L 2023. [链接](https://doi.org/10.1109/lra.2023.3245416)
34. Image-Based Visual Servoing of Rotorcrafts to Planar Visual Targets of Arbitrary Orientation. IEEE RA-L 2021. [链接](https://doi.org/10.1109/lra.2021.3101878)
35. Position-Based Visual Servoing for Target Tracking by a Quadrotor UAV. AIAA GNC 2016. [链接](https://doi.org/10.2514/6.2016-2092)
36. Autonomous Drone Hunter Operating by Deep Learning and All-Onboard Computations in GPS-Denied Environments. PLOS ONE 2019. [链接](https://doi.org/10.1371/journal.pone.0225092)
37. Autonomous Target Tracking of UAV Using High-Speed Visual Feedback. Applied Sciences 2019. [链接](https://doi.org/10.3390/app9214552)
38. Robust Visual Servoing Formation Tracking Control for Quadrotor UAV Team. Aerospace Science and Technology 2020. [链接](https://doi.org/10.1016/j.ast.2020.106061)
39. UAV Guidance: A Stereo-Based Technique for Interception of Stationary or Moving Targets. Lecture Notes in Computer Science 2015. [链接](https://doi.org/10.1007/978-3-319-22416-9_30)
40. AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles. Springer FSR 2017. [链接](https://doi.org/10.1007/978-3-319-67361-5_40)

## 5. 文献综述写作提纲

### 5.1 综述题目建议

可使用如下题目之一：

- 低慢小目标视觉检测、跟踪与拦截技术研究综述
- 面向无人机低空小目标的机器视觉检测与拦截控制研究综述
- 基于机器视觉的低慢小目标检测、预测与拦截技术综述

### 5.2 综述结构建议

1. 引言  
   说明低慢小目标的应用背景、研究意义、技术难点和本文综述范围。

2. 低慢小目标视觉检测方法研究现状  
   从小目标成像特点、特征金字塔、类别不平衡、多尺度特征增强、轻量化检测网络等角度展开。

3. 无人机与反无人机检测跟踪数据集及基准  
   比较 UAVDT、VisDrone、Anti-UAV、WebUAV-3M、BIRDSAI 等数据集的场景、目标类型和评价方式。

4. 目标跟踪与状态估计研究现状  
   对比检测后跟踪、基于深度关联的多目标跟踪、点目标跟踪、视觉跟踪和卡尔曼预测等方法。

5. 视觉伺服与目标拦截控制研究现状  
   从图像误差闭环、目标可见性约束、目标逼近控制、视觉引导拦截与无人机自主追踪角度进行综述。

6. 仿真平台与实验验证方法  
   说明 AirSim 等仿真平台在视觉算法验证中的作用，以及仿真到实装之间的差异。

7. 现有研究不足与本文课题切入点  
   可归纳为：
   - 复杂背景下低慢小目标检测稳定性不足
   - 漏检情况下跟踪与控制耦合不够紧密
   - 检测、预测与拦截控制一体化研究相对不足
   - 面向低成本、可复现实验平台的研究还不够系统

8. 总结与展望  
   结合智能感知、轻量化模型、预测控制、多传感器融合等方向进行展望。

### 5.3 综述写作时每章可挂接的核心文献

- 第 2 章优先使用：1-11
- 第 3 章优先使用：12-23
- 第 4 章优先使用：24-31
- 第 5 章优先使用：32-39
- 第 6 章优先使用：40

## 6. 可直接使用的综述开头草稿

低慢小目标通常具有飞行高度低、运动速度慢、目标尺寸小、雷达反射截面积小等特点，典型目标包括小型无人机、微型飞行器及其他低空小型活动目标。该类目标在复杂低空环境中易与建筑物、树木、云层和地面纹理产生混淆，对传统感知与拦截系统提出了较高要求。随着无人机平台在巡检、测绘、物流和安防等领域的广泛应用，基于机器视觉实现低慢小目标的检测、跟踪、预测与拦截逐渐成为智能无人系统领域的重要研究方向。

从现有研究来看，低慢小目标拦截并不是单一检测问题，而是一个涉及场景建模、目标检测、状态估计、视觉伺服和闭环控制的系统性问题。一方面，小目标在图像中像素占比低、纹理信息弱，导致检测算法容易出现漏检和误检；另一方面，即使检测结果能够输出目标位置，若缺少稳定的状态预测与控制机制，也难以支撑无人机完成持续跟踪和有效逼近。因此，如何构建一套从视觉感知到拦截控制的一体化技术链路，是该方向的关键问题。

近年来，目标检测领域在特征金字塔、多尺度表示、Transformer 检测器和轻量化网络方面取得了显著进展，为低慢小目标检测提供了方法基础；无人机视觉领域则逐步形成了 VisDrone、UAVDT、Anti-UAV、WebUAV-3M 等典型数据集和基准，为算法对比提供了统一平台；在控制方面，视觉伺服、目标引导和自主追踪研究进一步推动了无人机对动态目标的闭环控制能力提升。然而，现有研究在复杂背景下的小目标稳健检测、短时漏检条件下的预测保持、检测模块与控制模块的紧耦合集成等方面仍存在不足，这也构成了本文课题的主要研究切入点。

## 7. 综述配图建议

建议在文献综述中至少加入 2 张图：

1. 低慢小目标视觉拦截技术链路图  
   内容建议：仿真环境 -> 图像采集 -> 数据标注 -> YOLO 检测 -> 卡尔曼预测 -> 视觉伺服控制 -> 拦截结果评估。

2. 文献主题演进图  
   内容建议：以“检测方法”“数据集/基准”“跟踪预测”“视觉伺服与拦截控制”四类为横向分区，展示 2017-2025 年代表性论文分布。

如果后续需要，我可以继续把这两张图的生成提示词和论文中的图题说明一起补齐。
