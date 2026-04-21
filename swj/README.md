# 云台追踪上位机 (swj)

一个基于 **PyQt5** 的显示型上位机，用来可视化 OpenMV + STM32 FOC 云台小球追踪系统。**只需要连 OpenMV 一个 USB 设备**，dx/dy/found 等数据都由 OpenMV 直接提供。

## 文件说明

| 文件 | 作用 |
|---|---|
| `openmv.py` | OpenMV 端固件。**保留** UART3 给电机发送原有 7 字节数据包；**新增** USB VCP 发送「JPEG 图像 + 识别信息」给 PC，并在图像上绘制目标框/两路中心十字 |
| `upper_computer.py` | PC 端 PyQt5 上位机，仅连 OpenMV USB |
| `requirements.txt` | Python 依赖 |
| `上位机.txt` | 原始需求说明 |

## 安装依赖

```powershell
pip install -r requirements.txt
```

## 使用步骤

1. 用 OpenMV IDE 把 `openmv.py` 烧/保存到 OpenMV，然后**关闭 IDE**（否则 IDE 占用 USB VCP）。
2. 运行上位机：

   ```powershell
   python upper_computer.py
   ```

3. 选择 **OpenMV 串口** → 点 **连接 OpenMV**。

> 电机依旧通过 OpenMV 的 UART3 接收数据包，只是 PC 上位机不再关心电机串口。

## 显示内容（右侧数据面板）

| 项 | 来源 |
|---|---|
| 图像中心点坐标 | 固定 `(80, 60)` |
| 物体中心点坐标 `(cx, cy)` | OpenMV USB |
| 是否检测到物体 | OpenMV USB |
| 物体移动速度 `px/s` | PC 端用相邻帧位移/时间差计算 |
| x/y 轴偏移量 `dx, dy` | OpenMV USB |
| 追上耗时 `ms` | PC 端「位置跳变方案」计算：<br>跳变条件——`|Δdx|` 或 `|Δdy|` > 阈值 (默认 15 px)；<br>起点——**跳变前一帧**的时间戳；<br>终点——首次 `|dx|,|dy| < 5` 的帧 |

## USB VCP 协议（OpenMV → PC）

```
+------+-----+-------------+----------+----------+------+
|Header|Len  |Info(17B)    |JpegLen(4)|JPEG bytes|Tail  |
|AA 55 |2B LE|<Bhhhhhhhh>  |uint32 LE |          |55 AA |
+------+-----+-------------+----------+----------+------+

Info = found(B), dx(h), dy(h), cx(h), cy(h), w(h), h(h), img_w(h), img_h(h)
```

## 参数调整

`upper_computer.py` 开头：

```python
JUMP_TH_DIFF = 15   # 跳变阈值（像素）
LOCK_TH      = 5    # 追上阈值（像素）
```

根据实际 dx/dy 抖动幅度微调。
