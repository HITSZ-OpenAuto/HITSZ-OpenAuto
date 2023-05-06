> Machine Vision Revision (Summarized from lessons)
> Prof. Hu Liang
> Version 1.0
> 200320721 Tang Longbin
> 2023/5/4
> 欢迎补充！

# Lec 1 Introduction

## What can Machine Vision do?
- Increase profits
- Reduce defects
- Increase yield
- Track, Trace and Control

## How?
- Measurement (Gauging)
- Counting
- Location
- Decoding
- Inspection (Defect detection)

## Key parts of a Machine Vision System
- Lighting
- Lens
- Sensor (CCD/CMOS)
- Vision Processing (Algorithm & Software)
- Communication

# Lec 2 How to design a MV system

## Machine Vision System Design
### Specialization of the task
- Task and Benefit
- Part (Shape and features)
  - Part Presentation
  - Part motion (Indexed positioning / Continuous movement)
- Performance Requirements
  - Accuracy
  - Time performance
- Information Interface
- Installation Space
- Environment
- Checklist
### Design of the system
- Camera Type
- **Field of View (FOV)**
- Resolution
  - camera sensor resolution (R*C)
  - Spatial Resolution (Depends on the camera sensor and FOV)
  - Measurement Accuracy (Depends on spatial resolution, feature contrast and software algorithms)
  - Calculation of Resolution
- Choice of Camera, Frame Grabber and Hardware Platform
- Lens Design
  - Focal length
  - Lens Flange Focal Distance (法兰距)
  - Lens Diameter and Sensor Size
- Choice of Illumination
- Mechanical Design
- Electrical Design
- Software
  - **ROI (Region Of Interest)**
### Calculation of costs
### Development and installation of the system

## Field of View (FOV)

FOV = maximum part size + tolerance in positioning + margin + adaption to the aspect of the camera sensor

## Calculation of Resolution

$$
R_s = \frac{FOV}{R_c} = \frac{S_f}{N_f}
$$

$$
R_c = \frac{FOV}{R_s} = FOV * \frac{N_f}{S_f}
$$

|Name|Variable|Unit|
|:-:|:-:|:-:|
|Camera resolution|$R_c$|pixel|
|Spatial resolution|$R_s$|mm/pixel|
|Field of View|$FOV$|mm|
|Size of the smallest feature|$S_f$|mm|
|Number of pixels to map the smallest feature|$N_f$|pixel|

## Pixel Rate

$$
PR = Rc_{hor} * Rc_{ver} * fr + overhead (10\% - 20\%)
$$

|Name|Variable|Unit|
|:-:|:-:|:-:|
|Pixel Rate|$PR$|pixel/s|
|Camera resolution horizontal|$Rc_{hor}$|pixel|
|Camera resolution vertical|$Rc_{ver}$|pixel|
|Frame Rate (帧率)|$fr$|Hz|
|Camera resolution|$R_c$|pixel|
|Line Frequency|$fs$|Hz|

## Focal length

$$
\frac{1}{f'} = \frac{1}{a'} - \frac{1}{a} 
$$

$f'$ : focal length

$a$ : standoff distance (work distance)

$a'$ : distance between the lens and the image sensor

## The magification $\beta$

$$
\beta = \frac{y'}{y} = \frac{a'}{a}
$$

$y$ : size of real-world object

$y'$ : size of the image object

$$
\beta = -\frac{sensor\space size}{FOV}
$$

so

$$
f' = a * \frac{\beta}{1-\beta}
$$

$$
a = f' * \frac{1 -\beta}{\beta}
$$

## Example

shown in the pdf (end of the lec2)

# Lec 3 Camera Sensor

## Light and CCD/CMOS sensor
- Quantum Efficiency (QE, 量子效率) : the ratio of light that the sensor converts into charge.
- **The Full Well Capacity (满井容量)** : the maximum number of electron that register a signal in a pixel.
  - 4,000 electrons --- small pixels
  - 10,000 electrons --- medium pixels
  - 50,000 electrons --- large pixels
- CCD and CMOS (The difference is how they transfer the charge out of the pixel and into the camera's electronic "read out".)
  - CCD : high image quality, low speed.
  - CMOS : low image quality, high speed.

## Digital Cameras: Basics
- Camera Controls (What can affect the quality of images?)
  - Gain
  - Exposure
  - Trigger
  - Image Format
  - Resolution
- Shutter
  - Global shutter (全局快门)
  - Rolling shutter (卷帘快门)
- Trigger
  - Hardware trigger (external)
    - Strobe (频闪)
  - Software trigger (internal)

## Camera Type
### Area Camera
- Fixed aspect ratio
- Easy image processing
- Longer intergration time possible
- Skipping and Binning (将相邻像元感应的电荷加在一起，以单个像素的模式读出)
### Color Area Camera
- 3-CCD (using a separate CCD for each color)
  - great image quality
  - expensive
- 1-CCD (Bayer Pattern)
  - Green 50%, Blue 25%, Red 25%
  - Lower color spatial resolution
  - Lower manufacturing cost
### Line Scan Camera
- 2nd dimension comes by movement
- Very good price/pixel performance
- High pixel fill-factor (typically 100%)
- Very short intergration time
- Difficult design in image processing
- $\frac{L_o}{H_c} = \frac{V_o}{V_c}$ (**图像不拉伸或者压缩的条件**)

## Camera Interfaces
- **GigE Vision Standard**
- IEEE 1394 (Vision Standard : DCAM)
- Camera Link
- USB
- Interface Comparison and the GenICam Standard

### GigE Vision
- Main advantages
  - Cable length and cost effective components
  - Accessories are reliable

### IEEE 1394
- Invented by Apple and TI in the late 1980s
  - Apple's original name : **"FireWire"**
- Standardized by IEEE in 1995

### Camera Link (CL)
- Require a "camera file"
- High Speed

### USB
- Maximum cable length is 5m

### vInterface Comparison and the GenICam Standard (相机通用接口)

## Smart Camera

# Lec 4 Lens

- Pinhole camera model (小孔成像)

## Gaussian Optics

- Light Refraction ($n_1 \sin \alpha_1 = n_2 \sin \alpha_2$)
- Paraxial Approximation (近轴近似) when $\alpha$ is small ($\sin i \approx i$)

## Glossary of Terms for Lens
- Field of View (FOV)
- Depth of Field (DOF)
- Work Distance (WD)
- Resolution
- Sensor Size
- Primary Magnification (Pmag)

## Resolution

## Contrast
$$
Contrast = \frac{I_{max} - I_{min}}{I_{max} + I_{min}}
$$

- Resolution is defined at a specific contrast

### Modulation Transfer Function (MTF)
$$
F_{MTF} = \frac{g_1 - g_2}{255}
$$

## Depth of Field (DoF)
- F# and Aperture (光圈)
$$
F\# = \frac{f}{D}
$$
- 大光圈，小景深；小光圈，大景深

## Distortion (畸变)
$$ 
Distortion = \frac{AD - PD}{PD} * 100\%
$$

AD : Actual distance
PD : Predicted distance

- Distortion < 0 ： 负畸变（桶形畸变）
- Distortion > 0 ： 正畸变（枕形畸变）

# Lec 5 Lighting

## Vision Lighting Sources
- **LED-Light Emitting Diode**
- Quartz Halogen-W/Fiber Optics
- Fluorescent
- Xenon
- Metal Halide
- High Pressure Sodium

## Solid Angle
$$
d\Omega = \frac{dA \cos\theta}{r^2}
$$

## Measuring LED light power
- 光通量 Flux（$\Phi$）
  - 单位： 流明 (lm)
- 光强 ($I$)
  - $I = \frac{d\Phi}{d\Omega}$
  - 单位：坎德拉 (cd)
- 照度（$E$）
  - $E = \frac{d\Phi}{dS}$
  - 单位： 勒克斯 (lx)
- 亮度（$L$）
  - $L = \frac{d\Phi}{dSd\Omega\cos\theta}$
  - 单位： 尼特 (nit)

## How to change contrast
- Light Pattern (Structure)
- Direction (Geometry)
- Spectrum (Color/WaveLength)
- Light Character (Filtering)

## Basic Lighting Techniques
- Bright Field (亮场照明) : greater than 45°
- Dark Field Lighting (暗场照明）: less than 45°
  - 表面凹凸表现力强
- Vertical Lighting
- Back Lighting
- Multi-angle Lighting
- Diffuse Dome
- On-axis Diffuse
- Flat Diffuse
- Point Source1 
- Strobe

## Pass Filters and Polarization Filters
Shown in the end of the pdf (lec 5)

# Lec 6 Introduction of MV Software
略

# Lec 7 Algorithm

## Binary Image
- Component Labeling
  - Row-by-Row (most common)
  - 行程编码 DFS
- Size Filter
- Euler Number (Genus)
  - $E = C - H$
  - C : the number of components
  - H : the number of holes
- Boundary
- Distance
  - Euclidean
  - city block
  - chessboard

## Image Enhancement
- Gray Value Transformations (GVT)
  - can use LUT (Lookup Table) to increase the speed
- Radiometric Calibration
- Image Smoothing
  - Temporal averaging (时域平均)
  - Mean Filter (均值滤波)
  - Gaussian Filter
  - Median and Rank Filter
- Fourier Transform
  - DFT
  - FFT

# Lec 8 Algorithm Fundamentals

## Geometric Transformations

### Affine Transformations (仿射变换)
- rotation and translation (平移)
- apparent change in size

$$
\begin{bmatrix}
    \hat{r} \\
    \hat{c} \\
    1
\end{bmatrix} =
\begin{bmatrix}
    a_{11} & a_{12} & a_{13} \\
    a_{21} & a_{22} & a_{23} \\
    0 & 0 & 1
\end{bmatrix} *
\begin{bmatrix}
    r \\
    c \\
    1
\end{bmatrix}
$$

Translation
$$
\begin{bmatrix}
    1 & 0 & t_r \\
    0 & 1 & t_c
\end{bmatrix}
$$

Scaling
$$
\begin{bmatrix}
    s_r & 0 & 0 \\
    0 & s_c & 0
\end{bmatrix}
$$

Rotation by $\alpha$
$$
\begin{bmatrix}
    \cos\alpha & -\sin\alpha & 0 \\
    \sin\alpha & \cos\alpha & 0
\end{bmatrix}
$$

Skew (倾斜) of the vertical axis by $\theta$
$$
\begin{bmatrix}
    \cos\theta & 0 & 0 \\
    \sin\theta & 1 & 0
\end{bmatrix}
$$

### Projective Transformations (投影变换)
- H : homography (单应), DoF = 8

$$
\begin{bmatrix}
    \hat{r} \\
    \hat{c} \\
    1
\end{bmatrix} =
\begin{bmatrix}
    h_{11} & h_{12} & h_{13} \\
    h_{21} & h_{22} & h_{23} \\
    h_{31} & h_{32} & h_{33}
\end{bmatrix} *
\begin{bmatrix}
    r \\
    c \\
    w
\end{bmatrix}
$$

### Nearest-Neighbor Interpolation
- the closest of the four adjacent pixel centers

### Bilinear Interpolation
- use four corresponding gray values and weights them appropriately

### Bicubic Interpolation

### Smoothing to avoid  aliasing

### Polar Transformations

## Image Segmentation

### Subpixel-Precise Thresholding
- 插值得到二维曲面，然后二值化

## Feature Extraction
### Region Features
- Area
- Moments (矩)
  - $m_{p,q} = \Sigma g_{r,c}r^pc^q$
  - 通过二阶中心矩的长轴短轴比可以判断圆与椭圆
- Ellipse Parameters
- Enclosing Rectangles and Circles
- Contour Length
  - Compactness of a region : $c = \frac{l^2}{4 \pi a} $
- Rectangularity
  - $R = \frac{A_0}{A_{MER}} $
- Roundness
  - $C = \frac{P^2}{4 \pi A} $
  - P is the perimeter
  - A is the area
- Circularity
  - $C = \min(1, C')$
  - $C' = \frac{A}{\pi d^2_{max}}$
- Statistical Features
  - minimum and maximum gray value in a region
  - mean gray value within the region
  - the varianve of the gray values
  - standard deviation

## Morphology
略

## Blob analysis
- Blob 是指对一个提取的 Region 进行特征分析的过程

# Lec 9 NCC Template Matching
- To descrip the object to be found by a template
- By computing the similarity between the template and the image

## Gray-Value-Based Template Matching

### Similarity Measures Based on Gray Value Differences
- $SAD(r,c) = \frac{1}{n} \Sigma_{(u,v)} |t(u,v) - f(r+u,c+v)|$
- $SSD(r,c) = \frac{1}{n} \Sigma_{(u,v)} (t(u,v) - f(r+u,c+v))^2$
We can find instances with a certain upper threshold.
Disadvantage : Affected by the lighting.

### Normalized Cross-Correlation (NCC)

$$
NCC(r,c) = \frac{1}{n} \Sigma_{(u,v)} \frac{t(u,v) - m_t}{\sqrt{s_t^2}} * \frac{f(r+u,c+v) - m_f(r,c)}{\sqrt{s_f^2(r,c)}}
$$
$$
m_t = \frac{1}{n} \Sigma_{(u,v)} t(u,v)
$$
$$
s_t^2 = \frac{1}{n} \Sigma_{(u,v)} (t(u,v) - m_t)^2
$$
$$
m_f = \frac{1}{n} \Sigma_{(u,v)} f(r+u,c+v)
$$
$$
s_f^2 = \frac{1}{n} \Sigma_{(u,v)} (f(r+u,c+v) - m_f(r,c)^2
$$

## Matching Using Image Pyramids
- Image Pyramids --- Sub-sampling

## Robust Template Matching
- Mean squared edge distance
- Hausdorff distance

# Lec 10 Robust Template Matching

## Generalized Hough Transform (广义霍夫变换)
- Compute centroid $(x_c, y_c)$
- For each egde point $(x, y)$, compute its distance to centroid $r(x-x_c, y-y_c)^T$
- Find edge orientation (gradient angle $\phi$)
- Construct a table (R-table) of angle and r values
### Detection precedures
- For each edge point
  - create an accumulator array of 2D $(x,y)$
  - For each edge point $(x_i,y_i,\phi_i)$, to use its gradient orient to index stored table
  - For each entry $r_k^i$ in table, compute:
    - $x_c = x_i + r_k^i \cos \theta_k^i$
    - $y_c = y_i + r_k^i \sin \theta_k^i$
  - Incremental accumulator : $A(x_c, y_c) = A(x_c, y_c) + 1$
  - Find local maxima in $A(x_c, y_c)$

### Handle Scale and Rotation
- Use accumulator array : $A(x_c, y_c, S, \alpha)$
- S is the scale factor
- $\alpha$ is the rotation factor
- Use
  - $x_c = x_i + r_k^i S \cos (\theta_k^i + \alpha)$
  - $y_c = y_i + r_k^i S \sin (\theta_k^i + \alpha)$
- $A(x_c, y_c, S, \alpha) = A(x_c, y_c, S, \alpha) + 1$

## Shape-Based Matching
- Similar with Gray-Value-Based Template Matching but using gradient
- 略

## Deformable Matching
- 略

# Lec 11 Edge Detection and 1D 2D measurement

## Definition of Edges
- Definition of Edges of 1D
  - $f''(x) = 0$
  - $f'(x) > 0$ : positive edge
  - $f'(x) < 0$ : negative edge
- Definition of Edges of 2D
  - gradient $\max(|\nabla f(r,c)|)$
  - zero-crossings of the Laplacian $\Delta f(r,c) = 0$

## 1D Edge Extraction
- The first derivative is given by : $f_i' = \frac{1}{2}(f_{i+1} - f_{i-1})$
- The second derivative is given by : $f_i'' = \frac{1}{2}(f_{i+1} - 2f_i + f_{i-1})$
- Using two convolution masks $\frac{1}{2}(1 \space 0 \space -1)$ and $\frac{1}{2}(1 \space -2 \space 1)$

## 2D Edge Extraction
- Discrete derivatives are given in 2D
  - $f_{r;i,j}' = \frac{1}{2}(f_{i+1,j} - f_{i-1,j})$
  - $f_{c;i,j}' = \frac{1}{2}(f_{i,j+1} - f_{i,j-1})$
  - $f_r = \begin{bmatrix}
    -1 & -a & -1 \\
    0 & 0 & 0 \\
    1 & a & 1
    \end{bmatrix}
    f_c = \begin{bmatrix}
    -1 & 0 & 1 \\
    -a & 0 & a \\
    -1 & 0 & 1
    \end{bmatrix}$
  - a = 1 : Prewitt Filter
  - a = $\sqrt{2}$ : Frei Filter
  - a = 2 : Sobel Filter
- Non-Maximum Supression

## Accuracy and Precision of Edges
- Precision : how repeatable we can extract the value; the official name for precision is **repeatability**. Given by the variance $V[x] = \sigma_x^2$.
- Accuracy : how close on average the extracted value is to its true value. Discripted by the difference of the expected value $E(x)$ of the value to the true value $T$ : $[E(x) - T]$.

# 补充内容 Segmentation and Fitting of Geometric Primitives

## Fitting Lines
- Line equation : $y = mx + b$
- Hessian normal form of the line : $\alpha r + \beta c + \gamma = 0$ with $\alpha^2 + \beta^2 = 1$
- Least-Squares Line Fitting
  - $\min\epsilon^2 = \Sigma (\alpha r_i + \beta c_i + \gamma)^2 - \lambda (\alpha^2 + \beta^2 - 1) n $
- Robust Line Fitting
  - weight $w_i$ for each point depending on the distance $\delta_i = |\alpha r_i + \beta c_i + \gamma|$.
  - 首先令 $w_i = 1$, 计算出当前 $\alpha$ 和 $\beta$， 然后得出新 $w_i$ 进行迭代。

## Fitting Circles
- Least-Squares Circle Fitting
  - $\min\epsilon^2 = \Sigma (\sqrt{(r_i - \alpha)^2 + (c_i - \beta)^2} - \rho)^2$
- Robust Circle Fitting
  - same as line fitting

## Fitting Ellipses
- similar with circles fitting

## Segmentation of contours
- 略

## Camera Calibration
- 略