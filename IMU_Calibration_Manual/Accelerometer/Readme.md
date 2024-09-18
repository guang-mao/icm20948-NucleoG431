# 加速度計校準說明:

### 參考[連結](https://blog.csdn.net/u014430081/article/details/127175830)

## 六面位置校準 
### 1、 誤差模型: 
**f<sup>b</sup><sub>理想值</sub> = K<sub>a</sub> ( f<sup>b</sup><sub>量測值</sub> - $\nabla$<sup>b</sup> )** 

$$
K_{a}=\begin{bmatrix}
k_{xx} & k_{xy} & k_{xz} \\
k_{yx} & k_{yy} & k_{yz} \\
k_{zx} & k_{zy} & k_{zz}
\end{bmatrix}, \nabla^b = 加速度計偏移 \tag{1}
$$

**f<sup>b</sup><sub>量測值</sub> = F<sub>a</sub> f<sup>b</sup><sub>理想值</sub> + $\nabla$<sup>b</sup>**

$$
F_{a} = K_{a}^{-1} = \begin{bmatrix}
F_{xx} & F_{xy} & F_{xz} \\
F_{yx} & F_{yy} & F_{yz} \\
F_{zx} & F_{zy} & F_{zz}
\end{bmatrix} \tag{2}
$$

### 2、 6位置法: 

**當X軸朝上靜止時，f<sup>b</sup><sub>理想值</sub> = [g, 0, 0]<sup>T</sup>;當X軸朝下靜止時，f<sup>b</sup><sub>理想值</sub> = [-g, 0, 0]<sup>T</sup>**

$$
\begin{cases}
f^{b}_{實際值x}(X+)=F_{xx}g + \nabla_{x} \\
f^{b}_{實際值y}(X+)=F_{yx}g + \nabla_{y} \\
f^{b}_{實際值z}(X+)=F_{zx}g + \nabla_{z} \\
\end{cases};

\begin{cases}
f^{b}_{實際值x}(X-)=-F_{xx}g + \nabla_{x} \\
f^{b}_{實際值y}(X-)=-F_{yx}g + \nabla_{y} \\
f^{b}_{實際值z}(X-)=-F_{zx}g + \nabla_{z} \\
\end{cases} \tag{3}
$$

<br>

**當Y軸朝上靜止時，f<sup>b</sup><sub>理想值</sub> = [0, g, 0]<sup>T</sup>;當Y軸朝下靜止時，f<sup>b</sup><sub>理想值</sub> = [0, -g, 0]<sup>T</sup>**

$$
\begin{cases}
f^{b}_{實際值x}(Y+)=F_{xy}g + \nabla_{x} \\
f^{b}_{實際值y}(Y+)=F_{yy}g + \nabla_{y} \\
f^{b}_{實際值z}(Y+)=F_{zy}g + \nabla_{z} \\
\end{cases};

\begin{cases}
f^{b}_{實際值x}(Y-)=-F_{xy}g + \nabla_{x} \\
f^{b}_{實際值y}(Y-)=-F_{yy}g + \nabla_{y} \\
f^{b}_{實際值z}(Y-)=-F_{zy}g + \nabla_{z} \\
\end{cases} \tag{4}
$$

<br>

**當Z軸朝上靜止時，f<sup>b</sup><sub>理想值</sub> = [0, 0, g]<sup>T</sup>;當Z軸朝下靜止時，f<sup>b</sup><sub>理想值</sub> = [0, 0, -g]<sup>T</sup>**

$$
\begin{cases}
f^{b}_{實際值x}(Z+)=F_{xz}g + \nabla_{x} \\
f^{b}_{實際值y}(Z+)=F_{yz}g + \nabla_{y} \\
f^{b}_{實際值z}(Z+)=F_{zz}g + \nabla_{z} \\
\end{cases};

\begin{cases}
f^{b}_{實際值x}(Z-)=-F_{xz}g + \nabla_{x} \\
f^{b}_{實際值y}(Z-)=-F_{yz}g + \nabla_{y} \\
f^{b}_{實際值z}(Z-)=-F_{zz}g + \nabla_{z} \\
\end{cases} \tag{5}
$$

### 3、 參數求解: 

**求偏移:**
<br><br>
**$\nabla$<sub>x</sub> = { f<sup>b</sup><sub>量測值x</sub>(X+) + f<sup>b</sup><sub>量測值x</sub>(X-) + f<sup>b</sup><sub>量測值x</sub>(Y+) + f<sup>b</sup><sub>量測值x</sub>(Y-) + f<sup>b</sup><sub>量測值x</sub>(Z+) + f<sup>b</sup><sub>量測值x</sub>(Z-) } / 6**

**$\nabla$<sub>y</sub> = { f<sup>b</sup><sub>量測值y</sub>(X+) + f<sup>b</sup><sub>量測值y</sub>(X-) + f<sup>b</sup><sub>量測值y</sub>(Y+) + f<sup>b</sup><sub>量測值y</sub>(Y-) + f<sup>b</sup><sub>量測值y</sub>(Z+) + f<sup>b</sup><sub>量測值y</sub>(Z-) } / 6**

**$\nabla$<sub>z</sub> = { f<sup>b</sup><sub>量測值z</sub>(X+) + f<sup>b</sup><sub>量測值z</sub>(X-) + f<sup>b</sup><sub>量測值z</sub>(Y+) + f<sup>b</sup><sub>量測值z</sub>(Y-) + f<sup>b</sup><sub>量測值z</sub>(Z+) + f<sup>b</sup><sub>量測值z</sub>(Z-) } / 6**

<br>

**求F<sub>a</sub>:**
<font size = 4>

$$
\begin{cases}
F_{xx}= \frac {F^b_{量測值x}(X+)-F^b_{量測值x}(X-)} {2g} \\
F_{yx}= \frac {F^b_{量測值y}(X+)-F^b_{量測值y}(X-)} {2g} \\
F_{zx}= \frac {F^b_{量測值z}(X+)-F^b_{量測值z}(X-)} {2g} \\
\end{cases}
$$

$$
\begin{cases}
F_{xy}= \frac {F^b_{量測值x}(Y+)-F^b_{量測值x}(Y-)} {2g} \\
F_{yy}= \frac {F^b_{量測值y}(Y+)-F^b_{量測值y}(Y-)} {2g} \\
F_{zy}= \frac {F^b_{量測值z}(Y+)-F^b_{量測值z}(Y-)} {2g} \\
\end{cases}
$$

$$
\begin{cases}
F_{xz}= \frac {F^b_{量測值x}(Z+)-F^b_{量測值x}(Z-)} {2g} \\
F_{yz}= \frac {F^b_{量測值y}(Z+)-F^b_{量測值y}(Z-)} {2g} \\
F_{zz}= \frac {F^b_{量測值z}(Z+)-F^b_{量測值z}(Z-)} {2g} \\
\end{cases}
$$
</font>

## 舉例

**x軸向上:** <br>
A<sub>x</sub>(X+) = +0.99288<br>
A<sub>y</sub>(X+) = -0.00190<br>
A<sub>z</sub>(X+) = -0.02084<br>

**x軸向下:** <br>
A<sub>x</sub>(X-) = -1.00754<br>
A<sub>y</sub>(X-) = -0.04359<br>
A<sub>z</sub>(X-) = +0.01802<br>

**y軸向上:** <br>
A<sub>x</sub>(Y+) = -0.01016<br>
A<sub>y</sub>(Y+) = +0.98420<br>
A<sub>z</sub>(Y+) = +0.02022<br>

**y軸向下:** <br>
A<sub>x</sub>(Y-) = -0.00614<br>
A<sub>y</sub>(Y-) = -1.02083<br>
A<sub>z</sub>(Y-) = +0.02624<br>

**z軸向上:** <br>
A<sub>x</sub>(Z+) = +0.02267<br>
A<sub>y</sub>(Z+) = -0.00654<br>
A<sub>z</sub>(Z+) = +1.00031<br>

**z軸向下:** <br>
A<sub>x</sub>(Z-) = -0.02842<br>
A<sub>y</sub>(Z-) = -0.03453<br>
A<sub>z</sub>(Z-) = -1.00631<br>

<br>

$$
\nabla_x = \frac { A_{x}(X+) + A_{x}(X-) + A_{x}(Y+)  + A_{x}(Y-)  + A_{x}(Z+)  + A_{x}(Z-) } {6}
$$

$$
\nabla_y = \frac { A_{y}(X+) + A_{y}(X-) + A_{y}(Y+)  + A_{y}(Y-)  + A_{y}(Z+)  + A_{y}(Z-) } {6}
$$

$$
\nabla_z = \frac { A_{z}(X+) + A_{z}(X-) + A_{z}(Y+)  + A_{z}(Y-)  + A_{z}(Z+)  + A_{z}(Z-) } {6}
$$

<br>

$$
F_{xx} = \frac {A_{x}(X+) - A_{x}(X-) } {2},
F_{xy} = \frac {A_{x}(Y+) - A_{x}(Y-) } {2},
F_{xz} = \frac {A_{x}(Z+) - A_{x}(Z-) } {2}
$$

$$
F_{yx} = \frac {A_{y}(X+) - A_{y}(X-) } {2},
F_{yy} = \frac {A_{y}(Y+) - A_{y}(Y-) } {2},
F_{yz} = \frac {A_{y}(Z+) - A_{y}(Z-) } {2},
$$

$$
F_{zx} = \frac {A_{z}(X+) - A_{z}(X-) } {2},
F_{zy} = \frac {A_{z}(Y+) - A_{z}(Y-) } {2},
F_{zz} = \frac {A_{z}(Z+) - A_{z}(Z-) } {2},
$$

<br>

$$
K_a = 
\begin{bmatrix}
F_{xx} & F_{xy} & F_{xz} \\
F_{yx} & F_{yy} & F_{yz} \\
F_{zx} & F_{zy} & F_{zz}
\end{bmatrix}^{-1}
$$

<br>

$$
\begin{bmatrix}
A_{x(ideal)} \\
A_{y(ideal)} \\
A_{z(ideal)}
\end{bmatrix} = K_{a}
(
  \begin{bmatrix}
  A_{x(measure)} - \nabla_x\\
  A_{y(measure)} - \nabla_y\\
  A_{z(measure)} - \nabla_z
  \end{bmatrix}
)
$$
