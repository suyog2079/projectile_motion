$t \leftarrow \Delta t$
### Equations of motion used
$$\begin{align}
&x_{t+1} = x_{t}+v_{x} t\\
&y_{t+1} = y_{t}+v_{y} t-\frac{1}{2}g t^2 \\ 
&v_{x(t+1)} = v_{x(t)}  \\
&v_{y(t+1)} = v_{y(t)} -g t
\end{align} $$

### State Transition Model:
$$X_{k+1} = A_{k}X_{k}+B_{k}u_{k}+w_{k}$$
#### State vector

$$X = \begin{bmatrix}
x \\
y \\
v_x \\
v_y
\end{bmatrix}$$

#### Input vector:
$$u = \begin{bmatrix}
g
\end{bmatrix}$$

#### State transition matrix
$$A_{k} = \begin{bmatrix}
1 & 0 &  t & 0 \\
0 & 1 &  0 & t\\
0 & 0 &  1 & 0 \\
0 & 0 &  0 & 1 
\end{bmatrix}$$

#### Input transition matrix
$$B_{k} = \begin{bmatrix}
0  \\
-\frac{1}{2}t^2 \\
0 \\
-t
\end{bmatrix}$$

#### Final state transition model
$$
\begin{bmatrix}
x \\
y \\
v_x \\
v_y
\end{bmatrix}
= \begin{bmatrix}
1 & 0 &  t & 0 \\
0 & 1 &  0 & t\\
0 & 0 &  1 & 0 \\
0 & 0 &  0 & 1 
\end{bmatrix}
\begin{bmatrix}
x \\
y \\
v_x \\
v_y
\end{bmatrix} + 
\begin{bmatrix}
0  \\
-\frac{1}{2}t^2 \\
0 \\
-t
\end{bmatrix}
\begin{bmatrix}
g
\end{bmatrix}$$

### Measurement Model
$$Y_{k} = C_{k}X_{k}+v_{k}$$
#### Measurement vector
$$Y = \begin{bmatrix}
x \\
y \\
v_{x} \\
v_{y}
\end{bmatrix}$$
#### Measurement model matrix:
$$C = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}$$

