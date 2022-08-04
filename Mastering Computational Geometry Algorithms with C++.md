# Mastering Computational Geometry Algorithms with C++ in udemyCourse

https://www.udemy.com/course/mastering-computational-geometry-cpp/



# Sec 1 : Introduction to Computational Geometry and Vector algebra.



## 게임엔진에서 충돌 판정은 어떻게 진행될까?



<img src="C:\Users\junhy\Desktop\CGA\collision.png" alt="collision" style="zoom:25%;" /> 

- 각 오브젝트의  큰 사각형 바운더리를 만들어서 겹치면 충돌판정을 함 (initial check) 
  - 이렇게 하면 정확한 충돌처리를 할 수 없음
- 정확한 판정은 convex hull 알고리즘을 통해 판정됨
  - 그러나 이 연산은 비싸기 때문에 사각형 바운더리 overlap되면 진행됨
  - We usually calculate convex hull for the models in preprocessing stage of the game and use that convex hull data in the first stage of the collision detection.

<img src="C:\Users\junhy\Desktop\CGA\collision1.png" alt="collision1" style="zoom:25%;" /> 

- 성능을 위해서 we partition the game space through multiple sub spaces and arrange that space in tree like data structure called BSP or Binary Space Partition, and then perform the collision checks
- 게임 안의 공간을 Binary spatial partitioning 알고리즘을 통해서 나눈뒤에 같은 공간안의 오브젝트에 대해 충돌판정을 함
- This will reduce the number of checks we need to render a frame greatly and increase the FPS value.



# Part 1. Goals

1.  Fluent in  vector algebra
2. Knows the basic 2D and 3D primitives and there math representation.
3. should be able to implement vector representation.
4. Able to perform queries like distance, angle, intersection between geometric primitives.
5. Should be able perform orientation check for a point compared to a line.



## Polygon

polygon can be think of as a planer figure bounded by set of line segments

<img src="C:\Users\junhy\Desktop\CGA\typeofpolygon.png" alt="typeofpolygon" style="zoom:25%;" /> 



## Normal vectors

- 주어진 벡터의 원점에서 수직인 벡터
- v(x, y) = u(  y, -x ) or u ( -y , x)

<img src="C:\Users\junhy\Desktop\CGA\normlaVector.png" alt="normlaVector" style="zoom: 33%;" /> 



## 내적

- 벡터 내적에서 가장 중요한 공식

$$
a\cdot b=|a||b|cos\theta
$$

- 두 벡터의 크기가 1인 경우

$$
\vec{a}\cdot \vec{b}=cos\theta
$$

- 두 벡터가 같다면

$$
a\cdot a=|a||a|cos\theta \\
a\cdot a=|a||a| * 1
$$

- 두 벡터가 수직인 경우

$$
a\cdot b=|a||b|cos 90\degree \\
a\cdot b=|a||b| * 0
$$

- 분배 법칙 성립

$$
\vec{u}\cdot (\vec{v}+\vec{w})=\vec{u}\cdot \vec{v} + \vec{u}\cdot \vec{w}
$$

- 사잇각

$$
u\cdot v=|u||v|cos\theta \\
\therefore cos\theta=\frac{u\cdot v}{|u||v|}
$$



## 외적

- 기본연산

$$
u=(u_x,u_y,u_z\ ),v=(v_x,v_y,v_z\ )\\
u\times v=(u_yv_z-v_yu_z,\ u_zv_x-v_zu_x,u_xv_y-v_xu_y) \\
u\times v=|u||v|sin\theta
$$



![crossproduct](C:\Users\junhy\Desktop\CGA\crossproduct.png)

- 평행사변형 크기 = 밑변 * 높이 이므로 외적 크기와 같다.
- 파란 삼각형의 면적은 외적의 절반크기이다. == 외적의 크기는 삼각형의 2배이다.

- 세 점이 주어졌을 때 외적을 통한 삼각형 크기 구하기

```c++
double areaTriangle2D(const Point2d& a, const Point2d& b, const Point2d& c)
{
	Vector2f AB = b - a;
	Vector2f AC = c - a;
	double result = crossProduct2D(AB, AC);
	return result / 2;
}
```





### how about the direction of the resulting Vector

- 외적의 부호는 sin값에 의해서만 영향받음 (u , v는 양수이므로)

- $$
  u\times v=|u||v|sin\theta
  $$

- <img src="C:\Users\junhy\Desktop\CGA\sin.png" alt="sin" style="zoom:50%;" />

- $$
  0 < \theta < 180 : Positive \\
  180 < \theta < 360 : Negative \\
  $$



### by convention(Right hand rule), we measure the angle in counterclockwise direction.

<img src="C:\Users\junhy\Desktop\CGA\directions.png" alt="directions" style="zoom:50%;" /> 

- angle from A to E will be the reflex angle with the value two hundred and eighty or degrees angle in counterclockwise direction



## Find relative Position of a Point Compared to a Line segment (좌우판별)

 **위치벡터**(position vector)는 좌표의 원점 O을 처음 점으로 공간 내의 임의의 한 P점을 끝 점으로 하는 벡터이다.

흩어져 있는 벡터들은 서로 연산을 하기도, 평행여부를 확인하기도 어렵다. 그래서 모든 벡터를 평행이동하여 벡터의 처음 점(시점)을 한 점으로 같게 하면 위치벡터가 된다.



- So what is the Vector presenting this line segment?

<img src="C:\Users\junhy\Desktop\CGA\RelativePosition.png" alt="RelativePosition" style="zoom:50%;" /> 

- cross product result will be positive if ABP form counterclockwise rotation
- result in value will be negative If ABP form clockwise rotation 
- we can decide whether the P's left or right to the AB line based
- 외적의 결과는 항상 벡터이므로 업벡터와 내적을 통해 스칼라값을 얻는다.
  - 스칼라 삼중곱 (외적후에 내적)

$$
(AB\times AP)\cdot Y > 0  : 왼쪽 \\
(AB\times AP)\cdot Y < 0  : 오른쪽
$$



요약 

선분에 대한 점을 분류했다.

3점으로 이루어진 삼각형을 계산하는 함수가 필요하다(스칼라 삼중곱)

- LEFT, RIGHT, BEHIND, BEYOND, BETWEEN, ORIGIN, DESTINATION





## Representation of a Line


$$
y  = mX + c \\ 
L(t) = V \times t + a
$$

- Slope intersect form of a line
- Parametric form of a line

<img src="C:\Users\junhy\Desktop\CGA\line.png" alt="line" style="zoom: 33%;" /> 

- 0 < t < 1 
  - Line segment

- t > 0
  - Ray

- -inf < t < +inf
  - Line


###  parametric formulation.

$$
L(t) = v t + a \\
r(x,y,z) = v(k,m,n)t + a(x_{1},y_{1},z_{1}) \\
x = kt + x_{1} \quad y=mt+y_{1}\quad z=nt+z_{1}\\
\therefore t=\frac{x-x_{1}}k = \frac{y-y_{1}}m = \frac{z-z_{1}}n
$$



## Representation of a Plane

<img src="C:\Users\junhy\Desktop\CGA\Plane.png" alt="Plane" style="zoom:50%;" /> 

n is the normal vector to the plane and Point Q is the norm point on the plane

If Point P represent any point in plane, then the direction to P minus Q is perpendicular to the normal Vector n

So the Dot product of those two is zero.
$$
n \cdot (P-Q) = 0 \\
if \quad n (A,B,C),\quad P(x,y,z),\quad Q(x_{1},y_{1},z_{1}) \\ 
\therefore A(x - x_{1}) + B(y - y_{1}) + C(z - z_{1}) = 0 \\
Ax + By + Cz = Ax_{1}+By_{1}+Cz_{1}\\
Ax + By + Cz  = D \\
D = n \cdot Q
$$


### Exercise

Find the Vector equation of the line passing through the point P(2,1,1) and perpendicular to the plane 2x+3y+z = 5.
$$
n = (2,3,1)\\
n \cdot (P-Q) = 0 \\
n \cdot (P(x,y,z))-Q(2,1,1)) = 0 \\
(2,3,1) \cdot  (x-2,y-1,z-1) = 0\\
2(x-2) + 3(y-1) + (z-1) = 0 \\
\therefore 2x+3y+z = 8
$$



### ?? : Ax + By + Cz  = D 평면의 normal vector는  왜 (A,B,C) 일까?

![normalvector1](C:\Users\junhy\Desktop\CGA\normalvector1.png) 

[평면의 법선벡터]: https://www.quora.com/Why-are-A-B-coordinates-of-the-normal-vector-of-line-Ax-By-C-0



### passing three points

```c++
Plane(Point3d& _p1, Point3d& _p2, Point3d& _p3)
{
    auto v12 = _p2 - _p1;
    auto v13 = _p3 - _p1;

    normal = crossProduct2D(v12, v13);
    d = dotProduct(normal, _p1);
}
```

계산 후 법선 방향에서 점을 보면 반시계 방향으로 회전하는 점을 볼 수 있습니다.



## Intersection of two lines



### 두 선분이 교차하는가?

- 조건 : 한 선분의 끝점은 다른 선과 비교하여 다른 면에 있어야 합니다.

<img src="C:\Users\junhy\Desktop\CGA\intersection.png" alt="intersection" style="zoom:50%;" /> 

-  If Point C is on the left side, then the Point D has to be on the right side.
-  And if the point C on the right side, then the Point D has to be on the other side.

```c++
return _xor(ab_c == LEFT, ab_d == LEFT) && _xor(cd_a == LEFT, cd_b == LEFT);
```



-  교차 조건에 포함

<img src="C:\Users\junhy\Desktop\CGA\intersection2.png" alt="intersection2" style="zoom:50%;" /> 

```c++
if (   ab_c == BETWEEN || ab_c == ORIGIN || ab_c == DESTINATION
    || ab_d == BETWEEN || ab_d == ORIGIN || ab_d == DESTINATION
    || cd_a == BETWEEN || cd_a == ORIGIN || cd_a == DESTINATION
    || cd_b == BETWEEN || cd_b == ORIGIN || cd_b == DESTINATION)
{
    return true;
}
```





## Intersection Point of Two Lines

### 두 선분이 교차한다면, 교차점은 어디인가?



equation of AB line
$$
p(t) = v  t + a\\
p(t) = ( b- a) t + a
$$




<img src="C:\Users\junhy\Desktop\CGA\intersectionPoint.png" alt="intersectionPoint" style="zoom:50%;" /> 

- AB , CD가 교차한다면
  - CP 와 CD는 같은 방향벡터를 가짐
  - P는 AB와 CD라인 위에 위치함

- CP direction Vector
  $$
  P(t) - C
  $$
  
- Normal Vector to CD is  n

- CP 벡터와 CD의 법선벡터는 수직이므로 내적값은 0

$$
n \cdot \big(p(t) - C\big) = 0\\
n \cdot \big((b-a)t + a - c\big) = 0\\
n \cdot (a-c)+ n \cdot (b-a)t= 0\\
\therefore t = \frac{n \cdot(c-a)} {n \cdot(b-a)}
$$

### CODE
```c++
// 네개의 점과 교차점
bool Intersection(const Point2d& a, const Point2d& b, const Point2d& c, const Point2d& d, Point2d& intersection)
{
    Vector2f AB = b - a;
    Vector2f CD = d - c;

    Vector2f normal (CD[Y], - CD[X]);

    auto denominator = dotProduct(normal, AB);
    if (isEqualD(denominator, ZERO))
    {
        auto AC = c - a;
        auto Numerator = dotProduct(normal, AC);

        auto t = Numerator / denominator;

        auto x = a[X] + t * AB[X];
        auto y = a[Y] + t * AB[Y];

        intersection.assign(X, x);
        intersection.assign(Y, y);
    }
    
    return false;
}

// 두 선분과 교차점
bool Intersection(const Line2d& l1, const Line2d& l2, Point2d& intersection)
{
	auto l1_start = l1.getPoint();
	auto l1_end = l1_start + l1.getDir();
	auto l2_start = l2.getPoint();
	auto l2_end = l2_start + l2.getDir();

	return Intersection(l1_start, l1_end, l2_start, l2_end, intersection);
}
```





## Angle between Two Lines



내적을 이용!
$$
u\cdot v=|u||v|cos\theta \\
cos\theta=\frac{u\cdot v}{|u||v|}\\
\therefore \theta = \arccos \frac{u\cdot v}{|u||v|}\\
$$

### CODE

```c++
float angleLines2D(const Line2d& l1, const Line2d& l2)
{
	auto l1_mag = l1.getDir().magnitude();
	auto l2_mag = l2.getDir().magnitude();

	auto dot = dotProduct(l1.getDir(), l2.getDir());
	auto angle = acos(fabs(dot) / (l1_mag * l2_mag));
	
	return radiansToDegrees(angle);
}
```



Lines could be?

- 2D 
  - parallel 
  - intersection
- 3D 
  - parallel
  - intersecting
  - skewed 
  - <img src="C:\Users\junhy\Desktop\CGA\skewed.png" alt="skewed" style="zoom: 25%;" /> 
    - which means those are neither intersecting nor parallel.





## Angle between a Line and Plane

<img src="C:\Users\junhy\Desktop\CGA\angleBtwLP.png" alt="angleBtwLP" style="zoom: 33%;" /> 

- 평면과 교차하는 직선의 사잇각은

  평면의 법선과 교차하는 직선과의 각에서 90도를 빼주면 구할 수 있다.



### CODE

 ```c++
 float AngleLinePlane(const Line3d& l1, const Planef& p)
 {
 	auto dot = DotProduct(l1.GetDir(), p.GetNormal());
 	auto angle = acos(fabs(dot));
 	return RadiansToDegrees(angle);
 }
 ```





## Angle between Two Planes

<img src="C:\Users\junhy\Desktop\CGA\angleBtwPP.png" alt="angleBtwPP" style="zoom:33%;" /> 

- 두 평면 사잇각은 두 평면 법선벡터의 사잇각과 같음



### CODE

```c++
```



## Collinear vector

서로 평행하거나 한 직선에 놓여있을 때 조건을 만족한다.

u = K v 로 나타낼 수 있어야한다.

대응하는 계수 사이의 나눗셈은 동일한 값을 산출해야 합니다.
$$
\frac{u[x]}{v[x]} = \frac{u[y]}{v[y]} = \frac{u[z]}{v[z]}
$$
부동소숫점 연산은 비싸므로 같은 결과를 얻기 위해 다른 방법을 사용해보자
$$
\frac{a[x]}{b[x]} = \frac{a[y]}{b[y]} = \frac{a[z]}{b[z]}\\
a[x]b[y] - a[y]b[x] = 0\\
a[y]b[z] - a[z]b[y] = 0\\
a[x]b[z] - a[z]b[x] = 0\\
$$

### CODE

- 벡터

```c++
bool Iscollinear(const Vector3f& a, const Vector3f& b)
{
	float v1 = (a[X] * b[Y] - a[Y] * b[X]);
	float v2 = (a[Y] * b[Z] - a[Z] * b[Y]);
	float v3 = (a[X] * b[Z] - a[Z] * b[X]);

	return IsNearlyEqual(v1, ZERO) && IsNearlyEqual(v2, ZERO) && IsNearlyEqual(v3, ZERO);
}
```

- 세 점

```c++
bool zoonyeok::Iscollinear(const Point3d& a, const Point3d& b, const Point3d& c)
{
	Vector3f AB = b - a;
	Vector3f AC = c - a;
	return Iscollinear(AB, AC);
}
```



## Coplanar vectors

같은 평면에 수평이거나 같은 평면에 놓인 벡터

<img src="C:\Users\junhy\Desktop\CGA\coplanar.png" alt="coplanar" style="zoom:33%;" />

A C D 벡터는 각각 다른 평면에 있으므로  noncoplanar vector

<img src="C:\Users\junhy\Desktop\CGA\coplanar2.png" alt="coplanar2" style="zoom:50%;" />

<img src="C:\Users\junhy\Desktop\CGA\coplanar3.png" alt="coplanar3" style="zoom:50%;" />

D점을 AB , BC 벡터가 놓인 평면으로 옮기면 부피가 0이됨, 모든 벡터가 같은 평면에 있음 

따라서 이 세 벡터가 이루는 **사면체의 부피의 값이 0이면 Coplanar가 된다.**



### volume of tetrahedron

![tetrahedron](C:\Users\junhy\Desktop\CGA\tetrahedron.png)

직육면체를 잘랐을 때 사면체의 부피는 다음과 같다
$$
V = (1/3) * (a*b/2)*c = (1/6)*a*b*c
$$
그리고 abc의 부피는 스칼라 삼중곱으로 구할 수 있다.
$$
V_{tetrahedron} = \frac{1}{6} \big(AB \cdot (AD \times AC))
$$


### CODE

- 스칼라삼중곱

```c++
float ScalarTripleProduct(Vector3f v1, Vector3f v2, Vector3f v3)
{
	Vector3f cross = CrossProduct3D(v2, v3);
	return DotProduct(v1, cross);
}
```

- 벡터 3개

```c++
// 스칼라삼중곱이 0이 될때만 부피의 값이 0이 되므로 명시적으로 1/6을 해줄 필요없음
bool IsCoplanar(const Vector3f a, const Vector3f b, const Vector3f c)
{
	float scalarTripleProduct = ScalarTripleProduct(a, b, c);
	return IsNearlyEqual(scalarTripleProduct, ZERO);
}
```

- 네 점

```c++
bool IsCoplanar(const Point3d& a, const Point3d& b, const Point3d& c, const Point3d& d)
{
	Vector3f AB = b - a;
	Vector3f AC = c - a;
	Vector3f AD = d - a;
	return IsCoplanar(AB, AC, AD);
}
```





## Distance between Point and a Line



<img src="C:\Users\junhy\Desktop\CGA\distancebtw.png" alt="distancebtw" style="zoom:50%;" />

We have to first find the line, which is perpendicular to the given line, which goes to the reference point. 

then if we find the intersection point between the line in the question and the new line, we just found we held the point on the line, which is closest to the reference point.

1. 주어진 선분과 수직이면서 reference point를 지나는 선분을 찾는다
2. 수직인 선분과 원래 선분의 교차점을 찾는다.

<img src="C:\Users\junhy\Desktop\CGA\distancebtwline.png" alt="distancebtwline" style="zoom:50%;" />
$$
x(t) = v t + a \\
v \cdot (Y - x(t)) = 0 \\
v \cdot (Y - (v t + a)) = 0 \\
v\cdot Y - v \cdot a = |v|^2 \cdot t\\
\therefore t = \frac{v \cdot (Y-a)}{|v|^2}
$$

- 만약 v가 정규화되었다면 |v|^2 = 1이므로
  $$
  t = v \cdot (Y-a)
  $$
  
- 두 점 사이의 거리는 Y - X(t)를 찾고 크기를 구하면 됨



### CODE 

```c++
// Line and point
float distance(Line3d& line, Point3d& C)
{
	Vector3f AC = C - line.GetPoint();
	float t = DotProduct(line.GetDirection(), AC);

	Vector3f xt = line.GetPoint() + line.GetDirection() * t;
	Vector3f dist_vec = xt - C;

	return dist_vec.Magnitude();
}
```



선분의 종류는 세 개

- Line
- Line segment
  - 0 < t < 1
  - 0 < t < 1를 벗어난다면 reference point에 가까운 선분의 끝점을 찾고 거리를 구한다.

 <img src="C:\Users\junhy\Desktop\CGA\linesegmentandPoinrt.png" alt="linesegmentandPoinrt" style="zoom:33%;" />

- Ray
  - t가 음수라면 선분의 가장 가까운 점이 Ray의 시작점이 된다
  - 따라서 reference point와 Ray의 시작점의 거리를 구한다.

<img src="C:\Users\junhy\Desktop\CGA\RayandPoint.png" alt="RayandPoint" style="zoom:33%;" /> 





## Distance between Point and a Plane



<img src="C:\Users\junhy\Desktop\CGA\Planeandpoint.png" alt="Planeandpoint" style="zoom:33%;" /> 
$$
Cos\theta = \frac{|XQ|}{|PQ|} \\
|XQ| = |PQ| * Cos\theta \\
|XQ| = |PQ| * \frac{QX \cdot QP}{|QX| |QP|}
$$
vector QX는 평면의 nomal의 collinear이고 반대방향이므로 -n 이 된다.
$$
d = |XQ| = |PQ| * \frac{-nk \cdot (P-Q)}{|-nk||(QP)|} \\
d = \frac{n \cdot (Q - P)}{|n|} \\
d ={n \cdot (Q - P)}{} \\
d = n \cdot Q - n \cdot P
$$

- n is normalized so |n| is 1

- 따라서 점과 평면사이의 거리는 벡터 PQ와 평면의 법선과의 내적으로 구할 수 있다.
- n dot P 는 평면의 법선과 평면 위의 점 사이의 내적으로
  - Plane 클래스 생성자에서 d = dotProduct(normal, _p1)  이미 계산된 값

$$
d = n \cdot Q - r
$$



### Exercise

find the distance between plane 2x+4y+4z-8 = 0 and point 2,2,2
$$
Ax+By+Cz = D\\
n(A,B,C) ,\quad D = Ax1 + By1+Cz1 \\
2x+3y+4z = 8\\
n(2,3,4) \quad, r = 8 \\
$$
 정규화 해줘야 하므로 식은 다음과같다
$$
d = \frac{n \cdot Q - n \cdot P}{|n|} \\
d = \frac{n \cdot Q - r}{|n|} \\
d = \frac{ (2,3,4) \cdot (2,2,2) - 8}{\sqrt29}
$$


### CODE

```C++
float distance(Planef& p, Point3d& Q)
{
	float result = DotProduct(p.GetNormal(), Q) - p.DistancePointToPlane();
	return result;
}
```

- 결과 값이 양수라면 점은 평면과 같은 방향에 놓여있고
- 음수라면 점은 평면과 반대방향에 있다.
- orientation check로 사용됨





## Intersection between Line and a Plane

$$
Q(t) = P + dt \\
Ax + By + Cz = D
$$

- Q(t) : General point on the line
- D is the directional vector
- P is the origin of the line.
- <img src="C:\Users\junhy\Desktop\CGA\PlaneAndLIne.png" alt="PlaneAndLIne" style="zoom:50%;" />

- 선분과 평면의 교차점 R은 평면의 방정식과 선분의 방정식을 만족함
- 선분은 2D 이거나 3D이므로 이 방정식의 성분을 다음과 같이 쓸 수 있음

$$
Q_{x}(t_{1}) = P_{x} + d_{x} t_{1} \\
Q_{y}(t_{1}) = P_{y} + d_{y} t_{1} \\
Q_{z}(t_{1}) = P_{z} + d_{z} t_{1} \\
$$

교차점 R은 평면 위에 있으므로 다음의 방정식을 만족함
$$
A(P_{x} + d_{x}t_{1}) + B(P_{y} + d_{y}t_{1}) + Z(P_{z} + d_{z}t_{1})  = D
$$
이 식을 정리하면
$$
t_{1} = \frac{-(AP_{x} +BP_{y}+CP_{z} ) + D}{Ad_{x} + Bd_{y} + Cd_{z}}
$$
dotproduct between normal vector of the plane and the directional of the line

평면의 법선과 선분과 내적임
$$
t_{1} = \frac{-(n\cdot P) + D }{n \cdot d}
$$
평면이 법선 n을 갖는다면 밑변을 n d(선분의 방향벡터)로 쓸 수 있다

t1을 찾고 선분의 방정식을 계산하면 교차점을 찾을 수 있다. 

1. 만약 n d가 0이라면 선분과 평면이 Collinear 하다는 의미이므로 So line is either parallel to the plane or it is on the plane. 선분은 평면에 수평이거나 평면 위에 있다.

2. If the line is parallel to the plane, there are no intersection points. 선분이 평면에 수직인 경우 , 교차점은 존재하지 않는다.

3. If it is on the plane, then there are infinite number of intersections as those light together. 선분이 평면 위에 있다면 교차점은 무한대이다.



### Exercise

find the intersection point between plane 2x + 3y + 4z = 8 and the line go through A(2,2,3) and B(4,3,6)
$$
t_{1} = \frac{-(n\cdot P) + D }{n \cdot d}
$$

1. plane normal  :  n
2. line directional vector  :  d
3. Line origin :  P
4. plane constant  : D

- n  = (2,3,4) 
- d  = BA (2,1,3)
- P = A(2,2,3)
- D = 8

$$
t_{1} = \frac{-14}{19}\\
Q(t_{1}) = P + dt_{1}\\
$$

따라서 교차점은 
$$
Q(t_{1}) =(0.53,1.26,0.79)
$$


### CODE

```c++
bool Intersection(const Line3d& line, const Planef& plane, Point3d& point)
{
	Vector3f n = plane.GetNormal();
	float D = plane.DistancePointToPlane();
	Vector3f d = line.GetDirection();
	Vector3f P = line.GetPoint();
	float dotnd = DotProduct(n, d);

    // 선분이 평면에 수직인 경우, 교차점은 존재하지 않는다.
	if (IsNearlyEqual(dotnd, ZERO))
	{
		return false;
	}

	float t = (-1 * DotProduct(n, P) + D) / dotnd;
	point.Assign(X, P[X] + t * d[X]);
	point.Assign(Y, P[Y] + t * d[Y]);
	point.Assign(Z, P[Z] + t * d[Z]);
	return true;
}
```





## Intersection between two Plane

- 두 평면의 교차점은 한 점이 아닌 선분이 된다.

<img src="C:\Users\junhy\Desktop\CGA\TwoPlanes.png" style="zoom:50%;" /> 

 we need to find the direction Vector and a point on the line formed by the intersecting planes

교차하는 평면의 선분 위의 점과 방향벡터를 구해야한다.

The general equation of a plane은 
$$
Ax + By + Cz = D\\
n \cdot P = d \\
$$

$$
n_{1} \cdot P_{1} = d_{1} \\
n_{2} \cdot P_{2} = d_{2}
$$

평면의 direction은 두 평면의 법선의 외적으로 구할 수 있다.

Line Direction Vector = n1 X n2

So if the point on this line is considered as R, then we can write the R 
$$
R = an_{1} + bn_{2}
$$
점 R은 두 평면위에 있으므로, 두 평면의 방정식에 적용할 수 있다.
$$
n_{1} \cdot (an_{1} + bn_{2}) = d_{1} \\
n_{2} \cdot (an_{1} + bn_{2}) = d_{2}
$$
이 두 방정식으로 a,b값을 찾을 수 있다
$$
an_{1} \cdot n_{1} + bn_{1}\cdot n_{2} = d_{1} \\
a|n_{1}|^2 + bn_{1}\cdot n_{2} = d_{1} \\
$$

$$
an_{2} \cdot n_{1} + bn_{2}\cdot n_{2} = d_{2} \\
an_{2} \cdot n_{1} + b|n_{2}|^2 = d_{2} \\
$$

이 방정식을 풀면
$$
b = \frac{d_{1}n_{2} \cdot n_{1} - d_{2}|n_{1}|^2}{((n_{1} \cdot n_{2})^2 - |n_{2}|^2|n_{1}|^2 )}
$$

$$
a = \frac{d_{2}n_{2} \cdot n_{1} - d_{1}|n_{2}|^2}{((n_{1} \cdot n_{2})^2 - |n_{2}|^2|n_{1}|^2 )}
$$

$$
R = an_{1} + bn_{2} \\
Direction = n_{1} \times n_{2}
$$



### Exercise

find the intersection line between plane 2x+3y+4z = 8 and plane 3x+4y+8z = 4
$$
dir = n_{1} \times n_{2} \\
n_{1} = (2,3,4) \\
n_{2} = (3,4,8) \\
$$
dir = (8,-4,-1)
$$
a = \frac{d_{2}n_{2} \cdot n_{1} - d_{1}|n_{2}|^2}{((n_{1} \cdot n_{2})^2 - |n_{2}|^2|n_{1}|^2 )}
$$

- |n1| = sprt(29)
- |n2| = sprt(89)
- n2 dot n1 = 50
- d1 = 8
- d2 = 4
- a = 512 / 81

$$
b = \frac{d_{1}n_{2} \cdot n_{1} - d_{2}|n_{1}|^2}{((n_{1} \cdot n_{2})^2 - |n_{2}|^2|n_{1}|^2 )}
$$

- b = 284 / 81

$$
R = an_{1} + bn_{2} \\
$$

- a = 512 / 81
- b = 284 / 81



### CODE

Normal vector를 적용하면 Magnitude는 1이 되므로 식은 다음과 같다.
$$
b = \frac{d_{1}n_{2} \cdot n_{1} - d_{2}}{((n_{1} \cdot n_{2})^2 - 1 )} \\
a = \frac{d_{2}n_{2} \cdot n_{1} - d_{1}}{((n_{1} \cdot n_{2})^2 - 1 )}
$$

```c++
bool Intersection(const Planef& plane1, Planef& plane2, Line3d& line)
{
	Vector3f n1 = plane1.GetNormal();
	Vector3f n2 = plane2.GetNormal();
	float d1 = plane1.DistancePointToPlane();
	float d2 = plane2.DistancePointToPlane();

	Vector3f direction = CrossProduct3D(n1, n2);

	// check if the planes are parallel.
	if (IsNearlyEqual(direction.Magnitude(), ZERO)) return false;

	float n1n2 = DotProduct(n1, n2);
	float n1n2_2 = n1n2 * n1n2;

	float a = (d2 * n1n2 - d1) / (n1n2_2 - 1);
	float b = (d1 * n1n2 - d2) / (n1n2_2 - 1);

	Vector3f point = n1 * a + n2 * b;

	line.SetPoint(point);
	direction.Normalize();
	line.SetDirection(direction);

	return true;
}
```



## Overview of needed DataStructure and Algorithms

- List
- Stack
- Queue



### std::list

- Doubly connected list
- 원소를 찾은 다음 삽입과 삭제가 상수시간에 가능

### std::vector

- 연속적인 메모리 공간에 저장됨
- 캐시 최적화로 효율적인 메모리 접근이 가능
- 원소 삭제시 성능이 떨어짐
