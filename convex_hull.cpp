#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

// Andrew 算法求凸包
// 算法过程：
//  第一步： 首先将所有点排个序，以x坐标为第一关键字，以y坐标为第二关键字。
//  第二步： 从左至右维护上半部分的边界，从右至左维护下半部分的边界
// 2D point struct
struct Point {
  double x, y;
  // 重载运算符，用于排序
  bool operator<(const Point &p) const {
    return x < p.x || (x == p.x && y < p.y);
  }
};
// 计算叉积, 用于判断点的位置关系，oa x ob, >0 b在a的左侧， <0 b在a的右侧
double cross(const Point &O, const Point &A, const Point &B) {
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// andrew 算法求凸包
vector<Point> convexhull(vector<Point> &points) {
  int n = points.size();
  vector<Point> hull;

  // 将传入的points按照从左到右的顺序排序，如果x一样则按y大小
  sort(points.begin(), points.end());

  // 下凸壳，如果在连线左边，就推出栈
  for (int i = 0; i < n; i++) {
    while (hull.size() > 2 &&
           cross(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
      hull.pop_back();
    }
    hull.push_back(points[i]);
  }
  // 上凸壳，如果在连线右边，就推出栈
  for (int i = n -2; i >=0; i--) {
    while (hull.size() > 2 &&
           cross(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
      hull.pop_back();
    }
    hull.push_back(points[i]);
  }
  // 删除重复的点

  return hull;
}
// 测试
int main() {
  vector<Point> points;
  Point p1 = {1, 1};
  Point p2 = {2, 2};
  Point p3 = {3, 3};
  Point p4 = {4, 4};
  Point p5 = {5, 5};
  Point p6 = {6, 6};
  Point p7 = {7, 7};
  Point p8 = {8, 8};
  Point p9 = {9, 9};
  Point p10 = {10, 10};
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);
  points.push_back(p7);
  points.push_back(p8);
  points.push_back(p9);
  points.push_back(p10);
  vector<Point> hull = convexhull(points);
  for (int i = 0; i < hull.size(); i++) {
    cout << hull[i].x << " " << hull[i].y << endl;
  }
  return 0;
}
