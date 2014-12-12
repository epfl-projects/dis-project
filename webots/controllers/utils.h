/* 
  * Code snippets borrowed from the following sources:
  *  - www.geeksforgeeks.org
  *  - Emile Cormier, http://stackoverflow.com
  *  - http://paulbourke.net/geometry/polygonmesh/source1.c
*/
#include <math.h> 
#define INF 10000
#define EPSILON 0.001 



typedef struct Point
{
  double x;
  double y;
} Point;



//******************************************************* 
//      CENTROID OF A POLYGON
//*******************************************************

Point getCentroid(Point vertices[], int vertexCount) {
  Point centroid = {0, 0};
  double signedArea = 0.0;
  double x0 = 0.0; // Current vertex X
  double y0 = 0.0; // Current vertex Y
  double x1 = 0.0; // Next vertex X
  double y1 = 0.0; // Next vertex Y
  double a = 0.0;  // Partial signed area

  // For all vertices except last
  int i=0;
  for (i=0; i<vertexCount-1; ++i)
  {
    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[i+1].x;
    y1 = vertices[i+1].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;
  }

  // Do last vertex
  x0 = vertices[i].x;
  y0 = vertices[i].y;
  x1 = vertices[0].x;
  y1 = vertices[0].y;
  a = x0*y1 - x1*y0;
  signedArea += a;
  centroid.x += (x0 + x1)*a;
  centroid.y += (y0 + y1)*a;

  signedArea *= 0.5;
  centroid.x /= (6.0*signedArea);
  centroid.y /= (6.0*signedArea);

  return centroid;
}





//******************************************************* 
//      POLYGON AREA
//*******************************************************

double PolygonArea(Point *polygon, int N)
{
  int i,j;
  double area = 0;

  for (i=0;i<N;i++) {
    j = (i + 1) % N;
    area += polygon[i].x * polygon[j].y;
    area -= polygon[i].y * polygon[j].x;
  }

  area /= 2;
  return(area < 0 ? -area : area);
}






//******************************************************* 
//      CONVEX HULL
//*******************************************************




// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

  if (fabs(val) < EPSILON) return 0;  // colinear
  return (val > EPSILON)? 1: 2; // clock or counterclock wise
}



// array 'hull' contains convex hull
int convexHull(Point points[], int n, Point hull[])
{
    // There must be at least 3 points
    if (n < 3) return 0;
 
    // Initialize Result
    int next[n];
    for (int i = 0; i < n; i++)
        next[i] = -1;
 
    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < n; i++)
        if (points[i].x < points[l].x)
            l = i;
 
    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again
    int p = l, q;
    do
    {
      // Search for a point 'q' such that orientation(p, i, q) is
      // counterclockwise for all points 'i'
      q = (p+1)%n;
      for (int i = 0; i < n; i++)
        if (orientation(points[p], points[i], points[q]) == 2)
           q = i;

      next[p] = q;  // Add q to result as a next point of p
      p = q; // Set p as q for next iteration
    } while (p != l);
 
    
    int j = 0;
    int starting_index = -1, current_index = -1;
    // find first index whose value is not -1
    for (int i = 0; i < n; i++)
    { 
      if (next[i] != -1) {
        starting_index = i;
        current_index = i;
        break;
      }  
    }
    do {
      hull[j].x = points[current_index].x;
      hull[j].y = points[current_index].y;
      j++;
      current_index = next[current_index];
    } while (current_index != starting_index);
    return j;
}

