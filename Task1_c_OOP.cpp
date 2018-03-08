#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define fpair pair<float,pair< pair<int,int>,int> >
#define triplet pair< pair<int,int>,int>
#define make_pair3(a,b,c) make_pair(make_pair(a,b),c)

#define width 10
#define breadth 20

struct vertex
{
  int state;
  int orgstate;
  triplet parent;
  float f,g,h;
};

class PathPlanner
{
  private:
    int rows,cols;
    int rotState,rotateby;
    triplet src,des;
    bool destreach;
    Mat img;
    multiset <fpair > openList;
    vector< vector<vector<vertex> > > matrix;
    //vector< vector<bool> > closedList(1000,vector<vector<bool> >(1000, vector<bool>(rotState)));
    bool closedList[1000][1000][50];
    bool isDestination(vertex);
    bool isValid(triplet);
    bool access(vertex);
    float heuristic(int,int,int,triplet);
    void sucessor(int,int,int,int,float,int,int,int);
    void tracePath(int,int,int);
    void minkowski(int,int,int);
    void makebot(int,int,int);
    void DrawRectangleWhite(cv::Mat&,cv::Point,cv::Size,int);
    void DrawRectangleBlue(cv::Mat&,cv::Point,cv::Size,int);

  public:
    PathPlanner(Mat,int,int);
    void initialize();
    void tracePath(int,int);
    void displayImage();
    void AStarSearch();
    void CreateMinkowski();
};

void PathPlanner::displayImage()
{
  imshow("Final Image",img);
}


Mat imgt=imread("task.png",1);

PathPlanner::PathPlanner(Mat A,int x,int y)
{
  img=A;
  rotState=36;
  rotateby=360/rotState;
  destreach=false;
  rows=x;
  cols=y;
  matrix.resize(1000,vector<vector<vertex> >(1000, vector<vertex>(rotState)));
}

Mat img;
Mat imgRotateTemp;



void PathPlanner::DrawRectangleWhite(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, int rotationDegrees)
{
    cv::Scalar color = cv::Scalar(255.0, 255.0, 255.0);
    cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);
    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i)
    {
        vertices[i] = vertices2f[i];
    }
    cv::fillConvexPoly(image,vertices,4,color);
}


void PathPlanner::DrawRectangleBlue(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, int rotationDegrees)
{
    cv::Scalar color = cv::Scalar(255.0,0,0);
    cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);
    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i)
    {
        vertices[i] = vertices2f[i];
    }
    cv::fillConvexPoly(image,vertices,4,color);
}

int main(int argc,char** argv)
{
   img=imread("task.png",1);
   PathPlanner Path(img,img.rows,img.cols);
   Path.initialize();
   Path.AStarSearch();
   Path.displayImage();
   //imwrite("Task1_b.png",img);
   waitKey(0);
   destroyAllWindows();
  return 0;
}








void PathPlanner::initialize()
{
  int ismax=0,ismin=INT_MAX,idmax=0,idmin=INT_MAX;
  int jsmax=0,jsmin=INT_MAX,jdmax=0,jdmin=INT_MAX;

  cout<<"ROWS      :    "<<rows<<endl;
  cout<<"COLOUMNS  :    "<<cols<<endl;

  for(int i=0;i<rows;i++)
  {
    for(int j=0;j<cols;j++)
    {
      matrix[i][j][0].f=matrix[i][j][0].g=matrix[i][j][0].h=FLT_MAX;
      matrix[i][j][0].parent.first.first=matrix[i][j][0].parent.first.second=-1;

      Vec3b store=img.at<Vec3b>(i,j);
      if(store[0]==0 && store[1]==0 && store[2]==0)
      {
      matrix[i][j][0].state=1;
      matrix[i][j][0].orgstate=1;
      }
      else if(store[0]==0 && store[1]>0 && store[2]==0)
      {
        matrix[i][j][0].state=4;
        matrix[i][j][0].orgstate=4;
        if(i>idmax)idmax=i;
        if(i<idmin)idmin=i;
        if(j>jdmax)jdmax=j;
        if(j<jdmin)jdmin=j;
      }
      else if(store[0]==0 && store[1]==0 && store[2]>0)
      {
        matrix[i][j][0].state=0;
        matrix[i][j][0].orgstate=0;
        if(i>ismax)ismax=i;
        if(i<ismin)ismin=i;
        if(j>jsmax)jsmax=j;
        if(j<jsmin)jsmin=j;
      }
      else if(store[0]>0 && store[1]>0 && store[2]>0)
      {
      matrix[i][j][0].state=-1;
      matrix[i][j][0].orgstate=-1;
      }
      else
      {
      matrix[i][j][0].state=1;
      matrix[i][j][0].orgstate=1;
     }
    }
   }

   for(int i=0;i<rows;i++)
   {
     for(int j=0;j<cols;j++)
     {
       for(int k=1;k<rotState;k++)
       {
         matrix[i][j][k]=matrix[i][j][0];
       }
     }
   }

 imshow("Before Minkowski",img);

   src=make_pair3((ismax+ismin)/2,(jsmax+jsmin)/2,0);
   des=make_pair3((idmax+idmin)/2,(jdmax+jdmin)/2,0);

   printf("Source        :(%d,%d)\n",src.first.first,src.first.second);
   printf("Destination   :(%d,%d)\n",des.first.first,des.first.second);
   cout<<"Angles To turn:"<<rotState<<endl;


for(int k=0;k<rotState;k++)                           //3D-CONFIGURATION SPACE GENERATED
{
   imgRotateTemp=imread("task.png");

   for(int i=0;i<rows;i++)
   {
     for(int j=0;j<cols;j++)
     {
       if(matrix[i][j][k].state==-1)
       {
         minkowski(j,i,k*rotateby);
       }
     }
  /*  imshow("Terribly Intermediate minkowski",imgRotateTemp);
     waitKey(0);*/
   }

  //imshow("Intermediate minkowski",imgRotateTemp);
   cout<<"calculating layer "<<k<<endl;
  // waitKey(0);

   for(int i=0;i<rows;i++)
   {
     for(int j=0;j<cols;j++)
     {
       matrix[i][j][k].f=matrix[i][j][k].g=matrix[i][j][k].h=FLT_MAX;
       matrix[i][j][k].parent.first.first=matrix[i][j][k].parent.first.second=-1;

       Vec3b store=imgRotateTemp.at<Vec3b>(i,j);

       if(store[0]==0 && store[1]==0 && store[2]==0)
       {
       matrix[i][j][k].state=1;
       matrix[i][j][k].orgstate=1;
       }
       else if(store[0]==0 && store[1]>0 && store[2]==0)
       {
         matrix[i][j][k].state=4;
         matrix[i][j][k].orgstate=4;
         if(i>idmax)idmax=i;
         if(i<idmin)idmin=i;
         if(j>jdmax)jdmax=j;
         if(j<jdmin)jdmin=j;
       }
       else if(store[0]==0 && store[1]==0 && store[2]>0)
       {
         matrix[i][j][k].state=0;
         matrix[i][j][k].orgstate=0;
         if(i>ismax)ismax=i;
         if(i<ismin)ismin=i;
         if(j>jsmax)jsmax=j;
         if(j<jsmin)jsmin=j;
       }
       else if(store[0]>0 && store[1]>0 && store[2]>0)
       {
       matrix[i][j][k].state=-1;
       matrix[i][j][k].orgstate=-1;
       }
       else
       {
       matrix[i][j][k].state=1;
       matrix[i][j][k].orgstate=1;
      }
     }
   }

}
}






bool PathPlanner::isValid (triplet a)
{
  return((a.first.first>=0 && a.first.first<rows) && (a.first.second>=0 && a.first.second<cols)  && (a.second>=0 && a.second<rotState));
}

bool PathPlanner::access(vertex a)
{
  if(a.state==-1) return false;
  else return true;
}

bool PathPlanner::isDestination(vertex a)
{
  if(a.state==4)
  return true;
  else
  return false;
}

float PathPlanner::heuristic(int i,int j,int k,triplet a)
{
  return (sqrt((i-a.first.first)*(i-a.first.first)+(j-a.first.second)*(j-a.first.second)+(k-a.first.second)*(k-a.first.second)));
}

void PathPlanner::AStarSearch()
{
  int i,j,k;
  i=src.first.first;
  j=src.first.second;
  k=0;

  matrix[i][j][k].f=0.0;
  matrix[i][j][k].g=0.0;
  matrix[i][j][k].h=0.0;
  matrix[i][j][k].parent.first.first=i;
  matrix[i][j][k].parent.first.second=j;
  matrix[i][j][k].parent.second=k;

  openList.insert(make_pair(0.0,matrix[i][j][k].parent));

  while(openList.empty()==false)
  {
    fpair p=*openList.begin();
    openList.erase(openList.begin());
    i=p.second.first.first;
    j=p.second.first.second;
    k=p.second.second;
    closedList[i][j][k]=true;

    sucessor(2,i-1,j-1,k,matrix[i][j][k].g,i,j,k);    //north-west
    if(destreach==true) return;
    sucessor(3,i-1,j-1,k+1,matrix[i][j][k].g,i,j,k);    //north-west
    if(destreach==true) return;
    sucessor(3,i-1,j-1,k-1,matrix[i][j][k].g,i,j,k);    //north-west
    if(destreach==true) return;

    sucessor(1,i-1,j,k,matrix[i][j][k].g,i,j,k);      //north
    if(destreach==true) return
    sucessor(2,i-1,j,k+1,matrix[i][j][k].g,i,j,k);      //north
    if(destreach==true) return;
    sucessor(2,i-1,j,k-1,matrix[i][j][k].g,i,j,k);      //north
    if(destreach==true) return;

    sucessor(2,i-1,j+1,k,matrix[i][j][k].g,i,j,k);    //north-east
    if(destreach==true) return;
    sucessor(3,i-1,j+1,k+1,matrix[i][j][k].g,i,j,k);    //north-east
    if(destreach==true) return;
    sucessor(3,i-1,j+1,k-1,matrix[i][j][k].g,i,j,k);    //north-east
    if(destreach==true) return;

    sucessor(1,i,j+1,k,matrix[i][j][k].g,i,j,k);      //east
    if(destreach==true) return;
    sucessor(2,i,j+1,k+1,matrix[i][j][k].g,i,j,k);      //east
    if(destreach==true) return;
    sucessor(2,i,j+1,k-1,matrix[i][j][k].g,i,j,k);      //east
    if(destreach==true) return;

    sucessor(2,i+1,j+1,k,matrix[i][j][k].g,i,j,k);    //south-east
    if(destreach==true) return;
    sucessor(3,i+1,j+1,k+1,matrix[i][j][k].g,i,j,k);    //south-east
    if(destreach==true) return;
    sucessor(3,i+1,j+1,k-1,matrix[i][j][k].g,i,j,k);    //south-east
    if(destreach==true) return;

    sucessor(1,i+1,j,k,matrix[i][j][k].g,i,j,k);     //south
    if(destreach==true) return;
    sucessor(2,i+1,j,k+1,matrix[i][j][k].g,i,j,k);     //south
    if(destreach==true) return;
    sucessor(2,i+1,j,k-1,matrix[i][j][k].g,i,j,k);     //south
    if(destreach==true) return;

    sucessor(2,i+1,j-1,k,matrix[i][j][k].g,i,j,k);   //south-west
    if(destreach==true) return;
    sucessor(3,i+1,j-1,k+1,matrix[i][j][k].g,i,j,k);   //south-west
    if(destreach==true) return;
    sucessor(3,i+1,j-1,k-1,matrix[i][j][k].g,i,j,k);   //south-west
    if(destreach==true) return;

    sucessor(1,i,j-1,k,matrix[i][j][k].g,i,j,k);     //west
    if(destreach==true) return;
    sucessor(2,i,j-1,k+1,matrix[i][j][k].g,i,j,k);     //west
    if(destreach==true) return;
    sucessor(2,i,j-1,k-1,matrix[i][j][k].g,i,j,k);     //west
    if(destreach==true) return;

    sucessor(1,i,j,k+1,matrix[i][j][k].g,i,j,k);     //top
    if(destreach==true) return;
    sucessor(1,i,j,k-1,matrix[i][j][k].g,i,j,k);     //bottom
    if(destreach==true) return;
  }
}

void PathPlanner::sucessor(int state,int i,int j,int k,float cost,int ii,int jj,int kk)
{
   float g,h,f;

   if(isValid(make_pair3(i,j,k))==true)
   {

     if(isDestination(matrix[i][j][k])==true)
     {
       matrix[i][j][k].parent.first.first=ii;
       matrix[i][j][k].parent.first.second=jj;
       matrix[i][j][k].parent.second=kk;
       destreach=true;
       cout<<"Destination Reached\n"<<endl;
       tracePath(i,j,k);
       return;
     }
     else if(closedList[i][j][k]==false && access(matrix[i][j][k])==true)
     {
       if(state==1)
       g=cost+1;
       else if(state==2)
       g=cost+sqrt(2);
       else
       g=cost+sqrt(3);

       h=heuristic(i,j,k,des);
       f=g+h;

       if(matrix[i][j][k].f==FLT_MAX||matrix[i][j][k].f>f)
       {
         openList.insert(make_pair(f,make_pair3(i,j,k)));
         matrix[i][j][k].f=f;
         matrix[i][j][k].g=g;
         matrix[i][j][k].h=h;
         matrix[i][j][k].parent.first.first=ii;
         matrix[i][j][k].parent.first.second=jj;
         matrix[i][j][k].parent.second=kk;
       }

     }
   }
}

void PathPlanner::tracePath(int i,int j,int k)
{
  printf("Current node  :(%d,%d,%d)\n",i,j,k);
  img.at<Vec3b>(i,j)[0]=255;
  triplet t=make_pair3(i,j,k);
  makebot(j,i,k*rotateby);
  if(t==matrix[i][j][k].parent)
  {
    return;
  }
  if(matrix[i][j][k].state==0)
  return;
  else
  tracePath(matrix[i][j][k].parent.first.first,matrix[i][j][k].parent.first.second,matrix[i][j][k].parent.second);
}

void PathPlanner::minkowski(int i,int j,int k)
{
  Point t;
  t.x=i;
  t.y=j;
  DrawRectangleWhite(imgRotateTemp,t,Size(width,breadth),k);
}

void PathPlanner::makebot(int i, int j,int k)
{
    Mat temp=imread("task.png",1);
  Point t;
  t.x=i;
  t.y=j;

  DrawRectangleBlue(temp,t,Size(width,breadth),k);
    waitKey(30);
  imshow("In Transition:",temp);
}
