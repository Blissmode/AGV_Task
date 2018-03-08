#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define fpair pair<float,pair<int,int> >
#define width 10
#define breadth 20

struct vertex
{
  int state;
  int orgstate;
  pair<int,int> parent;
  float f,g,h;
};

class PathPlanner
{
  private:
    int rows,cols;
    pair<int,int> src,des;
    bool destreach;
    Mat img;
    multiset <fpair > openList;
    bool isValid(pair<int,int>);
    bool access(vertex);
    float heuristic(int,int,pair<int,int>);
    void sucessor(int,int,int,float,int,int);
    vector< vector<vertex> > matrix;
    vector< vector<bool> > closedList;
    bool isDestination(vertex);
    void minkowski(int,int);
    void makebot(int,int);

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
  destreach=false;
  rows=x;
  cols=y;
  matrix.resize(1000,vector<vertex>(1000));
  closedList.resize(1000,vector<bool>(1000,false));
}

Mat img;

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
    matrix[i][j].f=matrix[i][j].g=matrix[i][j].h=FLT_MAX;
    matrix[i][j].parent.first=matrix[i][j].parent.second=-1;

    Vec3b store=img.at<Vec3b>(i,j);
    if(store[0]==0 && store[1]==0 && store[2]==0)
    matrix[i][j].state=1;
    else if(store[0]==0 && store[1]>0 && store[2]==0)
    {
      matrix[i][j].state=4;
      if(i>idmax)idmax=i;
      if(i<idmin)idmin=i;
      if(j>jdmax)jdmax=j;
      if(j<jdmin)jdmin=j;
    }
    else if(store[0]==0 && store[1]==0 && store[2]>0)
    {
      matrix[i][j].state=0;
      if(i>ismax)ismax=i;
      if(i<ismin)ismin=i;
      if(j>jsmax)jsmax=j;
      if(j<jsmin)jsmin=j;
    }
    else if(store[0]>0 && store[1]>0 && store[2]>0)
    matrix[i][j].state=-1;
    else
    matrix[i][j].state=1;
  }
 }

 src=make_pair((ismax+ismin)/2,(jsmax+jsmin)/2);
 des=make_pair((idmax+idmin)/2,(jdmax+jdmin)/2);

 printf("Source        :(%d,%d)\n",src.first,src.second);
 printf("Destination   :(%d,%d)\n",des.first,des.second);
}












int main(int argc,char** argv)
{
  img=imread("task.png",1);
  PathPlanner Path(img,img.rows,img.cols);
  Path.initialize();
  Path.CreateMinkowski();
  Path.AStarSearch();
  Path.displayImage();
//   imwrite("Task1_a.png",img);
  waitKey(0);
  destroyAllWindows();
return 0;
}




void PathPlanner::CreateMinkowski()
{
    for(int i=0;i<rows;i++)
    {
      for(int j=0;j<cols;j++)
      {
        if(matrix[i][j].orgstate==-1)
         minkowski(i,j);
      }
    }
}

bool PathPlanner::isValid (pair<int,int> a)
{
  return((a.first>=0 && a.first<rows) && (a.second>=0 && a.second<cols));
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

float PathPlanner::heuristic(int i,int j,pair<int,int> a)
{
  return (sqrt((i-a.first)*(i-a.first)+(j-a.second)*(j-a.second)));
}

void PathPlanner::AStarSearch()
{
  int i,j;
  i=src.first;
  j=src.second;
  matrix[i][j].f=0.0;
  matrix[i][j].g=0.0;
  matrix[i][j].h=0.0;
  matrix[i][j].parent.first=i;
  matrix[i][j].parent.second=j;

  openList.insert(make_pair(0.0,matrix[i][j].parent));

  while(openList.empty()==false)
  {
    fpair p=*openList.begin();
    openList.erase(openList.begin());
    i=p.second.first;
    j=p.second.second;
//    printf("Current node f  :(%d,%d)\n",i,j);
    closedList[i][j]=true;

    sucessor(1,i-1,j-1,matrix[i][j].g,i,j);    //north-west
    if(destreach==true) return;
    sucessor(2,i-1,j,matrix[i][j].g,i,j);      //north
    if(destreach==true) return;
    sucessor(3,i-1,j+1,matrix[i][j].g,i,j);    //north-east
    if(destreach==true) return;
    sucessor(4,i,j+1,matrix[i][j].g,i,j);      //east
    if(destreach==true) return;
    sucessor(5,i+1,j+1,matrix[i][j].g,i,j);    //south-east
    if(destreach==true) return;
    sucessor(6,i+1,j,matrix[i][j].g,i,j);     //south
    if(destreach==true) return;
    sucessor(7,i+1,j-1,matrix[i][j].g,i,j);   //south-west
    if(destreach==true) return;
    sucessor(8,i,j-1,matrix[i][j].g,i,j);     //west
    if(destreach==true) return;


  }
}

void PathPlanner::sucessor(int state,int i,int j,float cost,int ii,int jj)
{
  float g,h,f;
   if(isValid(make_pair(i,j))==true)
   {

     if(isDestination(matrix[i][j])==true)
     {
       matrix[i][j].parent.first=ii;
       matrix[i][j].parent.second=jj;
       destreach=true;
       cout<<"Destination Reached\n"<<endl;
       tracePath(i,j);
       return;
     }
     else if(closedList[i][j]==false && access(matrix[i][j])==true)
     {
       if(state%2==0)
       g=cost+1;
       else
       g=cost+sqrt(2);
       h=heuristic(i,j,des);
       f=g+h;

       if(matrix[i][j].f==FLT_MAX||matrix[i][j].f>f)
       {
         openList.insert(make_pair(f,make_pair(i,j)));
         matrix[i][j].f=f;
         matrix[i][j].g=g;
         matrix[i][j].h=h;
         matrix[i][j].parent.first=ii;
         matrix[i][j].parent.second=jj;
       }

     }
   }
}

void PathPlanner::tracePath(int i,int j)
{
  printf("Current node  :(%d,%d)\n",i,j);
  img.at<Vec3b>(i,j)[0]=255;
  pair<int,int> t=make_pair(i,j);
  makebot(i,j);
  if(t==matrix[i][j].parent)
  {
    return;
  }
  if(matrix[i][j].state==0)
  return;
  else
  tracePath(matrix[i][j].parent.first,matrix[i][j].parent.second);

}

void PathPlanner::minkowski(int i,int j)
{
int imax=i+width;
int jmax=j-breadth;
int jtemp=j;

  for(;i<=imax;i++)
  {
    for(j=jtemp;j>jmax;j--)
    {
      if(isValid(make_pair(i,j)))
      {
        matrix[i][j].state=-1;
        imgt.at<Vec3b>(i,j)[0]=255;
        imgt.at<Vec3b>(i,j)[1]=255;
        imgt.at<Vec3b>(i,j)[2]=255;
      }
      else
      continue;
    }
  }
}

void PathPlanner::makebot(int i, int j)
{
  int imax=i-width;
  int jmax=j+breadth;
  int jtemp=j;
  Mat temp=imread("task.png",1);


  for(;i>imax;i--)
  {
    for(j=jtemp;j<jmax;j++)
    {
      if(isValid(make_pair(i,j)))
      {
        temp.at<Vec3b>(i,j)[0]=255;
        temp.at<Vec3b>(i,j)[1]=0;
        temp.at<Vec3b>(i,j)[2]=0;
      }
      else
      continue;
    }
  }
    waitKey(30);
  imshow("In Transition:",temp);
}
