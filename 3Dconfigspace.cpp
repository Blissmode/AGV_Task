#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define fpair pair<float,pair< pair<int,int>,int> >
#define triplet pair< pair<int,int>,int>
#define make_pair3(a,b,c) make_pair(make_pair(int,int),int)

#define width 10
#define breadth 20

int rotState=10;
int rotateby=360/rotState;


struct vertex
{
  int state;
  int orgstate;
  triplet parent;
  float f,g,h;
};


Mat img=imread("task.png",1);
Mat imgt=imread("task.png",1);
triplet src,des;
multiset <fpair > openList;

int rows,cols;

bool isValid(triplet);
bool access(vertex);
float heuristic(int,int,int,triplet);
void AStarSearch();
void sucessor(int,int,int,float,int,int);
void tracePath(int,int);
void minkowski(int,int,int);
void makebot(int,int);

int ismax=0,ismin=INT_MAX,idmax=0,idmin=INT_MAX;
int jsmax=0,jsmin=INT_MAX,jdmax=0,jdmin=INT_MAX;
bool destreach=false;


vector< vector<vector<vertex> > > matrix(1000,vector<vector<vertex> >(1000, vector<vertex>(rotState)));
vector< vector<bool> > closedList(1000,vector<vector<bool> >(1000, vector<vertex>(rotState,false)));

void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, int rotationDegrees)
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

int main(int argc,char** argv)
{
  rows=img.rows;
  cols=img.cols;

  cout<<"ROWS      :    "<<rows<<endl;
  cout<<"COLOUMNS  :    "<<cols<<endl;

  for(int i=0;i<rows;i++)
  {
    for(int j=0;j<cols;j++)
    {
      matrix[i][j][0].f=matrix[i][j][0].g=matrix[i][j][0].h=FLT_MAX;
      matrix[i][j].parent.first.first=matrix[i][j].parent.first.second=-1;

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


   src=make_pair3((ismax+ismin)/2,(jsmax+jsmin)/2,0);
   des=make_pair3((idmax+idmin)/2,(jdmax+jdmin)/2,0);

   printf("Source        :(%d,%d)\n",src.first.first,src.first.second);
   printf("Destination   :(%d,%d)\n",des.first.first,des.first.second);

   for(int i=0;i<rows;i++)
   {
     for(int j=0;j<cols;j++)
     {
       if(matrix[i][j].orgstate==-1)
       {
         for(int k=0;k<rotState;k++)
         minkowski(i,j,k);
       }
     }
   }

   AStarSearch();

   imshow("Final image",img);
   imshow("minkowski",imgt);
   imwrite("Task1_b.png",img);
   waitKey(0);
   destroyAllWindows();
  return 0;
}
















bool isValid (triplet a)
{
  return((a.firts.first>=0 && a.first.first<rows) && (a.first.second>=0 && a.first.second<cols)  && (a.second>=0 && a.second<rotState));
}

bool access(vertex a)
{
  if(a.state==-1) return false;
  else return true;
}

bool isDestination(vertex a)
{
  if(a.state==4)
  return true;
  else
  return false;
}

float heuristic(int i,int j,int k,triplet a)
{
  return (sqrt((i-a.first.first)*(i-a.first.first)+(j-a.first.second)*(j-a.first.second)+(k-a.first.second)*(k-a.first.second)));
}

void AStarSearch()
{
  int i,j,k;
  i=src.first.first;
  j=src.first.second;
  k=0;

  matrix[i][j][k].f=0.0;
  matrix[i][j][k].g=0.0;
  matrix[i][j][k].h=0.0;
  matrix[i][j][k].parent.first=i;
  matrix[i][j][k].parent.second=j;

  openList.insert(make_pair(0.0,matrix[i][j][k].parent));

  while(openList.empty()==false)
  {
    fpair p=*openList.begin();
    openList.erase(openList.begin());
    i=p.second.first.first;
    j=p.second.first.second;
    k=p.second.second;
//    printf("Current node f  :(%d,%d)\n",i,j);
    closedList[i][j][k]=true;

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

void sucessor(int state,int i,int j,int k,float cost,int ii,int jj,int kk)
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
       if(state%2==0)
       g=cost+1;
       else
       g=cost+sqrt(2);

       h=heuristic(i,j,k,des);
       f=g+h;

       if(matrix[i][j].f==FLT_MAX||matrix[i][j].f>f)
       {
         openList.insert(make_pair(f,make_pair(i,j)));
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

void tracePath(int i,int j,int k)
{
  printf("Current node  :(%d,%d,%d)\n",i,j,k);
  img.at<Vec3b>(i,j)[0]=255;
  triplet t=make_pair(i,j,k);
  makebot(i,j,k);
  if(t==matrix[i][j][k].parent)
  {
    return;
  }
  if(matrix[i][j][k].state==0)
  return;
  else
  tracePath(matrix[i][j][k].parent.first.first,matrix[i][j][k].parent.first.second,matrix[i][j][k].parent.second);

}

void minkowski(int i,int j,int k)
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

void makebot(int i, int j,int k)
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
