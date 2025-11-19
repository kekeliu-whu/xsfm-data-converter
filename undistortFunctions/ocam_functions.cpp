/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"

#include <iostream>

//------------------------------------------------------------------------------
int get_ocam_model(struct ocam_model &myocam_model, const char *filename)
{
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;
 
 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);				  
   exit(-1);
 }
 
 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", &(myocam_model.length_pol));
 for (i = 0; i < myocam_model.length_pol; i++)
 {
     fscanf(f," %lf",&(myocam_model.pol[i]));
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", &(myocam_model.length_invpol));
 for (i = 0; i < myocam_model.length_invpol; i++)
 {
     fscanf(f," %lf",&(myocam_model.invpol[i]));
 }
 
 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", &(myocam_model.xc), &(myocam_model.yc));

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", &(myocam_model.c),&(myocam_model.d),&(myocam_model.e));

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", &(myocam_model.height), &(myocam_model.width));

 fclose(f);
 return 0;
}

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2], struct ocam_model &myocam_model)
{
 double xc      = (myocam_model.xc);
 double yc      = (myocam_model.yc); 
 double c       = (myocam_model.c);
 double d       = (myocam_model.d);
 double e       = (myocam_model.e);
 int length_pol = (myocam_model.length_pol); 
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );
  
 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = myocam_model.pol[0];
 double r_i = 1;
 int i;
 
 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*myocam_model.pol[i];
 }
 
 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );
 
 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp; 
 point3D[2] = invnorm*zp;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model &myocam_model)
{
 double xc          = (myocam_model.xc);
 double yc          = (myocam_model.yc); 
 double c           = (myocam_model.c);
 double d           = (myocam_model.d);
 double e           = (myocam_model.e);
 int    width       = (myocam_model.width);
 int    height      = (myocam_model.height);
 int length_invpol  = (myocam_model.length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;
  
  if (norm != 0) 
  {
    invnorm = 1/norm;
    t  = theta;
    rho = myocam_model.invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*myocam_model.invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;
  
    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT( cv::Mat &mapx, cv::Mat &mapy, struct ocam_model &ocam_model, float focal)
{
     int i, j;
     int width = mapx.cols; //New width
     int height = mapx.rows;//New height     
     float *data_mapx = mapx.ptr<float>();
     float *data_mapy = mapy.ptr<float>();
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -focal;
     double M[3];
     double m[2];
     
     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {   
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);
             *( data_mapx + i*width+j ) = (float) m[1];
             *( data_mapy + i*width+j ) = (float) m[0];
         }
}

//------------------------------------------------------------------------------
void create_panoramic_undistortion_LUT ( cv::Mat &mapx, cv::Mat &mapy, float Rmin, float Rmax, float xc, float yc )
{
     int i, j;
     float theta;
     int width = mapx.cols;
     int height = mapx.rows;     
     float *data_mapx = mapx.ptr<float>();
     float *data_mapy = mapy.ptr<float>();
     float rho;
     
     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             theta = -((float)j)/width*2*3.14159265358979323846; // Note, if you would like to flip the image, just inverte the sign of theta
             rho   = Rmax - (Rmax-Rmin)/height*i;
             *( data_mapx + i*width+j ) = yc + rho*sin(theta); //in OpenCV "x" is the
             *( data_mapy + i*width+j ) = xc + rho*cos(theta);             
         }
}

//------------------------------------------------------------------------------
int get_ocam_model_from_xml(struct ocam_model &myocam_model, int camera_id, const char *filename)
{
 tinyxml2::XMLDocument doc;
 if (doc.LoadFile(filename) != tinyxml2::XML_SUCCESS)
 {
   printf("File %s cannot be opened or parsed\n", filename);
   return -1;
 }

 tinyxml2::XMLElement *root = doc.RootElement();
 if (!root)
 {
   printf("No root element in XML\n");
   return -1;
 }

 tinyxml2::XMLElement *cameraHead = root->FirstChildElement("CameraHead");
 if (!cameraHead)
 {
   printf("No CameraHead found\n");
   return -1;
 }

 tinyxml2::XMLElement *cameraModel = cameraHead->FirstChildElement("CameraModel");
 while (cameraModel)
 {
     const char* sensorName = cameraModel->FirstChildElement("SensorName")->GetText();
     if (sensorName && std::string(sensorName) == "cam" + std::to_string(camera_id))
     {
         break;
     }
     cameraModel = cameraModel->NextSiblingElement("CameraModel");
 }

 if (!cameraModel)
 {
   printf("CameraModel with SensorName cam%d not found\n", camera_id);
   return -1;
 }

 tinyxml2::XMLElement *ocamModel = cameraModel->FirstChildElement("OCamModel");
 if (!ocamModel)
 {
   printf("No OCamModel found\n");
   return -1;
 }

 // Read affine coefficients
 ocamModel->FirstChildElement("c")->QueryDoubleText(&(myocam_model.c));
 ocamModel->FirstChildElement("d")->QueryDoubleText(&(myocam_model.d));
 ocamModel->FirstChildElement("e")->QueryDoubleText(&(myocam_model.e));

 // Read center coordinates
 ocamModel->FirstChildElement("cx")->QueryDoubleText(&(myocam_model.xc));
 ocamModel->FirstChildElement("cy")->QueryDoubleText(&(myocam_model.yc));

 // Read polynomial coefficients (cam2world)
 tinyxml2::XMLElement *cam2world = ocamModel->FirstChildElement("cam2world");
 if (cam2world)
 {
   int i = 0;
   for (tinyxml2::XMLElement *coeff = cam2world->FirstChildElement("coeff"); coeff; coeff = coeff->NextSiblingElement("coeff"))
   {
     if (i < MAX_POL_LENGTH)
     {
       coeff->QueryDoubleText(&(myocam_model.pol[i]));
       i++;
     }
   }
   myocam_model.length_pol = i;
 }

 // Read inverse polynomial coefficients (world2cam)
 tinyxml2::XMLElement *world2cam = ocamModel->FirstChildElement("world2cam");
 if (world2cam)
 {
   int i = 0;
   for (tinyxml2::XMLElement *coeff = world2cam->FirstChildElement("coeff"); coeff; coeff = coeff->NextSiblingElement("coeff"))
   {
     if (i < MAX_POL_LENGTH)
     {
       coeff->QueryDoubleText(&(myocam_model.invpol[i]));
       i++;
     }
   }
   myocam_model.length_invpol = i;
 }

 // Read image size
 tinyxml2::XMLElement *imageSize = cameraModel->FirstChildElement("ImageSize");
 if (imageSize)
 {
   imageSize->FirstChildElement("Width")->QueryIntText(&(myocam_model.width));
   imageSize->FirstChildElement("Height")->QueryIntText(&(myocam_model.height));
 }

 return 0;
}
