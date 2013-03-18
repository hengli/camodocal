#include "CataCamera.h"

#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include <iomanip>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../gpl/gpl.h"

namespace camodocal
{

CataCamera::Parameters::Parameters()
 : m_imageWidth(0)
 , m_imageHeight(0)
 , m_xi(0.0)
 , m_k1(0.0)
 , m_k2(0.0)
 , m_p1(0.0)
 , m_p2(0.0)
 , m_gamma1(0.0)
 , m_gamma2(0.0)
 , m_u0(0.0)
 , m_v0(0.0)
{

}

CataCamera::Parameters::Parameters(const std::string& cameraName,
                                   int w, int h,
                                   double xi,
                                   double k1, double k2,
                                   double p1, double p2,
                                   double gamma1, double gamma2,
                                   double u0, double v0)
 : m_cameraName(cameraName)
 , m_imageWidth(w)
 , m_imageHeight(h)
 , m_xi(xi)
 , m_k1(k1)
 , m_k2(k2)
 , m_p1(p1)
 , m_p2(p2)
 , m_gamma1(gamma1)
 , m_gamma2(gamma2)
 , m_u0(u0)
 , m_v0(v0)
{
}
std::string&
CataCamera::Parameters::cameraName(void)
{
	return m_cameraName;
}

int&
CataCamera::Parameters::imageWidth(void)
{
	return m_imageWidth;
}

int&
CataCamera::Parameters::imageHeight(void)
{
	return m_imageHeight;
}

double&
CataCamera::Parameters::xi(void)
{
	return m_xi;
}

double&
CataCamera::Parameters::k1(void)
{
	return m_k1;
}

double&
CataCamera::Parameters::k2(void)
{
	return m_k2;
}

double&
CataCamera::Parameters::p1(void)
{
	return m_p1;
}

double&
CataCamera::Parameters::p2(void)
{
	return m_p2;
}

double&
CataCamera::Parameters::gamma1(void)
{
	return m_gamma1;
}

double&
CataCamera::Parameters::gamma2(void)
{
	return m_gamma2;
}

double&
CataCamera::Parameters::u0(void)
{
	return m_u0;
}

double&
CataCamera::Parameters::v0(void)
{
	return m_v0;
}

const std::string&
CataCamera::Parameters::cameraName(void) const
{
	return m_cameraName;
}

int
CataCamera::Parameters::imageWidth(void) const
{
	return m_imageWidth;
}

int
CataCamera::Parameters::imageHeight(void) const
{
	return m_imageHeight;
}

double
CataCamera::Parameters::xi(void) const
{
	return m_xi;
}

double
CataCamera::Parameters::k1(void) const
{
	return m_k1;
}

double
CataCamera::Parameters::k2(void) const
{
	return m_k2;
}

double
CataCamera::Parameters::p1(void) const
{
	return m_p1;
}

double
CataCamera::Parameters::p2(void) const
{
	return m_p2;
}

double
CataCamera::Parameters::gamma1(void) const
{
	return m_gamma1;
}

double
CataCamera::Parameters::gamma2(void) const
{
	return m_gamma2;
}

double
CataCamera::Parameters::u0(void) const
{
	return m_u0;
}

double
CataCamera::Parameters::v0(void) const
{
	return m_v0;
}

bool
CataCamera::Parameters::read(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened())
	{
		return false;
	}

	m_imageWidth = static_cast<int>(fs["image_width"]);
	m_imageHeight = static_cast<int>(fs["image_height"]);
	fs["camera_name"] >> m_cameraName;

	cv::FileNode n = fs["mirror_parameters"];
	m_xi = static_cast<double>(n["xi"]);

	n = fs["distortion_parameters"];
	m_k1 = static_cast<double>(n["k1"]);
	m_k2 = static_cast<double>(n["k2"]);
	m_p1 = static_cast<double>(n["p1"]);
	m_p2 = static_cast<double>(n["p2"]);

	n = fs["projection_parameters"];
	m_gamma1 = static_cast<double>(n["gamma1"]);
	m_gamma2 = static_cast<double>(n["gamma2"]);
	m_u0 = static_cast<double>(n["u0"]);
	m_v0 = static_cast<double>(n["v0"]);

	return true;
}

void
CataCamera::Parameters::write(const std::string& filename) const
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	fs << "image_width" << m_imageWidth;
	fs << "image_height" << m_imageHeight;
	fs << "camera_name" << m_cameraName;

	// mirror: xi
	fs << "mirror_parameters";
	fs << "{" << "xi" << m_xi << "}";

	// radial distortion: k1, k2
	// tangential distortion: p1, p2
	fs << "distortion_parameters";
	fs << "{" << "k1" << m_k1
			  << "k2" << m_k2
			  << "p1" << m_p1
			  << "p2" << m_p2 << "}";

	// projection: xi, gamma1, gamma2, u0, v0
	fs << "projection_parameters";
	fs << "{" << "gamma1" << m_gamma1
			  << "gamma2" << m_gamma2
			  << "u0" << m_u0
			  << "v0" << m_v0 << "}";

	fs.release();
}

CataCamera::Parameters&
CataCamera::Parameters::operator=(const CataCamera::Parameters& other)
{
    if (this != &other)
    {
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_xi = other.m_xi;
        m_k1 = other.m_k1;
        m_k2 = other.m_k2;
        m_p1 = other.m_p1;
        m_p2 = other.m_p2;
        m_gamma1 = other.m_gamma1;
        m_gamma2 = other.m_gamma2;
        m_u0 = other.m_u0;
        m_v0 = other.m_v0;
    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, CataCamera::Parameters& params)
{
    out << "Camera Parameters" << std::endl;
    out << "   width " << params.m_imageWidth << std::endl;
    out << "  height " << params.m_imageHeight << std::endl;
    out << "    name " << params.m_cameraName << std::endl;

	out << "Mirror Parameters" << std::endl;
	out << std::fixed << std::setprecision(10);
	out << "      xi " << params.m_xi << std::endl;

	// radial distortion: k1, k2
	// tangential distortion: p1, p2
	out << "Distortion Parameters" << std::endl;
	out << "      k1 " << params.m_k1 << std::endl
		<< "      k2 " << params.m_k2 << std::endl
		<< "      p1 " << params.m_p1 << std::endl
		<< "      p2 " << params.m_p2 << std::endl;

	// projection: xi, gamma1, gamma2, u0, v0
	out << "Projection Parameters" << std::endl;
	out << "  gamma1 " << params.m_gamma1 << std::endl
		<< "  gamma2 " << params.m_gamma2 << std::endl
		<< "      u0 " << params.m_u0 << std::endl
		<< "      v0 " << params.m_v0 << std::endl;

	return out;
}

CataCamera::CataCamera(const std::string& cameraName,
                       int imageWidth, int imageHeight,
                       double xi, double k1, double k2, double p1, double p2,
                       double gamma1, double gamma2, double u0, double v0)
 : mParameters(cameraName, imageWidth, imageHeight,
               xi, k1, k2, p1, p2, gamma1, gamma2, u0, v0)
{
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.gamma1();
    m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
    m_inv_K22 = 1.0 / mParameters.gamma2();
    m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
}

CataCamera::CataCamera(const CataCamera::Parameters& params)
 : mParameters(params)
{
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.gamma1();
    m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
    m_inv_K22 = 1.0 / mParameters.gamma2();
    m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
}


CataCamera::~CataCamera()
{

}

/** 
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param u u image coordinate
 * \param v v image coordinate
 * \param X X coordinate of the point on the sphere
 * \param Y Y coordinate of the point on the sphere
 * \param Z Z coordinate of the point on the sphere
 */
void
CataCamera::liftSphere(double u, double v, double *X, double *Y, double *Z) const
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    double lambda;

    // Lift points to normalised plane
    // Matlab points start at 1 (calibration)
    mx_d = m_inv_K11*(u) + m_inv_K13;
    my_d = m_inv_K22*(v) + m_inv_K23;

    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
        // Apply inverse distortion model
        if (0)
        {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();

            // Inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }
        else
        {
            // Recursive distortion model
            int n = 6;
            double dx_u, dy_u;
            distortion(mx_d,my_d,&dx_u,&dy_u);
            // Approximate value
            mx_u = mx_d-dx_u;
            my_u = my_d-dy_u;

            for (int i=1;i<n;i++)
            {
                distortion(mx_u,my_u,&dx_u,&dy_u);
                mx_u = mx_d-dx_u;
                my_u = my_d-dy_u;
            }
        }
    }

    // Lift normalised points to the sphere (inv_hslash)
    double xi = mParameters.xi();
    if (xi==1)
    {
        lambda = 2/(mx_u*mx_u+my_u*my_u+1);
        *X = lambda*mx_u;
        *Y = lambda*my_u;
        *Z = lambda-1;
    }
    else
    {
        lambda = (xi+sqrt(1+(1-xi*xi)*(mx_u*mx_u+my_u*my_u)))/(1+mx_u*mx_u+my_u*my_u);
        *X = lambda*mx_u;
        *Y = lambda*my_u;
        *Z = lambda-xi;
    }
}

/** 
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param u u image coordinate
 * \param v v image coordinate
 * \param X X coordinate of the projective ray
 * \param Y Y coordinate of the projective ray
 * \param Z Z coordinate of the projective ray
 */
void
CataCamera::liftProjective(double u, double v, double *X, double *Y, double *Z) const
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    double lambda;

    // Lift points to normalised plane
    // Matlab points start at 1 (calibration)
    mx_d = m_inv_K11*(u) + m_inv_K13;
    my_d = m_inv_K22*(v) + m_inv_K23;

    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
        if (0)
        {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();

            // Apply inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }
        else
        {
            // Recursive distortion model
            int n = 8;
            double dx_u, dy_u;
            distortion(mx_d,my_d,&dx_u,&dy_u);
            // Approximate value
            mx_u = mx_d-dx_u;
            my_u = my_d-dy_u;

            for (int i=1;i<n;i++)
            {
                distortion(mx_u,my_u,&dx_u,&dy_u);
                mx_u = mx_d-dx_u;
                my_u = my_d-dy_u;
            }
        }
    }

    // Obtain a projective ray
    double xi = mParameters.xi();
    if (xi == 1.0)
    {
        *X = mx_u;
        *Y = my_u;
        *Z = (1-mx_u*mx_u-my_u*my_u)/2;
    }
    else
    {
        // Reuse variable
        rho2_d = mx_u*mx_u+my_u*my_u;
        *X = mx_u;
        *Y = my_u;
        *Z = 1-xi*(rho2_d+1)/(xi+sqrt(1+(1-xi*xi)*rho2_d));
    }
}


/** 
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param x 3D point x coordinate
 * \param y 3D point y coordinate
 * \param z 3D point z coordinate
 * \param u return value, contains the image point u coordinate
 * \param v return value, contains the image point v coordinate
 */
void
CataCamera::space2plane(double x, double y, double z, double *u, double *v) const
{
    double mx_u, my_u, mx_d, my_d;

    // Project points to the normalised plane
    z = z + mParameters.xi() * sqrt(x*x+y*y+z*z);
    mx_u = x/z;
    my_u = y/z;

    if (m_noDistortion)
    {
        mx_d = mx_u;
        my_d = my_u;
    }
    else
    {
        // Apply distortion
        double dx_u, dy_u;
        distortion(mx_u,my_u,&dx_u,&dy_u);
        mx_d = mx_u+dx_u;
        my_d = my_u+dy_u;
    }

    // Apply generalised projection matrix
    // Matlab points start at 1
    *u = mParameters.gamma1() * mx_d + mParameters.u0();
    *v = mParameters.gamma2() * my_d + mParameters.v0();
}


/** 
 * \brief Project a 3D points (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *        and calculate jacobian
 *
 * \param x 3D point x coordinate
 * \param y 3D point y coordinate
 * \param z 3D point z coordinate
 * \param u return value, contains the image point u coordinate
 * \param v return value, contains the image point v coordinate
 */
void
CataCamera::space2plane(double x, double y, double z,
                        double *u, double *v,
                        double *dudx, double *dvdx,
                        double *dudy, double *dvdy,
                        double *dudz, double *dvdz) const
{
    double xi = mParameters.xi();

    double mx_u, my_u, mx_d, my_d;
    double norm,inv_denom;
    double dxdmx, dydmx, dxdmy, dydmy;

    norm = sqrt(x*x+y*y+z*z);
    // Project points to the normalised plane
    inv_denom = 1/(z+xi*norm);
    mx_u = inv_denom*x;
    my_u = inv_denom*y;

    // Calculate jacobian
    inv_denom = inv_denom*inv_denom/norm;
    *dudx = inv_denom*(norm*z+xi*(y*y+z*z));
    *dvdx = -inv_denom*xi*x*y;
    *dudy = *dvdx;
    *dvdy = inv_denom*(norm*z+xi*(x*x+z*z));
    inv_denom = inv_denom*(-xi*z-norm); // reuse variable
    *dudz = x*inv_denom;
    *dvdz = y*inv_denom;

    if (m_noDistortion)
    {
        mx_d = mx_u;
        my_d = my_u;
    }
    else
    {
        // Apply distortion
        double dx_u, dy_u;
        distortion(mx_u,my_u,&dx_u,&dy_u,&dxdmx,&dydmx,&dxdmy,&dydmy);
        mx_d = mx_u+dx_u;
        my_d = my_u+dy_u;
    }

    double gamma1 = mParameters.gamma1();
    double gamma2 = mParameters.gamma2();

    // Make the product of the jacobians
    // and add projection matrix jacobian
    inv_denom = gamma1*(*dudx*dxdmx+*dvdx*dxdmy); // reuse
    *dvdx = gamma2*(*dudx*dydmx+*dvdx*dydmy);
    *dudx = inv_denom;

    inv_denom = gamma1*(*dudy*dxdmx+*dvdy*dxdmy); // reuse
    *dvdy = gamma2*(*dudy*dydmx+*dvdy*dydmy);
    *dudy = inv_denom;

    inv_denom = gamma1*(*dudz*dxdmx+*dvdz*dxdmy); // reuse
    *dvdz = gamma2*(*dudz*dydmx+*dvdz*dydmy);
    *dudz = inv_denom;
    
    // Apply generalised projection matrix
    // Matlab points start at 1
    *u = gamma1 * mx_d + mParameters.u0();
    *v = gamma2 * my_d + mParameters.v0();
}

/** 
 * \brief Projects an undistorted 2D point (\a mx_u,\a my_u) to the image plane in (\a u,\a v)
 *
 * \param mx_u 2D point x coordinate
 * \param my_u 3D point y coordinate
 * \param u return value, contains the image point u coordinate
 * \param v return value, contains the image point v coordinate
 */
void
CataCamera::undist2plane(double mx_u, double my_u, double *u, double *v) const
{
    double mx_d, my_d;

    if (m_noDistortion)
    {
        mx_d = mx_u;
        my_d = my_u;
    }
    else
    {
        // Apply distortion
        double dx_u, dy_u;
        distortion(mx_u,my_u,&dx_u,&dy_u);
        mx_d = mx_u + dx_u;
        my_d = my_u + dy_u;
    }

    // Apply generalised projection matrix
    // Matlab points start at 1
    *u = mParameters.gamma1() * mx_d + mParameters.u0();
    *v = mParameters.gamma2() * my_d + mParameters.v0();
}

/** 
 * \brief Calculates the reprojection distance between points
 *
 * \param x1 first 3D point x coordinate
 * \param y1 first 3D point y coordinate
 * \param z1 first 3D point z coordinate
 * \param x2 second 3D point x coordinate
 * \param y2 second 3D point y coordinate
 * \param z2 second 3D point z coordinate
 * \param dist return value, euclidean distance in the plane
 */
void
CataCamera::reprojectionDist(double x1, double y1, double z1, double x2, double y2, double z2, double *dist) const
{
    double u1,v1,u2,v2;

    space2plane(x1,y1,z1,&u1,&v1);
    space2plane(x2,y2,z2,&u2,&v2);

    *dist = sqrt((u1-u2)*(u1-u2)+(v1-v2)*(v1-v2));
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *  
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void
CataCamera::distortion(double mx_u, double my_u,
                       double *dx_u, double *dy_u) const
{
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = mx_u*mx_u;
    my2_u = my_u*my_u;
    mxy_u = mx_u*my_u;
    rho2_u = mx2_u+my2_u;
    rad_dist_u = k1*rho2_u+k2*rho2_u*rho2_u;
    *dx_u = mx_u*rad_dist_u + 2*p1*mxy_u + p2*(rho2_u+2*mx2_u);
    *dy_u = my_u*rad_dist_u + 2*p2*mxy_u + p1*(rho2_u+2*my2_u);
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate jacobian
 *
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void
CataCamera::distortion(double mx_u, double my_u,
                       double *dx_u, double *dy_u,
                       double *dxdmx, double *dydmx,
                       double *dxdmy, double *dydmy) const
{
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = mx_u*mx_u;
    my2_u = my_u*my_u;
    mxy_u = mx_u*my_u;
    rho2_u = mx2_u+my2_u;
    rad_dist_u = k1*rho2_u+k2*rho2_u*rho2_u;
    *dx_u = mx_u*rad_dist_u + 2*p1*mxy_u + p2*(rho2_u+2*mx2_u);
    *dy_u = my_u*rad_dist_u + 2*p2*mxy_u + p1*(rho2_u+2*my2_u);

    *dxdmx = 1 + rad_dist_u + k1*2*mx2_u + k2*rho2_u*4*mx2_u + 2*p1*my_u + 6*p2*mx_u;
    *dydmx = k1*2*mx_u*my_u + k2*4*rho2_u*mx_u*my_u + p1*2*mx_u + 2*p2*my_u;
    *dxdmy = *dydmx;
    *dydmy = 1 + rad_dist_u + k1*2*my2_u + k2*rho2_u*4*my2_u + 6*p1*my_u + 2*p2*mx_u;
}

void
CataCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx, float fy,
                                    cv::Size imageSize,
                                    float cx, float cy,
                                    cv::Mat rmat) const
{
    if (imageSize == cv::Size(0, 0))
    {
        imageSize = cv::Size(mParameters.imageWidth(), mParameters.imageHeight());
    }

    cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

    Eigen::Matrix3f K_rect;

    if (cx == -1.0f && cy == -1.0f)
    {
        K_rect << fx, 0, imageSize.width / 2,
                  0, fy, imageSize.height / 2,
                  0, 0, 1;
    }
    else
    {
        K_rect << fx, 0, cx,
                  0, fy, cy,
                  0, 0, 1;
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse();

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen(rmat, R);
    R_inv = R.inverse();

    // assume no skew
    Eigen::Matrix3f K;
    K << mParameters.gamma1(), 0, mParameters.u0(),
         0, mParameters.gamma2(), mParameters.v0(),
         0, 0, 1;

    for (int v = 0; v < imageSize.height; ++v)
    {
        for (int u = 0; u < imageSize.width; ++u)
        {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            // project world point in mirror frame onto unit sphere
            Eigen::Vector3f Xs = uo / uo.norm();

            // change point to new reference frame centered in C_p
            Xs(2) += mParameters.xi();

            // project point onto normalized plane
            Eigen::Vector3f m = Xs / Xs(2);

            Eigen::Vector2f ui;
            ui << m(0), m(1);

            // undistort point
            float rho_sqr =  square(m(0)) + square(m(1));

            float ratio_radial = mParameters.k1() * rho_sqr + mParameters.k2() * square(rho_sqr);

            Eigen::Vector2f du_radial = ui * ratio_radial;

            Eigen::Vector2f du_tangent;
            du_tangent(0) = 2.0f * mParameters.p1() * ui(0) * ui(1) +
                            mParameters.p2() * (rho_sqr + 2.0f * square(ui(0)));
            du_tangent(1) = mParameters.p1() * (rho_sqr + 2.0f * square(ui(1))) +
                            2.0f * mParameters.p2() * ui(0) * ui(1);

            Eigen::Vector3f ud;
            ud << ui + du_radial + du_tangent, 1.0f;

            Eigen::Vector3f xd = K * ud;

            mapX.at<float>(v,u) = xd(0);
            mapY.at<float>(v,u) = xd(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}

CataCamera::Parameters&
CataCamera::parameters(void)
{
    return mParameters;
}

const CataCamera::Parameters&
CataCamera::parameters(void) const
{
    return mParameters;
}

}
