#ifndef CATACAMERA_H
#define CATACAMERA_H

#include <opencv2/core/core.hpp>
#include <string>

namespace camodocal
{

class CataCamera
{
public:
    class Parameters
    {
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double xi, double k1, double k2, double p1, double p2,
                   double gamma1, double gamma2, double u0, double v0);

        std::string& cameraName(void);
        int& imageWidth(void);
        int& imageHeight(void);
        double& xi(void);
        double& k1(void);
        double& k2(void);
        double& p1(void);
        double& p2(void);
        double& gamma1(void);
        double& gamma2(void);
        double& u0(void);
        double& v0(void);

        const std::string& cameraName(void) const;
        int imageWidth(void) const;
        int imageHeight(void) const;
        double xi(void) const;
        double k1(void) const;
        double k2(void) const;
        double p1(void) const;
        double p2(void) const;
        double gamma1(void) const;
        double gamma2(void) const;
        double u0(void) const;
        double v0(void) const;

        bool read(const std::string& filename);
        void write(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, Parameters& params);

    private:
        std::string m_cameraName;
        int m_imageWidth;
        int m_imageHeight;
        double m_xi;
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_gamma1;
        double m_gamma2;
        double m_u0;
        double m_v0;
    };

    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const std::string& cameraName,
               int imageWidth, int imageHeight,
               double xi, double k1, double k2, double p1, double p2,
               double gamma1, double gamma2, double u0, double v0);
    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const Parameters& params);

    // Lift points from the image plane to the sphere
    void liftSphere(double u, double v, double *X, double *Y, double *Z) const;
    //%output X
    //%output Y
    //%output Z

    // Lift points from the image plane to the projective space
    void liftProjective(double u, double v, double *X, double *Y, double *Z) const;
    //%output X
    //%output Y
    //%output Z

    // Projects 3D points to the image plane (Pi function)
    void space2plane(double x, double y, double z, double *u, double *v) const;
    //%output u
    //%output v
    
    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void space2plane(double x, double y, double z,
                     double *u, double *v,
                     double *dudx, double *dvdx,
                     double *dudy, double *dvdy,
                     double *dudz, double *dvdz) const;
    //%output u
    //%output v
    //%output dudx
    //%output dvdx
    //%output dudy
    //%output dvdy
    //%output dudz
    //%output dvdz


    void undist2plane(double mx_u, double my_u, double *u, double *v) const;
    //%output u
    //%output v

    void reprojectionDist(double x1, double y1, double z1, double x2, double y2, double z2, double *dist) const;
    //%output dist

    void distortion(double mx_u, double my_u, double *mx_d, double *my_d) const;
    void distortion(double mx_u, double my_u, double *mx_d, double *my_d,
                    double *dxdmx, double *dydmx,
                    double *dxdmy, double *dydmy) const;

    void initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                 float fx = 300.0f, float fy = 300.0f,
                                 cv::Size imageSize = cv::Size(0, 0),
                                 float cx = -1.0f, float cy = -1.0f,
                                 cv::Mat rvec = cv::Mat::eye(3, 3, CV_32F)) const;

    Parameters& parameters(void);
    const Parameters& parameters(void) const;

    ~CataCamera();

 private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

}

#endif
