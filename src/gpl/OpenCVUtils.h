#ifndef OPENCVUTILS_H
#define OPENCVUTILS_H

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#ifndef HAVE_OPENCV3
#include <opencv2/core/internal.hpp>
#endif // HAVE_OPENCV3


/// @todo remove this dypedef and replace with more modern cast
#ifndef CV_CAST_8U
#define CV_CAST_8U(t) (uchar)(!((t) & ~255) ? (t) : (t) > 0 ? 255 : 0)
#endif // CV_CAST_8U

void cvEqualizeHist( const CvArr* srcarr, CvArr* dstarr, CvMat* mask )
{
    using namespace cv;

    CvMat sstub, *src = cvGetMat(srcarr, &sstub);
    CvMat dstub, *dst = cvGetMat(dstarr, &dstub);

    CV_Assert( CV_ARE_SIZES_EQ(src, dst) && CV_ARE_TYPES_EQ(src, dst) &&
               CV_MAT_TYPE(src->type) == CV_8UC1 );

    CV_Assert( CV_ARE_SIZES_EQ(src, mask) && CV_MAT_TYPE(mask->type) == CV_8UC1); 

#ifdef HAVE_OPENCV3
    CvSize size = cvGetSize(src);
#else // HAVE_OPENCV3
    CvSize size = cvGetMatSize(src);
#endif // HAVE_OPENCV3
    if( CV_IS_MAT_CONT(src->type & dst->type) )
    {
        size.width *= size.height;
        size.height = 1;
    }
    int x, y;
    const int hist_sz = 256;
    int hist[hist_sz];
    memset(hist, 0, sizeof(hist));

    for( y = 0; y < size.height; y++ )
    {
        const uchar* sptr = src->data.ptr + src->step*y;
        const uchar* mptr = mask->data.ptr + mask->step*y; 
        for( x = 0; x < size.width; x++ )
            if (mptr[x]) hist[sptr[x]]++;
    }

    float scale = 255.f/(cvCountNonZero(mask));
    int sum = 0;
    uchar lut[hist_sz+1];

    for( int i = 0; i < hist_sz; i++ )
    {
        sum += hist[i];
        int val = cvRound(sum*scale);
        lut[i] = CV_CAST_8U(val);
    }

    lut[0] = 0;
    cvSetZero(dst); 
    for( y = 0; y < size.height; y++ )
    {
        const uchar* sptr = src->data.ptr + src->step*y;
        const uchar* mptr = mask->data.ptr + mask->step*y; 
        uchar* dptr = dst->data.ptr + dst->step*y;
        for( x = 0; x < size.width; x++ )
            if (mptr[x]) dptr[x] = lut[sptr[x]];
    }
}

void equalizeHist(cv::InputArray _src, cv::OutputArray _dst, cv::InputArray _mask = cv::noArray())
{
    using namespace cv;

    Mat src = _src.getMat().clone();
    _dst.create( src.size(), src.type() );
    Mat dst = _dst.getMat();
    Mat mask; 
    if (_mask.empty()) mask = Mat::ones(src.size(), CV_8UC1); 
    else mask = _mask.getMat(); 
    CvMat _csrc = src, _cdst = dst, _cmask = mask;
    cvEqualizeHist( &_csrc, &_cdst, &_cmask);
}

#endif
