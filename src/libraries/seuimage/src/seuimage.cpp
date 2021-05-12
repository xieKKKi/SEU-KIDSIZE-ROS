#include <seuimage/seuimage.hpp>
#include <cuda_runtime.h>
#include <cuda.h>
#include <npp.h>

namespace seuimage
{
    bool YUV422ToRGB(CudaMatC &yuv422, CudaMatC &rgb)
    {
        int w=yuv422.width(), h=yuv422.height();
        NppiSize size = {w, h};
        Npp8u *src = (Npp8u*)(yuv422.data());
        Npp8u *dst = (Npp8u*)(rgb.data());
        NppStatus status = nppiYUV422ToRGB_8u_C2C3R(src, w*yuv422.channels(),
                    dst, w*rgb.channels(), size);
        if(status != NPP_SUCCESS) return false;
        return true;
    }

    bool BayerToRGB(CudaMatC &bayer, CudaMatC &rgb)
    {
        int iw=bayer.width(), ih=bayer.height();
        int ow=rgb.width(), oh=rgb.height();
        NppiSize iSize = {iw, ih}, oSize={ow, oh};
        NppiRect iRoi = {0, 0, iw, ih}, oRoi = {0, 0, ow, oh};

        Npp8u *src = (Npp8u*)(bayer.data());
        Npp8u *dst = (Npp8u*)(rgb.data());
        NppStatus status = nppiCFAToRGB_8u_C1C3R(src, iw*bayer.channels(), iSize, iRoi, 
            dst, ow*rgb.channels(), NPPI_BAYER_GRBG, NPPI_INTER_UNDEFINED);
        if(status != NPP_SUCCESS) return false;
        return true;
    }

    bool RGBToHSV(CudaMatC &rgb, CudaMatC &hsv)
    {
        int w = rgb.width(), h=rgb.height();
        NppiSize size = {w, h};

        Npp8u *src = (Npp8u*)(rgb.data());
        Npp8u *dst = (Npp8u*)(hsv.data());
        NppStatus status = nppiRGBToHSV_8u_C3R(src, w*rgb.channels(),
                    dst, w*rgb.channels(), size);
        if(status != NPP_SUCCESS) return false;
        return true;
    }

    bool Resize(CudaMatC &mSrc, CudaMatC &mDst)
    {
        int iw=mSrc.width(), ih=mSrc.height();
        int ow=mDst.width(), oh=mDst.height();
        NppiSize iSize = {iw, ih}, oSize={ow, oh};
        NppiRect iRoi = {0, 0, iw, ih}, oRoi = {0, 0, ow, oh};
        Npp8u *src = (Npp8u*)(mSrc.data());
        Npp8u *dst = (Npp8u*)(mDst.data());
        NppStatus status = nppiResize_8u_C3R(src, iw*mSrc.channels(), iSize, iRoi,
                                        dst, ow*mDst.channels(), oSize, oRoi, NPPI_INTER_LINEAR);
        if(status != NPP_SUCCESS) return false;
        return true;
    }

    bool RGB8uTo32fNorm(CudaMatC &mSrc, CudaMatF &mDst)
    {
        int w=mSrc.width(), h=mSrc.height();
        NppiSize size = {w, h};
        Npp8u *src = (Npp8u*)(mSrc.data());
        Npp32f *dst = (Npp32f*)(mDst.data());
        NppStatus status = nppiScale_8u32f_C3R(src, w*mSrc.channels(),
                            dst, w*mDst.channels()*sizeof(float),
                            size, 0.0f, 1.0f);
        if(status != NPP_SUCCESS) return false;
        return true;
    }

    bool PackedToPlanar(CudaMatF &packed, CudaMatF &planar)
    {
        int w=packed.width(), h=packed.height();
        Npp32f *src = (Npp32f*)(packed.data());
        Npp32f *dst = (Npp32f*)(planar.data());
        NppiSize size = {w, h};
        Npp32f *adst[3];
        adst[0] = dst;
        adst[1] = dst+w*h;
        adst[2] = dst+2*w*h;
        NppStatus status = nppiCopy_32f_C3P3R(src, w*packed.channels()*packed.elementSize(),
                adst, w*packed.elementSize(), size);
        if(status != NPP_SUCCESS) return false;
        return true;
    }

    bool RGBToBGR(CudaMatC &rgb, CudaMatC &bgr)
    {
        int w=rgb.width(), h=rgb.height();
        NppiSize size = {w, h};
        const int order[] = {2, 1, 0};
        Npp8u *src = (Npp8u*)(rgb.data());
        Npp8u *dst = (Npp8u*)(bgr.data());

        NppStatus status = nppiSwapChannels_8u_C3R(src, w*rgb.channels(),
                            dst, w*bgr.channels(), size, order);
        if(status != NPP_SUCCESS) return false;
        return true;
    }
}