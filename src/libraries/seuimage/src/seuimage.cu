#include <seuimage/seuimage.hpp>

#define BLOCKX 16
#define BLOCKY 16

__device__ unsigned char rgb_bound(int v)
{
  return v > 255 ? 255 : (v < 0 ? 0 : v);
}

__global__ void baygr2bgr_kernal(uint8_t *bayergr, uint8_t *bgr, int w, int h, 
    float rgain, float ggain, float bgain)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int outy = h - y;
  int outx = w - x;
  float r, g, b;

  b = bayergr[(y + ((y + 1) & 1)) * w + x - (x & 1)] * bgain;
  g = bayergr[y * w + x - (x & 1) + (y & 1)] * ggain;
  r = bayergr[(y - (y & 1)) * w + x + ((x + 1) & 1)] * rgain;

  bgr[outy * w * 3 + outx * 3 + 0] = rgb_bound(r);
  bgr[outy * w * 3 + outx * 3 + 1] = rgb_bound(g);
  bgr[outy * w * 3 + outx * 3 + 2] = rgb_bound(b);
}

__global__ void white_balance_kernal(unsigned char *rgb, float rgain, float ggain, float bgain)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int w = gridDim.x * blockDim.x;

  int offset = y * w * 3 + x * 3;
  float rf, gf, bf;
  rf = rgb[offset + 0] * rgain;
  gf = rgb[offset + 1] * ggain;
  bf = rgb[offset + 2] * bgain;
  rgb[offset + 0] = rgb_bound(rf);
  rgb[offset + 1] = rgb_bound(gf);
  rgb[offset + 2] = rgb_bound(bf);
}

__global__ void label_color_kernal(unsigned char *rgb, unsigned char *hsv, int h_l, int h_h, int s_l, int s_h, int v_l,
                                   int v_h)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  int w = gridDim.x * blockDim.x;

  int offset = y * w * 3 + x * 3;
  unsigned char h = hsv[offset + 0];
  unsigned char s = hsv[offset + 1];
  unsigned char v = hsv[offset + 2];
  if (h >= h_l && h <= h_h && s >= s_l && s <= s_h && v >= v_l && v <= v_h)
  {
    rgb[offset + 0] = 0;
    rgb[offset + 1] = 255;
    rgb[offset + 2] = 0;
  }
}

__global__ void build_map_kernal(float *pCamK, float *pDistort, float *pInvNewCamK, float *pMapx, float *pMapy,
                                 int outImgW, int outImgH)
{
  const int tidx = blockDim.x * blockIdx.x + threadIdx.x;
  const int tidy = blockDim.y * blockIdx.y + threadIdx.y;
  if (tidx < outImgW && tidy < outImgH)
  {
    float k1 = pDistort[0];
    float k2 = pDistort[1];
    float p1 = pDistort[2];
    float p2 = pDistort[3];
    float k3, k4, k5, k6, s1, s2, s3, s4;
    k3 = k4 = k5 = k6 = s1 = s2 = s3 = s4 = 0;
    float fx = pCamK[0];
    float fy = pCamK[4];
    float u0 = pCamK[2];
    float v0 = pCamK[5];

    float _x = tidx * pInvNewCamK[0] + tidy * pInvNewCamK[1] + pInvNewCamK[2];
    float _y = tidx * pInvNewCamK[3] + tidy * pInvNewCamK[4] + pInvNewCamK[5];
    float _w = tidx * pInvNewCamK[6] + tidy * pInvNewCamK[7] + pInvNewCamK[8];

    float w = 1. / _w;
    float x = _x * w;
    float y = _y * w;

    float x2 = x * x;
    float y2 = y * y;
    float r2 = x2 + y2;
    float _2xy = 2 * x * y;
    float kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
    float xd = (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2) + s1 * r2 + s2 * r2 * r2);
    float yd = (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy + s3 * r2 + s4 * r2 * r2);

    float invProj = 1.;
    float u = fx * invProj * xd + u0;
    float v = fy * invProj * yd + v0;

    int mapIdx = tidy * outImgW + tidx;
    pMapx[mapIdx] = (float)u;
    pMapy[mapIdx] = (float)v;
  }
}

__global__ void remap_kernal(unsigned char *pSrcImg, unsigned char *pDstImg, float *pMapx, float *pMapy, int inWidth,
                             int inHeight, int outWidth, int outHeight, int channels)
{
  const int tidx = blockDim.x * blockIdx.x + threadIdx.x;
  const int tidy = blockDim.y * blockIdx.y + threadIdx.y;
  if (tidx < outWidth && tidy < outHeight)
  {
    int mapIdx = tidy * outWidth + tidx;
    float u = pMapx[mapIdx];
    float v = pMapy[mapIdx];

    int u1 = floor(u);
    int v1 = floor(v);
    int u2 = u1 + 1;
    int v2 = v1 + 1;
    if (u1 >= 0 && v1 >= 0 && u2 < inWidth && v2 < inHeight)
    {
      float dx = u - u1;
      float dy = v - v1;
      float weight1 = (1 - dx) * (1 - dy);
      float weight2 = dx * (1 - dy);
      float weight3 = (1 - dx) * dy;
      float weight4 = dx * dy;

      int resultIdx = mapIdx * 3;
      for (int chan = 0; chan < channels; chan++)
      {
        pDstImg[resultIdx + chan] = (unsigned char)(weight1 * pSrcImg[(v1 * inWidth + u1) * 3 + chan] +
                                                    weight2 * pSrcImg[(v1 * inWidth + u2) * 3 + chan] +
                                                    weight3 * pSrcImg[(v2 * inWidth + u1) * 3 + chan] +
                                                    weight4 * pSrcImg[(v2 * inWidth + u2) * 3 + chan]);
      }
    }
  }
}

namespace seuimage
{
bool WhiteBalance(CudaMatC &rgb, float rgain, float ggain, float bgain)
{
  int w = rgb.width(), h = rgb.height();
  dim3 block(BLOCKX, BLOCKY);
  dim3 grid(w / BLOCKX, h / BLOCKY);
  white_balance_kernal<<<grid, block>>>(rgb.data(), rgain, ggain, bgain);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess)
    return false;
  return true;
}

bool Undistored(CudaMatC &in, CudaMatC &out, float *pCamK, float *pDistort, float *pInvNewCamK, float *pMapx,
                float *pMapy)
{
  int w = in.width(), h = in.height(), c = in.channels();
  dim3 block(BLOCKX, BLOCKY);
  dim3 grid((w + block.x - 1) / block.x, (h + block.y - 1) / block.y);
  build_map_kernal<<<grid, block>>>(pCamK, pDistort, pInvNewCamK, pMapx, pMapy, w, h);
  cudaThreadSynchronize();
  remap_kernal<<<grid, block>>>(in.data(), out.data(), pMapx, pMapy, w, h, w, h, c);
  cudaThreadSynchronize();
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess)
    return false;
  return true;
}

bool LabelColor(CudaMatC &rgb, CudaMatC &hsv, const std::vector<cv::Point> &HSV)
{
  int h_l = HSV[0].x, h_h = HSV[0].y;
  int s_l = HSV[1].x, s_h = HSV[1].y;
  int v_l = HSV[2].x, v_h = HSV[2].y;

  int w = rgb.width(), h = rgb.height();
  dim3 block(BLOCKX, BLOCKY);
  dim3 grid(w / BLOCKX, h / BLOCKY);
  label_color_kernal<<<grid, block>>>(rgb.data(), hsv.data(), h_l, h_h, s_l, s_h, v_l, v_h);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess)
    return false;
  return true;
}

bool CudaBayerToRGB(CudaMatC &bayer, CudaMatC &rgb, float rgain, float ggain, float bgain)
{
    int wi = bayer.width(), hi = bayer.height();
    int wo = rgb.width(), ho = bayer.height();
    if(wi!=wo || hi!=ho) return false;
    dim3 block(BLOCKX, BLOCKY);
    dim3 grid(wi / BLOCKX, hi / BLOCKY);
    baygr2bgr_kernal<<<grid, block>>>(bayer.data(), rgb.data(), wi, hi, rgain, ggain, bgain);
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
        return false;
    return true;
}
}  // namespace seuimage