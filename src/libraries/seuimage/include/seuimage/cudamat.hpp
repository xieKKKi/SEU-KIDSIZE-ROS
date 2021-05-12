#ifndef __CUDAMAT_H
#define __CUDAMAT_H

#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>

namespace seuimage
{
    template<typename T>
    class CudaMat
    {
    public:
        CudaMat(): m_h(0), m_w(0), m_c(0), m_data(NULL)
        {
        }
        CudaMat(int h, int w, int c)
            : m_h(h), m_w(w), m_c(c)
        {
            cudaMalloc(&m_data, h*w*c*sizeof(T));
        }
        bool create(int h, int w, int c)
        {
            if(m_h==h&&m_w==w&&m_c==c) return true;
            m_h=h; m_w=w; m_c=c;
            cudaError_t err = cudaMalloc(&m_data, h*w*c*sizeof(T));
            if(err!=cudaSuccess) return false;
            return true;
        }
        bool copyTo(CudaMat<T> &mat)
        {
            if(mat.data()==NULL) return false;
            if(mat.width()*mat.height()*mat.channels()<m_w*m_h*m_c)
                return false;
            cudaError_t err = cudaMemcpy(mat.data(), m_data, m_w*m_h*m_c*sizeof(T), cudaMemcpyDeviceToDevice);
            if(err!=cudaSuccess) return false;
            return true;
        }
        bool upload(cv::Mat &src)
        {
            if(src.data==NULL) return false;
            cudaError_t err = cudaMemcpy(m_data, src.data, m_h*m_w*m_c*sizeof(T), cudaMemcpyHostToDevice);
            if(err!=cudaSuccess) return false;
            return true;
        }
        bool download(cv::Mat &mat)
        {
            if(mat.empty()) return false;
            cudaError_t err = cudaMemcpy(mat.data, m_data,
                    mat.rows*mat.cols*mat.channels()*mat.elemSize1(), cudaMemcpyDeviceToHost);
            if(err!=cudaSuccess) return false;
            return true;
        }
        ~CudaMat()
        {
            //std::cout<<"CudaMat: cuda memory released"<<std::endl;
            cudaFree(m_data);
        }

        int width(){ return m_w; }
        int height() { return m_h; }
        int channels() { return m_c; }
        int elementSize() { return sizeof(T); };
        
        T* data()
        {
            return m_data;
        }
    private:
        int m_h, m_w, m_c;
        T* m_data;
    };

    typedef CudaMat<unsigned char> CudaMatC;
    typedef CudaMat<float> CudaMatF;
}

#endif // CUDAMAT_H