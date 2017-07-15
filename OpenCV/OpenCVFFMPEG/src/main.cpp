//#include <QApplication>
#include <QtWidgets>
//#include <QtCore/QCoreApplication>
#define __STDC_CONSTANT_MACROS
#define __STDC_LIMIT_MACROS
#define UINT64_C
//#define WinMain@16

#include "fcntl.h"
int f_desw;

extern "C" {
 #include <libavdevice/avdevice.h>
 #include <libswscale/swscale.h>
}

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    avdevice_register_all();
    avcodec_register_all();

    const char  *filenameSrc = "video=Integrated Webcam";

    AVCodecContext  *pCodecCtx;
    AVFormatContext *pFormatCtx = avformat_alloc_context();
    AVCodec * pCodec;
    AVInputFormat *iformat = av_find_input_format("dshow");
    AVFrame *pFrame, *pFrameRGB;

    if(avformat_open_input(&pFormatCtx,filenameSrc,iformat,NULL) != 0) return -12;
    if(av_find_stream_info(pFormatCtx) < 0)   return -13;
    av_dump_format(pFormatCtx, 0, filenameSrc, 0);
    int videoStream = 1;
    for(int i=0; i < pFormatCtx->nb_streams; i++)
    {
        if(pFormatCtx->streams[i]->codec->coder_type==AVMEDIA_TYPE_VIDEO)
        {
            videoStream = i;
            break;
        }
    }

    if(videoStream == -1) return -14;
    pCodecCtx = pFormatCtx->streams[videoStream]->codec;

    pCodec =avcodec_find_decoder(pCodecCtx->codec_id);
    if(pCodec==NULL) return -15; //codec not found

    if(avcodec_open2(pCodecCtx,pCodec,NULL) < 0) return -16;

    pFrame    = avcodec_alloc_frame();
    pFrameRGB = avcodec_alloc_frame();

    uint8_t *buffer;
    int numBytes;

    AVPixelFormat  pFormat = AV_PIX_FMT_BGR24;
    numBytes = avpicture_get_size(pFormat,pCodecCtx->width,pCodecCtx->height) ;
    buffer = (uint8_t *) av_malloc(numBytes*sizeof(uint8_t));
    avpicture_fill((AVPicture *) pFrameRGB,buffer,pFormat,pCodecCtx->width,pCodecCtx->height);

    int res;
    int frameFinished;
    AVPacket packet;
    while(res = av_read_frame(pFormatCtx,&packet)>=0)
    {

        if(packet.stream_index == videoStream){

            avcodec_decode_video2(pCodecCtx,pFrame,&frameFinished,&packet);

            if(frameFinished){

                struct SwsContext * img_convert_ctx;
                img_convert_ctx = sws_getCachedContext(NULL,pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,   pCodecCtx->width, pCodecCtx->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL,NULL);
                sws_scale(img_convert_ctx, ((AVPicture*)pFrame)->data, ((AVPicture*)pFrame)->linesize, 0, pCodecCtx->height, ((AVPicture *)pFrameRGB)->data, ((AVPicture *)pFrameRGB)->linesize);

                //OpenCV
                cv::Mat img(pFrame->height,pFrame->width,CV_8UC3,pFrameRGB->data[0]);
                cv::imshow("display",img);
                cvWaitKey(1);

                av_free_packet(&packet);
                sws_freeContext(img_convert_ctx);

            }

        }

    }

    av_free_packet(&packet);
    close(f_desw);
    avcodec_close(pCodecCtx);
    av_free(pFrame);
    av_free(pFrameRGB);
    avformat_close_input(&pFormatCtx);

    return a.exec();
}
