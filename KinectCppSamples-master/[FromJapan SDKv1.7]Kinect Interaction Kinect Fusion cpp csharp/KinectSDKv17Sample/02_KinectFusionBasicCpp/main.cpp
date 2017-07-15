#include <iostream>

#include <Windows.h>
#include <NuiApi.h>
#include <NuiKinectFusionApi.h>

#include <opencv2/opencv.hpp>



#define ERROR_CHECK( ret )  \
    if ( ret != S_OK ) {    \
    std::stringstream ss;	\
    ss << "failed " #ret " " << std::hex << ret << std::endl;			\
    throw std::runtime_error( ss.str().c_str() );			\
    }

const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

class KinectSample
{
private:

    INuiSensor* kinect;

    INuiFusionReconstruction*   m_pVolume;

    NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;
    NUI_FUSION_IMAGE_FRAME*     m_pPointCloud;
    NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;

    HANDLE imageStreamHandle;
    HANDLE depthStreamHandle;
    HANDLE streamEvent;

    DWORD width;
    DWORD height;

public:

    KinectSample()
        : kinect( 0 )
        , m_pVolume( 0 )
        , m_pDepthFloatImage( 0 )
        , m_pPointCloud( 0 )
        , m_pShadedSurface( 0 )
        , trackingErrorCount( 0 )
    {
    }

    ~KinectSample()
    {
        // �I������
        if ( kinect != 0 ) {
            kinect->NuiShutdown();
            kinect->Release();
        }
    }

    void initialize()
    {
        createInstance();

        // Kinect�̐ݒ������������
        ERROR_CHECK( kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON ) );

        // RGB�J����������������
        ERROR_CHECK( kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, CAMERA_RESOLUTION,
            0, 2, 0, &imageStreamHandle ) );

        // �����J����������������
        ERROR_CHECK( kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, CAMERA_RESOLUTION,
            0, 2, 0, &depthStreamHandle ) );

        // Near���[�h
        ERROR_CHECK( kinect->NuiImageStreamSetImageFrameFlags(
          depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE ) );

        // �X�P���g��������������
        ERROR_CHECK( kinect->NuiSkeletonTrackingEnable( 0, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT ) );

        // �t���[���X�V�C�x���g�̃n���h�����쐬����
        streamEvent = ::CreateEvent( 0, TRUE, FALSE, 0 );
        ERROR_CHECK( kinect->NuiSetFrameEndEvent( streamEvent, 0 ) );

        // �w�肵���𑜓x�́A��ʃT�C�Y���擾����
        ::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height );

        // KinectFusion�̏�����
        initializeKinectFusion();
    }

    void initializeKinectFusion()
    {
        HRESULT hr = S_OK;

        NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParams;
        reconstructionParams.voxelsPerMeter = 256;// 1000mm / 256vpm = ~3.9mm/voxel    
        reconstructionParams.voxelCountX = 512;   // 512 / 256vpm = 2m wide reconstruction
        reconstructionParams.voxelCountY = 384;   // Memory = 512*384*512 * 4bytes per voxel
        reconstructionParams.voxelCountZ = 512;   // This will require a GPU with at least 512MB

        // Reconstruction Volume �̃C���X�^���X�𐶐�
        hr = ::NuiFusionCreateReconstruction(
            &reconstructionParams,
            NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, // NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU
            -1, &IdentityMatrix(), &m_pVolume);
        if (FAILED(hr)){
            throw std::runtime_error( "::NuiFusionCreateReconstruction failed." );
        }

        // DepthFloatImage �̃C���X�^���X�𐶐�
        hr = ::NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, nullptr, &m_pDepthFloatImage);
        if (FAILED(hr)) {
            throw std::runtime_error( "::NuiFusionCreateImageFrame failed(Float)." );
        }

        // PointCloud �̃C���X�^���X�𐶐�
        hr = ::NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, nullptr, &m_pPointCloud);
        if (FAILED(hr)) {
            throw std::runtime_error( "::NuiFusionCreateImageFrame failed(PointCloud)." );
        }

        // �V�F�[�_�[�T�[�t�F�[�X�̃C���X�^���X�𐶐�
        hr = ::NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, nullptr, &m_pShadedSurface);
        if (FAILED(hr)) {
            throw std::runtime_error( "::NuiFusionCreateImageFrame failed(Color)." );
        }

        // ���Z�b�g
        m_pVolume->ResetReconstruction( &IdentityMatrix(), nullptr );
    }

    /// <summary>
    /// Set Identity in a Matrix4
    /// </summary>
    /// <param name="mat">The matrix to set to identity</param>
    const Matrix4& IdentityMatrix()
    {
        static Matrix4 mat;
        mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
        mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
        mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
        mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
        return mat;
    }

    void run()
    {
        cv::Mat image;

        // ���C�����[�v
        while ( 1 ) {
            // �f�[�^�̍X�V��҂�
            DWORD ret = ::WaitForSingleObject( streamEvent, INFINITE );
            ::ResetEvent( streamEvent );

            processDepth( image );

            // �摜��\������
            cv::imshow( "KinectSample", image );

            // �I���̂��߂̃L�[���̓`�F�b�N���A�\���̂��߂̃E�F�C�g
            int key = cv::waitKey( 10 );
            if ( key == 'q' ) {
                break;
            }
        }
    }

private:

    void createInstance()
    {
        // �ڑ�����Ă���Kinect�̐����擾����
        int count = 0;
        ERROR_CHECK( ::NuiGetSensorCount( &count ) );
        if ( count == 0 ) {
            throw std::runtime_error( "Kinect ��ڑ����Ă�������" );
        }

        // �ŏ���Kinect�̃C���X�^���X���쐬����
        ERROR_CHECK( ::NuiCreateSensorByIndex( 0, &kinect ) );

        // Kinect�̏�Ԃ��擾����
        HRESULT status = kinect->NuiStatus();
        if ( status != S_OK ) {
            throw std::runtime_error( "Kinect �����p�\�ł͂���܂���" );
        }
    }

    void drawRgbImage( cv::Mat& image )
    {
        // RGB�J�����̃t���[���f�[�^���擾����
        NUI_IMAGE_FRAME imageFrame = { 0 };
        ERROR_CHECK( kinect->NuiImageStreamGetNextFrame( imageStreamHandle, INFINITE, &imageFrame ) );

        // �摜�f�[�^���擾����
        NUI_LOCKED_RECT colorData;
        imageFrame.pFrameTexture->LockRect( 0, &colorData, 0, 0 );

        // �摜�f�[�^���R�s�[����
        image = cv::Mat( height, width, CV_8UC4, colorData.pBits );

        // �t���[���f�[�^���������
        ERROR_CHECK( kinect->NuiImageStreamReleaseFrame( imageStreamHandle, &imageFrame ) );
    }

    void processDepth( cv::Mat& mat )
    {
        // �����J�����̃t���[���f�[�^���擾����
        NUI_IMAGE_FRAME depthFrame = { 0 };
        ERROR_CHECK( kinect->NuiImageStreamGetNextFrame( depthStreamHandle, 0, &depthFrame ) );

        // �t���[���f�[�^�����ɁA�g�������f�[�^���擾����
		BOOL nearMode = FALSE;
		INuiFrameTexture *frameTexture = 0;
		kinect->NuiImageFrameGetDepthImagePixelFrameTexture( depthStreamHandle, &depthFrame, &nearMode, &frameTexture );

        // �����f�[�^���擾����
        NUI_LOCKED_RECT depthData = { 0 };
        frameTexture->LockRect( 0, &depthData, 0, 0 );
        if ( depthData.Pitch == 0 ) {
            std::cout << "zero" << std::endl;
        }

        // KinectFusion�̏������s��
        processKinectFusion( (NUI_DEPTH_IMAGE_PIXEL*)depthData.pBits, depthData.size, mat );

        // �t���[���f�[�^���������
        ERROR_CHECK( kinect->NuiImageStreamReleaseFrame( depthStreamHandle, &depthFrame ) );
    }

    int trackingErrorCount;
    void processKinectFusion( const NUI_DEPTH_IMAGE_PIXEL* depthPixel, int depthPixelSize, cv::Mat& mat ) 
    {
        // DepthImagePixel ���� DepthFloaatFrame �ɕϊ�����
        HRESULT hr = ::NuiFusionDepthToDepthFloatFrame( depthPixel, width, height, m_pDepthFloatImage,
                            NUI_FUSION_DEFAULT_MINIMUM_DEPTH, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH, TRUE );
        if (FAILED(hr)) {
            throw std::runtime_error( "::NuiFusionDepthToDepthFloatFrame failed." );
        }

        // �t���[������������
        Matrix4 worldToCameraTransform;
        m_pVolume->GetCurrentWorldToCameraTransform( &worldToCameraTransform );
        hr = m_pVolume->ProcessFrame( m_pDepthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
                                        NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, &worldToCameraTransform );
        if (FAILED(hr)) {
            // ��萔�G���[�ɂȂ����烊�Z�b�g
            // Kinect�܂��͑Ώۂ�f�������������� �Ȃǂ̏ꍇ
            ++trackingErrorCount;
            if ( trackingErrorCount >= 100 ) {
                trackingErrorCount = 0;
                m_pVolume->ResetReconstruction( &IdentityMatrix(), nullptr );
            }

            return;
        }

        // PointCloud���擾����
        hr = m_pVolume->CalculatePointCloud( m_pPointCloud, &worldToCameraTransform );
        if (FAILED(hr)) {
            throw std::runtime_error( "CalculatePointCloud failed." );
        }

        // PointCloud��2�����̃f�[�^�ɕ`�悷��
        hr = ::NuiFusionShadePointCloud( m_pPointCloud, &worldToCameraTransform,
                                    nullptr, m_pShadedSurface, nullptr );
        if (FAILED(hr)) {
            throw std::runtime_error( "::NuiFusionShadePointCloud failed." );
        }

        // 2�����̃f�[�^��Bitmap�ɏ�������
        INuiFrameTexture * pShadedImageTexture = m_pShadedSurface->pFrameTexture;
        NUI_LOCKED_RECT ShadedLockedRect;
        hr = pShadedImageTexture->LockRect(0, &ShadedLockedRect, nullptr, 0);
        if (FAILED(hr)) {
            throw std::runtime_error( "LockRect failed." );
        }

        mat = cv::Mat( height, width, CV_8UC4, ShadedLockedRect.pBits );

        // We're done with the texture so unlock it
        pShadedImageTexture->UnlockRect(0);
    }
};

void main()
{

    try {
        KinectSample kinect;
        kinect.initialize();
        kinect.run();
    }
    catch ( std::exception& ex ) {
        std::cout << ex.what() << std::endl;
    }
}
