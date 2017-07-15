// pcd_writer provides functions for save Kinect Fusion mesh data to PCD (Point Cloud Data) file.
//
// This source code is licensed under the MIT license. Please see the License in License.txt.
// Copyright (c) 2017 Tsukasa SUGIURA
// t.sugiura0204@gmail.com

#ifndef PCD_WRITER
#define PCD_WRITER

#include <Windows.h>
#include <NuiKinectFusionApi.h>
#include <fstream>

HRESULT WriteAsciiPcdFile(INuiFusionColorMesh* mesh, const std::string& file, const bool flip = true, const bool color = false )
{
    //HRESULT ret = S_OK;

    if( mesh == nullptr ){
        return E_INVALIDARG;
    }

    // Retrieve Vertex Count
    const unsigned int verticesCount = mesh->VertexCount();

    // Retrieve Vertices
    const Vector3* vertices = nullptr;
    HRESULT ret = mesh->GetVertices( &vertices );
    if( FAILED( ret ) ){
        return ret;
    }

    // Retrieve Colors
    const int* colors = nullptr;
    if( color ){
        ret = mesh->GetColors( &colors );
        if( FAILED( ret ) ){
            return ret;
        }
    }

    // Open Write File Stream
    std::ofstream ofs;
    ofs.open( file, std::ios::out );
    if( !ofs.is_open() ){
        return E_FAIL;
    }

    // Write Header COMMENTS
    const float version = 0.7f;
    ofs << "# .PCD v" << version << " - Point Cloud Data file format\n";

    // Write Header VERSION
    ofs << "VERSION " << version << "\n";

    if( color ){
        // Write Header FIELDS
        ofs << "FIELDS x y z rgb\n";

        // Write Header SIZE
        ofs << "SIZE 4 4 4 4\n";

        // Write Header TYPE
        ofs << "TYPE F F F I\n";

        // Write Header COUNT
        ofs << "COUNT 1 1 1 1\n";
    }
    else{
        // Write Header FIELDS
        ofs << "FIELDS x y z\n";

        // Write Header SIZE
        ofs << "SIZE 4 4 4\n";

        // Write Header TYPE
        ofs << "TYPE F F F\n";

        // Write Header COUNT
        ofs << "COUNT 1 1 1\n";
    }

    // Write Header WIDTH
    ofs << "WIDTH " << verticesCount << "\n";

    // Write Header HEIGHT
    ofs << "HEIGHT 1\n";

    // Write Header VIEWPOINT
    ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";

    // Write Header POINTS
    ofs << "POINTS " << verticesCount << "\n";

    // Write Header DATA
    ofs << "DATA ascii\n";

    // Write Points
    if( flip ){
        if( color ){
            for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                const uint8_t r = ( colors[index] >> 16 ) & 0xff;
                const uint8_t g = ( colors[index] >>  8 ) & 0xff;
                const uint8_t b = ( colors[index]       ) & 0xff;
                const uint32_t color = ( static_cast<uint32_t>( r ) << 16 | static_cast<uint32_t>( g ) << 8 | static_cast<uint32_t>( b ) );
                ofs << vertex.x << " " << -vertex.y << " " << -vertex.z << " " << color << "\n";
            }
        }
        else{
            for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                ofs << vertex.x << " " << -vertex.y << " " << -vertex.z << "\n";
            }
        }
    }
    else{
        if( color ){
            for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                const uint8_t r = ( colors[index] >> 16 ) & 0xff;
                const uint8_t g = ( colors[index] >> 8  ) & 0xff;
                const uint8_t b = ( colors[index]       ) & 0xff;
                const uint32_t color = ( static_cast<uint32_t>( r ) << 16 | static_cast<uint32_t>( g ) << 8 | static_cast<uint32_t>( b ) );
                ofs << vertex.x << " " << vertex.y << " " << vertex.z << " " << colors << "\n";
            }
        }
        else{
            for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                ofs << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
            }
        }
    }

    // Flush Buffer
    ofs.flush();

    // Close Write File Stream
    ofs.close();

    return ret;
}

HRESULT WriteBinaryPcdFile(INuiFusionColorMesh* mesh, const std::string& file, const bool flip = true, const bool color = false )
{
    //HRESULT ret = S_OK;

    if( mesh == nullptr ){
        return E_INVALIDARG;
    }

    // Retrieve Vertex Count
    unsigned int verticesCount = mesh->VertexCount();

    // Retrieve Vertices
    const Vector3* vertices = nullptr;
    HRESULT ret = mesh->GetVertices( &vertices );
    if( FAILED( ret ) ){
        return ret;
    }

    // Retrieve Colors
    const int* colors = nullptr;
    if( color ){
        ret = mesh->GetColors( &colors );
        if( FAILED( ret ) ){
            return ret;
        }
    }

    // Open Write File Stream
    std::ofstream ofs;
    ofs.open( file, std::ios::out | std::ios::binary );
    if( !ofs.is_open() ){
        return E_FAIL;
    }

    // Write Header COMMENTS
    const float version = 0.7f;
    ofs << "# .PCD v" << version << " - Point Cloud Data file format\n";

    // Write Header VERSION
    ofs << "VERSION " << version << "\n";

    if( color ){
        // Write Header FIELDS
        ofs << "FIELDS x y z rgb\n";

        // Write Header SIZE
        ofs << "SIZE 4 4 4 4\n";

        // Write Header TYPE
        ofs << "TYPE F F F I\n";

        // Write Header COUNT
        ofs << "COUNT 1 1 1 1\n";
    }
    else{
        // Write Header FIELDS
        ofs << "FIELDS x y z\n";

        // Write Header SIZE
        ofs << "SIZE 4 4 4\n";

        // Write Header TYPE
        ofs << "TYPE F F F\n";

        // Write Header COUNT
        ofs << "COUNT 1 1 1\n";
    }

    // Write Header WIDTH
    ofs << "WIDTH " << verticesCount << "\n";

    // Write Header HEIGHT
    ofs << "HEIGHT 1\n";

    // Write Header VIEWPOINT
    ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";

    // Write Header POINTS
    ofs << "POINTS " << verticesCount << "\n";

    // Write Header DATA
    ofs << "DATA binary\n";

    // Write Points
    if( flip ){
        if( color ){
             for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                const uint8_t r = ( colors[index] >> 16 ) & 0xff;
                const uint8_t g = ( colors[index] >> 8  ) & 0xff;
                const uint8_t b = ( colors[index]       ) & 0xff;
                const uint32_t color = ( static_cast<uint32_t>( r ) << 16 | static_cast<uint32_t>( g ) << 8 | static_cast<uint32_t>( b ) );
                ofs << vertex.x << " " << -vertex.y << " " << -vertex.z << " " << color << "\n";
            }
        }
        else{
            for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                ofs << vertex.x << " " << -vertex.y << " " << -vertex.z << "\n";
            }
        }
    }
    else{
        if( color ){
            for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                const uint8_t r = ( colors[index] >> 16 ) & 0xff;
                const uint8_t g = ( colors[index] >> 8  ) & 0xff;
                const uint8_t b = ( colors[index]       ) & 0xff;
                const uint32_t color = ( static_cast<uint32_t>( r ) << 16 | static_cast<uint32_t>( g ) << 8 | static_cast<uint32_t>( b ) );
                ofs << vertex.x << " " << vertex.y << " " << vertex.z << " " << colors << "\n";
            }
        }
        else{
            for( unsigned int index = 0; index < verticesCount; index++ ){
                const Vector3 vertex = vertices[index];
                ofs << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
            }
        }
    }

    // Flush Buffer
    ofs.flush();

    // Close Write File Stream
    ofs.close();

    return ret;
}

HRESULT WriteAsciiPcdFile(INuiFusionMesh* mesh, const std::string& file, const bool flip = true )
{
    //HRESULT ret = S_OK;

    if( mesh == nullptr ){
        return E_INVALIDARG;
    }

    // Retrieve Vertex Count
    unsigned int verticesCount = mesh->VertexCount();

    // Retrieve Vertices
    const Vector3* vertices = nullptr;
    HRESULT ret = mesh->GetVertices( &vertices );
    if( FAILED( ret ) ){
        return ret;
    }

    // Open Write File Stream
    std::ofstream ofs;
    ofs.open( file, std::ios::out );
    if( !ofs.is_open() ){
        return E_FAIL;
    }

    // Write Header COMMENTS
    const float version = 0.7f;
    ofs << "# .PCD v" << version << " - Point Cloud Data file format\n";

    // Write Header VERSION
    ofs << "VERSION " << version << "\n";

    // Write Header FIELDS
    ofs << "FIELDS x y z\n";

    // Write Header SIZE
    ofs << "SIZE 4 4 4\n";

    // Write Header TYPE
    ofs << "TYPE F F F\n";

    // Write Header COUNT
    ofs << "COUNT 1 1 1\n";

    // Write Header WIDTH
    ofs << "WIDTH " << verticesCount << "\n";

    // Write Header HEIGHT
    ofs << "HEIGHT 1\n";

    // Write Header VIEWPOINT
    ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";

    // Write Header POINTS
    ofs << "POINTS " << verticesCount << "\n";

    // Write Header DATA
    ofs << "DATA ascii\n";

    // Write Points
    if( flip ){
        for( unsigned int index = 0; index < verticesCount; index++ ){
            const Vector3 vertex = vertices[index];
            ofs << vertex.x << " " << -vertex.y << " " << -vertex.z << "\n";
        }
    }
    else{
        for( unsigned int index = 0; index < verticesCount; index++ ){
            const Vector3 vertex = vertices[index];
            ofs << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
        }
    }

    // Flush Buffer
    ofs.flush();

    // Close Write File Stream
    ofs.close();

    return ret;
}

HRESULT WriteBinaryPcdFile(INuiFusionMesh* mesh, const std::string& file, const bool flip = true )
{
    //HRESULT ret = S_OK;

    if( mesh == nullptr ){
        return E_INVALIDARG;
    }

    // Retrieve Vertex Count
    unsigned int verticesCount = mesh->VertexCount();

    // Retrieve Vertices
    const Vector3* vertices = nullptr;
    HRESULT ret = mesh->GetVertices( &vertices );
    if( FAILED( ret ) ){
        return ret;
    }

    // Open Write File Stream
    std::ofstream ofs;
    ofs.open( file, std::ios::out | std::ios::binary );
    if( !ofs.is_open() ){
        return E_FAIL;
    }

    // Write Header COMMENTS
    const float version = 0.7f;
    ofs << "# .PCD v" << version << " - Point Cloud Data file format\n";

    // Write Header VERSION
    ofs << "VERSION " << version << "\n";

    // Write Header FIELDS
    ofs << "FIELDS x y z\n";

    // Write Header SIZE
    ofs << "SIZE 4 4 4\n";

    // Write Header TYPE
    ofs << "TYPE F F F\n";

    // Write Header COUNT
    ofs << "COUNT 1 1 1\n";

    // Write Header WIDTH
    ofs << "WIDTH " << verticesCount << "\n";

    // Write Header HEIGHT
    ofs << "HEIGHT 1\n";

    // Write Header VIEWPOINT
    ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";

    // Write Header POINTS
    ofs << "POINTS " << verticesCount << "\n";

    // Write Header DATA
    ofs << "DATA binary\n";

    // Write Points
    if( flip ){
        for( unsigned int index = 0; index < verticesCount; index++ ){
            const Vector3 vertex = vertices[index];
            ofs << vertex.x << " " << -vertex.y << " " << -vertex.z << "\n";
        }
    }
    else{
        for( unsigned int index = 0; index < verticesCount; index++ ){
            const Vector3 vertex = vertices[index];
            ofs << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
        }
    }

    // Flush Buffer
    ofs.flush();

    // Close Write File Stream
    ofs.close();

    return ret;
}

#endif PCD_WRITER
