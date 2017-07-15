/**
 * Tucano - A library for rapid prototying with Modern OpenGL and GLSL
 * Copyright (C) 2014
 * LCG - Laboratório de Computação Gráfica (Computer Graphics Lab) - COPPE
 * UFRJ - Federal University of Rio de Janeiro
 *
 *
 * Tucano Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Tucano Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Tucano Library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __CAMERA__
#define __CAMERA__

#include <Eigen/Dense>
#include <cmath>

namespace Tucano
{

/**
 * @brief Defines an abstract camera with a projection and view matrices.
 *
 * For a visual representation of a camera use the Shapers::CameraRep class
 **/
class Camera {

protected:

    /// Projection, or intrinsic, matrix.
    Eigen::Matrix4f projection_matrix;

    /// View, or extrinsic, matrix.
    Eigen::Affine3f view_matrix;

    /// Viewport dimensions [minX, minY, width, height].
    Eigen::Vector4f viewport;

    /// Near plane used for projection matrix.
    float near_plane;

    /// Far plane used for projection matrix.
    float far_plane;

    /// Frustum left dimension.
    float frustum_left;

    /// Frustum right dimension.
    float frustum_right;

    /// Frustum top dimension.
    float frustum_top;

    /// Frustum bottom dimension.
    float frustum_bottom;

    /// Aspect ratio for projection matrix.
    float aspect_ratio;

    /// Camera's Focal Length.
    float focal_length;

    /// Field of view angle in y axis.
    float fovy;

    /// Flag to indicate if using a perspective or othograpic projection.
    bool use_perspective;

public:

    /**
     * @brief Reset view matrix
     */
    void resetViewMatrix (void)
    {
        view_matrix = Eigen::Affine3f::Identity();
    }

    /**
     * @brief Reset projection matrix
     */
    void resetProjectionMatrix (void)
    {
        projection_matrix = Eigen::Matrix4f::Identity();
    }

    /**
     * @brief Resets trackball to initial position and orientation
     */
    void reset (void)
    {
        resetViewMatrix();
        resetProjectionMatrix();
    }


    /**
     * @brief Returns the center of the camera in world space.
     * @return Camera center
     */
    Eigen::Vector3f getCenter (void) const
    {
        return view_matrix.linear().inverse() * (-view_matrix.translation());
    }

    /**
     * @brief Return the modelview matrix as a GLdouble array.
     *
     * Similar to OpenGL old method using glGet**(GL_MODELVIEW_MATRIX)
     * @param matrix A pointer to a GLdouble of at least 16 elements to fill with view matrix.
     */
    void getViewMatrix (GLdouble *matrix)
    {
        Eigen::Matrix4f mv = view_matrix.matrix();
        for (int i = 0; i < 16; ++i)
        {
            matrix[i] = mv(i);
        }
    }

    /**
     * @brief Return the projection matrix as a GLdouble array.
     *
     * Similar to OpenGL old method using glGet**(GL_PROJECTION_MATRIX)
     * @param matrix A pointer to a GLdouble of at least 16 elements to fill with projection matrix.
     */
    void getProjectionMatrix (GLdouble *matrix)
    {
        for (int i = 0; i < 16; ++i)
        {
            matrix[i] = projection_matrix(i);
        }
    }

    /**
     * @brief Returns screen space coordinates of a 3D point.
     *
     * Projects a given point to screen space using given viewport coordinates.
     * @param pt Given point to be projected.
     * @param viewport Vector containing viewport coordinates in following order [minX, minY, maxX, maxY]
     */
    inline Eigen::Vector3f projectPoint (const Eigen::Vector4f& pt, Eigen::Vector4f& viewport)
    {
        Eigen::Vector3f screen_pt = Eigen::Vector3f::Zero();
        Eigen::Vector4f proj = view_matrix * projection_matrix * pt;
        if (proj[3] == 0.0)
            return screen_pt;

        proj /= proj[3];

        screen_pt[0] = viewport[0] + viewport[2] * (proj[0] * 0.5 + 0.5);
        screen_pt[1] = viewport[1] + viewport[3] * (proj[1] * 0.5 + 0.5);
        screen_pt[2] = proj[2]*0.5 + 0.5;
        return screen_pt;
    }


    /**
     * @brief Returns the view matrix as an Affine 3x3 matrix
     * @return View Matrix.
     */
    Eigen::Affine3f getViewMatrix (void) const
    {
        return view_matrix;
    }

	/**
     * @brief Returns a pointer to the view matrix as an Affine 3x3 matrix
     * @return Pointer to View Matrix.
     */
	 Eigen::Affine3f* viewMatrix (void)
	{
		return &view_matrix;
	}

    /**
     * @brief Returns the view matrix as an 4x4 matrix
     * @return Projection Matrix.
     */
    Eigen::Matrix4f getProjectionMatrix (void) const
    {
        return projection_matrix;
    }

    /**
     * @brief Returns a pointer to the projection matrix as an 4x4 matrix
     * @return Pointer to projection Matrix.
     */
    Eigen::Matrix4f* projectionMatrix (void) 
    {
        return &projection_matrix;
    }


    /**
     * @brief Returns a 3x3 matrix containing only the rotation of the view matrix.
     * @return The rotation part of the view matrix as a 3x3 matrix.
     */
    Eigen::Matrix3f getRotationMatrix (void) const
    {
        return view_matrix.rotation();
    }


    /**
     * @brief Returns the translation part of the view matrix as a vector.
     * @return The translation part of the view matrix.
     */
    Eigen::Vector3f getTranslationMatrix (void) const
    {
        return view_matrix.translation();
    }    

    /**
     * @brief Returns the perspective scale.
     *
     * Usually this is element(1,1) of the projection matrix.
     * @return The perspective scale factor
     */
    float getPerspectiveScale (void) const
    {
        return (float)1.0f/tan((fovy/2.0f)*(M_PI/180.0f));
    }

    /**
     * @brief Returns the viewport coordinates.
     *
     * Viewport vector is as follows [minX, minY, width, height]
     * @return Viewport coordinates.
     */
    Eigen::Vector4f getViewport (void) const
    {
        return viewport;
    }

    /**
     * @brief Returns the dimensions of the viewport.
     *
     * @return Viewport dimensions.
     */
    Eigen::Vector2i getViewportSize (void) const
    {
        return Eigen::Vector2i(viewport[2], viewport[3]);
    }

	/**
	* @brief Returns the viewport aspect ratio
	* @return Viewport aspect ratio x/y
	*/
	float getViewportAspectRatio (void) const
	{
		return (viewport[2] / viewport[3]);
	}

    /**
     * @brief Sets the viewport coordinates.
     * @param vp Viewport coordinates.
     */
    void setViewport (const Eigen::Vector4f &vp)
    {
        viewport = vp;
    }

    /**
     * @brief Sets the viewport coordinates considering that the minimum coordinates are zero.
     *
     * Sets the new viewport as [0, 0, vp[0], vp[1]]
     * @param vp Viewport coordinates.
     */
    void setViewport (const Eigen::Vector2f &vp)
    {
        viewport = Eigen::Vector4f(0.0, 0.0, vp[0], vp[1]);
    }

    /**
     * @brief Sets the projection matrix from a given 4x4 matrix.
     * @param mat Given 4x4 matrix to set as projection matrix.
     */
    void setProjectionMatrix(const Eigen::Matrix4f& mat)
    {
        projection_matrix = mat;
    }

	 /**
     * @brief Sets the view matrix from a given an affine 3x3 matrix.
     * @param mat Given Affine 3x3 matrix to set as view matrix.
     */
    void setViewMatrix(const Eigen::Affine3f& mat)
    {
        view_matrix = mat;
    }


    /**
     * @brief Returns near plane value.
     * @return Near plane.
     */
    float getNearPlane (void) const
    {
        return near_plane;
    }

    /**
     * @brief Returns far plane value.
     * @return Far plane.
     */
    float getFarPlane (void) const
    {
        return far_plane;
    }

    /**
     * @brief Default destructor.
     */
    ~Camera()
    {
    }

    /**
      * @brief Returns current field of view angle in y axis.
      * @return Field of view angle.
      */
    float getFovy (void) const
    {
        return fovy;
    }

public:

    /**
     * @brief Default constructor
     */
    Camera (void)
    {

        frustum_left = -1.0;
        frustum_right = 1.0;
        frustum_bottom = -1.0;
        frustum_top = -1.0;
        near_plane = 0.1;
        far_plane = 100.0;
        fovy = 60.0;

        reset();
    }


    /**
     * @brief Returns a perspective projection matrix with the given parameters
     * @param fy Vertical field of view angle
     * @param in_aspect_ratio Ratio of width to the height of the viewport
     * @param in_near_plane Near plane
     * @param in_far_plane Far plane
     * @return The created perspective matrix.
     */
    static Eigen::Matrix4f createPerspectiveMatrix (float fy, float in_aspect_ratio, float in_near_plane,float in_far_plane)
    {
        Eigen::Matrix4f out = Eigen::Matrix4f::Zero();

        const float
                y_scale = (float)1.0/tan((fy/2.0)*(M_PI/180.0)),
                x_scale = y_scale / in_aspect_ratio,
                frustum_length = in_far_plane - in_near_plane;

        out(0,0) = x_scale;
        out(1,1) = y_scale;
        out(2,2) = -((in_far_plane + in_near_plane) / frustum_length);
        out(3,2) = -1.0;
        out(2,3) = -((2 * in_near_plane * in_far_plane) / frustum_length);

        return out;
    }

    /**
     * @brief Sets the projection matrix as a perspective matrix.
     *
     * Creates a perspective matrix with the given parameters and sets as the projection matrix.
     * @param fy Vertical field of view angle
     * @param in_aspect_ratio Ratio of width to the height of the viewport
     * @param in_near_plane Near plane
     * @param in_far_plane Far plane
     * @return The created perspective matrix.
     */
    Eigen::Matrix4f setPerspectiveMatrix (float fy, float in_aspect_ratio, float in_near_plane,float in_far_plane)
    {
        fovy = fy;
        aspect_ratio = in_aspect_ratio;
        near_plane = in_near_plane;
        far_plane = in_far_plane;

        Eigen::Matrix4f proj = createPerspectiveMatrix(fovy, aspect_ratio, near_plane, far_plane);
        setProjectionMatrix(proj);
        use_perspective = true;
        return proj;
    }

    /**
     * @brief Changes the fovy and computes new perspective projection matrix.
     * @param new_fovy New value for field of view.
     */
    void changeFovy (float new_fovy)
    {
        fovy = new_fovy;
        setPerspectiveMatrix(fovy, aspect_ratio, near_plane, far_plane);
    }

    /**
     * @brief Returns an orthographic projection matrix with the given parameters
     * @param left Left plane for orthographic view.
     * @param right Right plane for orthographic view.
     * @param bottom Bottom plane for orthographic view.
     * @param top Top lane for orthographic view.
     * @param near_plane Near plane for orthographic view.
     * @param far_plane Far plane for orthographic view.
     * @return The created orthographic matrix.
     */
    static Eigen::Matrix4f createOrthographicMatrix (float left, float right, float bottom, float top, float near_plane, float far_plane)
    {
        Eigen::Matrix4f out = Eigen::Matrix4f::Zero();

        out(0,0) = 2.0/(right-left);
        out(1,1) = 2.0/(top-bottom);
        out(2,2) = -2.0/(far_plane-near_plane);
        out(3,3) = 1.0;
        out(0,3) = -(right+left)/(right-left);
        out(1,3) = -(top+bottom)/(top-bottom);
        out(2,3) = -(far_plane+near_plane)/(far_plane-near_plane);

        return out;
    }

    /**
     * @brief Sets the projection matrix as a orthographic matrix.
     *
     * Creates an orthographic projection matrix with the given parameters and sets as the projection matrix.
     * @param left Left plane for orthographic view.
     * @param right Right plane for orthographic view.
     * @param bottom Bottom plane for orthographic view.
     * @param top Top lane for orthographic view.
     * @param near_plane Near plane for orthographic view.
     * @param far_plane Far plane for orthographic view.
     * @return The created orthographic matrix.
     */
    Eigen::Matrix4f setOrthographicMatrix (float left, float right, float bottom, float top, float near_plane, float far_plane)
    {
        Eigen::Matrix4f proj = createOrthographicMatrix(left, right, bottom, top, near_plane, far_plane);
        setProjectionMatrix(proj);
        use_perspective = false;
        return proj;
    }

    /**
     * @brief Increases the fov of the perspective matrix by a given increment.
     * @param inc Given increment to increase fov
     */
    void incrementFov (float inc)
    {
        if (use_perspective)
        {
            changeFovy(fovy + inc);
        }
    }

    /**
     * @brief Translates the view matrix by a given vector.
     * @param translation Translation to apply to view matrix.
     */
    void translate (const Eigen::Vector3f& translation)
    {
        view_matrix.translate(translation);
    }

    /**
     * @brief Rotate the view matrix by a given quaternion.
     * @param rotation Rotation to apply to view matrix.
     */
    void rotate (const Eigen::Quaternion<float>& rotation)
    {
        view_matrix.rotate(rotation);
    }

    /**
     * @brief Scales the view matrix by given factors.
     * @param scale_factors Scale factors in x, y, and z axis.
     */
    void scale (const Eigen::Vector3f& scale_factors)
    {
        view_matrix.scale(scale_factors);
    }

    /**
     * @brief Scales the view matrix by given factor in all axis.
     * @param scale_factor Scale factors in all axis.
     */
    void scale (float scale_factor)
    {
        view_matrix.scale(scale_factor);
    }
};

}
#endif
