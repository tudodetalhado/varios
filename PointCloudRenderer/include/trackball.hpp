/**
 * Tucano - A library for rapid prototying with Modern OpenGL and GLSL
 * Copyright (C) 2014
 * LCG - Laboratório de Computação Gráfica (Computer Graphics Lab) - COPPE
 * UFRJ - Federal University of Rio de Janeiro
 *
 * This file is part of Tucano Library.
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

#ifndef __TRACKBALL__
#define __TRACKBALL__

#include "camera.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace Tucano
{

/// Default fragment shader for rendering trackball representation.
const std::string trackball_fragment_code = "\n"
        "#version 430\n"
        "in vec4 ex_Color;\n"
        "out vec4 out_Color;\n"
        "in float depth;\n"
        "void main(void)\n"
        "{\n"
        "    out_Color = ex_Color;\n"
        "    gl_FragDepth = depth;\n"
        "}\n";

/// Default vertex shader for rendering trackball representation.
const std::string trackball_vertex_code = "\n"
        "#version 430\n"
        "layout(location=0) in vec4 in_Position;\n"
        "out vec4 ex_Color;\n"
        "out float depth;\n"
        "uniform mat4 modelMatrix;\n"
        "uniform mat4 viewMatrix;\n"
        "uniform mat4 projectionMatrix;\n"
        "uniform vec4 in_Color;\n"
        "uniform float nearPlane;\n"
        "uniform float farPlane;\n"
        "void main(void)\n"
        "{\n"
        "   gl_Position = (viewMatrix * modelMatrix) * in_Position;\n"
        "   depth = (farPlane+nearPlane)/(farPlane-nearPlane) + ( (2*nearPlane*farPlane)/(farPlane-nearPlane) ) * (1/gl_Position[2]);\n"
        "   depth = (depth+1.0)/2.0;\n"
        "   gl_Position = projectionMatrix * gl_Position;\n"
        "   ex_Color = in_Color;\n"
        "}\n";

/**
 * @brief Trackball class for manipulating a camera.
 *
 * This class simulates a standard trackball manipulation.
 * The visual representation is a sphere with radius equal to 0.8 centered in the middle of the window (represented by the equation: x^2 + y^2 + z^2 = r^2),
 * nested in an hyperbolic sheet opening (represented by the equation: z = (r^2/2)/sqrt(x^2+y^2).
 * It is usually used for camera and light manipulation.
 * It's important to notice that the x and y mouse coordinates passed to the trackball must be ranging from (-1,-1), being the
 * lower left corner of screen, to (1,1) being the higher right corner. This means the coordinate system is centered in screen
 * and normalized. Also, the view matrix is by default translated by the vector [0,0,-2], in order to remove the camera from the center of the world.
 **/
class Trackball : public Tucano::Camera {


private:

    /// The current scale being applied to the View Matrix.
    float zoom;

    /// Flag that indicates wether the trackball is being rotated.
    bool rotating;

    /// Flag that indicates wether the trackball is being translated.
    bool translating;

    /// Flag that indicates wether the trackball's representation should be drawn.
    bool drawTrackball;

    /// Projection matrix used for the trackball's drawing. By default it is defined as an orthogonal projection matrix.
    Eigen::Matrix4f trackballProjectionMatrix;

    /// Initial position vector used to calculate trackball's rotation.
    Eigen::Vector3f initialPosition;

    /// Final position vector used to calculate trackball's rotation.
    Eigen::Vector3f finalPosition;

    /// Initial position vector used to calculate trackball's translation.
    Eigen::Vector2f initialTranslationPosition;

    /// Final position vector used to calculate trackball's translation.
    Eigen::Vector2f finalTranslationPosition;

    /// Trackball's quaternion.
    Eigen::Quaternion<float> quaternion;

    /// Trackball's translation vector.
    Eigen::Vector3f translationVector;

    /// Default translation to move camera away from center
    Eigen::Vector3f defaultTranslation;

    /// The vertices for the first circle of the trackball's representation.
    float vertices[800];

    /// Buffer Objects for trackball's representation:
    GLuint* bufferIDs;

    /// The trackball radius. It's defined as 0.8 times the smallest viewport dimension.
    float radius;

    /// Flag to use default shaders set as const strings in class, or pass a directory with custom shaders
    bool use_default_shaders;


public:

    /**
     * @brief Resets trackball to initial position and orientation
     */
    void reset (void)
    {
        quaternion = Eigen::Quaternion<float>::Identity();
        zoom = 1.0;
        translationVector << 0.0, 0.0, 0.0;
        rotating = false;
        translating = false;
        Camera::reset();
        updateViewMatrix();
    }

    /**
     * @brief Returns wether the trackball is being rotated or not.
     * @return True if rotating, false otherwise.
     */
    bool isRotating (void)
    {
        return rotating;
    }

    /**
     * @brief Returns wether the trackball is being translated or not.
     * @return True if translating, false otherwise.
     */
    bool isTranslating (void)
    {
        return translating;
    }

    /**
     * @brief Returns the default translation for placing the camera outside the trackball sphere.
     * @return The default translation vector for the view matrix
     */
    Eigen::Vector3f getDefaultTranslation (void)
    {
        return defaultTranslation;
    }

    /**
     * @brief Retuns the zoom factor (the radius of the trackball).
     * @return Zoom factor.
     */
    float getZoom (void)
    {
        return zoom;
    }


    /**
     * @brief Sets the projection matrix used for the trackball rendering.
     * Note that this is usually different than the projection matrix for the scene.
     * @param mat Given trackball projection matrix.
     */
    void setTrackballProjectionMatrix (const Eigen::Matrix4f& mat)
    {
        trackballProjectionMatrix = mat;
    }


    /**
     * @brief Indicates that a rotation has ended.
     * Disables the rotating flag, indicating that the mouse callback functions will stop reading the mouse coordinates in order to stop the trackball's rotation.
     */
    void endRotation (void)
    {
        rotating = false;
    }

    /**
     * @brief Indicates that a translation has ended.
     * Disable the translating flag, indicating that the mouse callback functions will stop reading the mouse coordinates in order to stop the trackball's translation.
     */
    void endTranslation (void)
    {
        translating = false;
    }



    /**
     * @brief Returns wether the trackball representation should be drawn or not.
     * @return True if trackball representation is being rendered, false otherwise.
     */
    void setRenderFlag(bool flag)
    {
        drawTrackball = flag;
    }


    ///Default destructor.
    ~Trackball() {
        ///Delete buffers:
        if (bufferIDs) {
            delete [] bufferIDs;
        }
    }

private:

    /**
     * @brief Computes 3d coordinates on sphere from 2d position.
     * @param pos Given 2d screen position.
     */
    Eigen::Vector3f computeSpherePosition (const Eigen::Vector2f& pos)
    {
        //In order to handle points outside the sphere, we will use two surfaces: A Sphere with radius = 0.8 and a Hyperbolic Sheet. More information about this usage in: http://www.opengl.org/wiki/Trackball.

        float z;

        //If pos corresponds to a point before the intersection of the two surfaces, z can be calculated by the Sphere's equation.
        if(pos[0]*pos[0] + pos[1]*pos[1] <= (radius*radius)/2.0) {
            z = sqrt(radius*radius - pos[0]*pos[0] - pos[1]*pos[1]);
        }

        //Else, it will be calculated by the Hyperbolic Sheet's equation.
        else {
            z = (radius*radius)/(2.0*sqrt(pos[0]*pos[0] + pos[1]*pos[1]));
        }

        Eigen::Vector3f position = Eigen::Vector3f(pos[0], pos[1], z);

        return position;
    }

public:

    /**
     * @brief Initializes the view and projection matrices.
     * They are all initialized as Identity matrices, but view is then translated by the defaultTranslation vector.
     */
    void initOpenGLMatrices (void)
    {
        // reset all matrices
        reset();

        // translate viewMatrix outside the origin to be able to see the model.
        translate(defaultTranslation);

        trackballProjectionMatrix = Eigen::Matrix4f::Identity();
    }

    /**
     * @brief Default constructor.
     * @param shader_dir Given directory containing trackball shaders, optional, otherwise uses default shaders.
     */
    Trackball ()
    {
        // creates the mesh that will be used to represent the trackball's sphere.
        createTrackballRepresentation();

        // initialize buffers used for trackball rendering:
        bufferIDs = new GLuint[3];

        defaultTranslation << 0.0, 0.0, -4.0;

        zoom = 1.0;

        drawTrackball = true;
        radius = 0.8f;

        initOpenGLMatrices();
        reset();
    }

    /**
     * @brief Sets the trackball projection matrix as perspective.
     * @param fy Vertical field of view angle.
     * @param aspect_ratio Ratio of width to the height of the viewport.
     * @param near_plane Near plane.
     * @param far_plane Far plane.
     * @return Return the created perspective matrix.
     */
    Eigen::Matrix4f setTrackballPerspectiveMatrix (float fy, float aspect_ratio, float near_plane, float far_plane)
    {
        Eigen::Matrix4f proj = createPerspectiveMatrix(fy, aspect_ratio, near_plane, far_plane);
        setTrackballProjectionMatrix(proj);
        return proj;
    }

    /**
     * @brief Sets the trackball projection matrix as orthographic.
     * @param left Left plane for orthographic view.
     * @param right Right plane for orthographic view.
     * @param bottom Bottom plane for orthographic view.
     * @param top Top lane for orthographic view.
     * @param near Near plane for orthographic view.
     * @param far Far plane for orthographic view.
     * @return Return the created orthographic matrix.
     */
    Eigen::Matrix4f setTrackballOrthographicMatrix (float left, float right, float bottom, float top, float near_plane, float far_plane)
    {
        Eigen::Matrix4f proj = createOrthographicMatrix(left, right, bottom, top, near_plane, far_plane);
        setTrackballProjectionMatrix(proj);
        return proj;
    }

    /**
     * @brief Computes the trackball's rotation, using stored initial and final position vectors.
     */
    void computeRotationAngle (void)
    {
        //Given two position vectors, corresponding to the initial and final mouse coordinates, calculate the rotation of the sphere that will keep the mouse always in the initial position.

        if(initialPosition.norm() > 0) {
            initialPosition.normalize();
        }
        if(finalPosition.norm() > 0) {
            finalPosition.normalize();
        }

        //cout << "Initial Position: " << initialPosition.transpose() << " Final Position: " << finalPosition.transpose() << endl << endl;

        Eigen::Vector3f rotationAxis = initialPosition.cross(finalPosition);

        if(rotationAxis.norm() != 0) {
            rotationAxis.normalize();
        }

        float dot = initialPosition.dot(finalPosition);

        float rotationAngle = (dot <= 1) ? acos(dot) : 0;//If, by losing floating point precision, the dot product between the initial and final positions is bigger than one, ignore the rotation.

        Eigen::Quaternion<float> q (Eigen::AngleAxis<float>(rotationAngle,rotationAxis));

        quaternion = q * quaternion;
        quaternion.normalize();
    }

    /**
     * @brief Compute the trackball's translation, using stored initial and final position vectors.
     */
    void computeTranslationVector (void)
    {
        Eigen::Vector2f translationFactor = finalTranslationPosition - initialTranslationPosition;
        translationVector += quaternion.inverse() * Eigen::Vector3f(translationFactor[0],translationFactor[1], 0.0);
    }

    /**
    * @brief Nomalizes a screen position to range [-1,1].
    * @param pos Screen position
    * @return Returns position in normalized coordinates.
    */
    Eigen::Vector2f normalizePosition (const Eigen::Vector2f& pos)
    {
        return Eigen::Vector2f ((pos[0]/((viewport[2]-viewport[0])/2.0)) - 1.0,
                                1.0 - (pos[1]/((viewport[3] - viewport[1])/2.0)));
    }

    /**
     * @brief Computes and applies the rotation transformations to the trackball given new position.
     * @param pos New mouse position in normalized trackball system
     */
    void rotateCamera (const Eigen::Vector2f& pos)
    {
        Eigen::Vector3f normalized_pos = computeSpherePosition(normalizePosition(pos));

        if (!rotating)
        {
            rotating = true;
            initialPosition = normalized_pos;
        }
        else if ( pos != initialPosition.head(2))
        {
            finalPosition = normalized_pos;
            computeRotationAngle();
            updateViewMatrix();
            initialPosition = finalPosition;
        }
    }

    /**
     * @brief Computes and applies the translation transformations to the trackball given new position.
     * @param pos New mouse position in normalized trackball system
     */
    void translateCamera (const Eigen::Vector2f& pos)
    {
        Eigen::Vector2f normalized_pos = normalizePosition(pos);
        if (!translating)
        {
            translating = true;
            initialTranslationPosition = normalized_pos;
        }
        else if (pos != initialPosition.head(2))
        {
            finalTranslationPosition = normalized_pos;
            computeTranslationVector();
            updateViewMatrix();
            initialTranslationPosition = finalTranslationPosition;
        }
    }


    /**
     * @brief Increases the zoom on the scene by appling a scale to the View Matrix.
     * The current scale used in View Matrix is multiplied by the given scale factor.
     * @param scale The given scale to be multiplied by current scale factor.
     */
    void increaseZoom (float scale)
    {
        zoom *= scale;
        updateViewMatrix();
    }

    /**
     * @brief Decreases the zoom on the scene by appling a scale to the View Matrix.
     * The current scale used in View Matrix is divided by the given scale factor.
     * @param scale The given scale factor, to which the current scale factor will be divided by.
     */
    void decreaseZoom (float scale)
    {
        zoom /= scale;
        updateViewMatrix();
    }

    /**
     * @brief Applies a scale factor to the viewMatrix.
     * The current scale used in view matrix will be substituted by the given scale factor.
     * @param scale The new scale factor to be applied to View Matrix.
     */
    void applyScaleToViewMatrix (float scale)
    {
        zoom = scale;
        updateViewMatrix();
    }

    /**
     * @brief Translates the view matrix by a given vector trans.
     * @param translation Given translation vector.
     */
    void translateViewMatrix (const Eigen::Vector3f& translation)
    {
        translate(translation);
    }


    /**
     * @brief Applies all trackball's transformations to the view matrix.
     */
    void updateViewMatrix (void)
    {
        resetViewMatrix();
        translate(defaultTranslation);
        rotate(quaternion);
        translate(translationVector);
        scale(zoom);
    }

    /**
     * @brief Creates the circle representating rotation around Z axis.
     * The other circles will be created by simply rotating this one around the desired axis.
     */
    void createTrackballRepresentation (void)
    {
        int index = 0;
        for (float theta = 0; theta<2*M_PI; theta += (2*M_PI)/200.0) {

            vertices[index] = 0.8*cos(theta);
            vertices[index+1] = 0.8*sin(theta);
            vertices[index+2] = 0.0f;
            vertices[index+3] = 1.0f;
            index += 4;
        }
    }

   
  
};

}
#endif
