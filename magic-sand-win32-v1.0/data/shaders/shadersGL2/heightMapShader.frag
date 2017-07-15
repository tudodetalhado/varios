#version 120

varying float depthfrag;

uniform sampler2DRect heightColorMapSampler;
uniform sampler2DRect pixelCornerElevationSampler; // Sampler for the half pixel texture
uniform float contourLineFactor;
uniform int drawContourLines;

void main()
{
    vec2 depthPos = vec2(depthfrag, 0.5);//depthvalue*texsize, 0.5);
    vec4 color =  texture2DRect(heightColorMapSampler, depthPos);	//colormap converted depth

    if (drawContourLines == 1)
    {
        // Contour line computation
        /* Calculate the contour line interval containing each pixel corner by evaluating the half-pixel offset elevation texture: */
        float corner0=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x,gl_FragCoord.y)).r*contourLineFactor);
        float corner1=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x+1.0,gl_FragCoord.y)).r*contourLineFactor);
        float corner2=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x,gl_FragCoord.y+1.0)).r*contourLineFactor);
        float corner3=floor(texture2DRect(pixelCornerElevationSampler,vec2(gl_FragCoord.x+1.0,gl_FragCoord.y+1.0)).r*contourLineFactor);
        
        /* Find all pixel edges that cross at least one contour line: */
        int edgeMask=0;
        int numEdges=0;
        if(corner0!=corner1)
        {
            edgeMask+=1;
            ++numEdges;
        }
        if(corner2!=corner3)
        {
            edgeMask+=2;
            ++numEdges;
        }
        if(corner0!=corner2)
        {
            edgeMask+=4;
            ++numEdges;
        }
        if(corner1!=corner3)
        {
            edgeMask+=8;
            ++numEdges;
        }
        
        /* Check for all cases in which the pixel should be colored as a topographic contour line: */
        if(numEdges>2||edgeMask==3||edgeMask==12||(numEdges==2&&mod(floor(gl_FragCoord.x)+floor(gl_FragCoord.y),2.0)==0.0))
        {
            /* Topographic contour lines are rendered in black: */
            color=vec4(0.0,0.0,0.0,1.0);
        }
    }

    gl_FragColor = color;
}
