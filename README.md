# WE_MatrixTRS

This SceneScript library can help you when you are dealing with transformation matrices in Wallpaper Engine, like for puppet warp's bones (currently only the matrix is provided for them instead of the normal origin, angles and scale properties), or when you have view vectors instead of angles, like for camera trasforms, or generally when you want to do more complex things with the origin, angles and scale properties, for example: rotations around any arbitrary point in space, smooth interpolation between two instances of the 3 properties or reflections and projections.

The "MatrixTRS_library" file contains many stand alone functions that you can import singolarly in your scripts to do what you need. The "MatrixTRS_class", on the other end, contains a class with all the same functions as methods for it's objects, you can import the class into a single script in your project and create objects from it in shared variables, allowing you to access its methods from anywhere in the project without having to import it in every script.

The use of this requires at least basic knowledge about game development or linear algebra. The functions provided here will do all the complex math for you, but you still need to know how to use them.

Please note that these functions are specifically designed and optimized to work with 4x4 transformation matrices represented as flat arrays.
They may produce errors or incorrect results if used with general matrices or nested arrays.
Also, all the functions that ask the angles in input, need them to be in degrees, same thing for those that return angles.
