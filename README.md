# WE_MatrixTRS

The "MatrixTRS_class" file contains a SceneScript class with useful methods that handles transformation matrices in Wallpaper Engine.
You can import this class into a single script in your project and create objects from it in shared variables, allowing you to access its methods from anywhere in the project.

Alternatively, if you only need one or two functions or find it too cumbersome to work with the entire class, you can use the "MatrixTRS_library" file, which contains all the functions as standalone entities.

Please note that these functions are specifically designed and optimized to work with transformation matrices represented as flat arrays.
They may produce errors or incorrect results if used with general matrices or nested arrays.
