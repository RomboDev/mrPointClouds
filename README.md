# mrPointClouds

While this is for the now defunct mentalray renderer it could still be of some interest because it sports fully featured octree implementation with and without SSE support together with various traversals, REYES splatting, multithreaded (windows) code, global illumination point rasterizer (micro rasterizer), diffusion SSS and ambient occlusion and heavy use of C++ templates. It's tested for compilation with ICC on Windows.

Simply start by including _cpc_mentalray.h in your project. 

Actual mr shaders are those .cpp out of folders, while folders contain implementations and data stuctures.

If you don't plan to use mentalray and just use point containers, surfel nodes, octree etc. to collect and process points just disable mentalray support in the _cpc_mentalray.h by setting to 0 __MENTALRAY_ENHANCED__	define.

![Doc1](https://github.com/RomboDev/mrPointClouds/blob/master/Docs/001_PCloud_Bake.png?raw=true)
![Doc2](https://github.com/RomboDev/mrPointClouds/blob/master/Docs/002_PCloud_Shade.png?raw=true)
![Doc3](https://github.com/RomboDev/mrPointClouds/blob/master/Docs/003_PCloud_Visualizer.png?raw=true)
![Doc4](https://github.com/RomboDev/mrPointClouds/blob/master/Docs/004_PCloud_Bake_SSS.png?raw=true)
![Doc5](https://github.com/RomboDev/mrPointClouds/blob/master/Docs/005_PCloud_Eval_SSS.png?raw=true)
![Doc6](https://github.com/RomboDev/mrPointClouds/blob/master/Docs/006_PCloud_Examples.png?raw=true)
