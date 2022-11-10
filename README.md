# Visual-Servo---Paper
Este repositorio fue hecho para implementar una modificación al algoritmo descrito en el paper "Image-based estimation, planning, and control for high-speed flying through multiple openings" de Guo, Dejun and Leang

1. Se ejecuta vc_new_controller.launch
2. Este mismo ejecuta de primeras: chaumette.cpp (línea 159)
3. Este ejecuta internamente a compute_descriptors.cpp (línea 14)
4. Después camera_norm.cpp (línea 27)
5. interaction_Mat.cpp (Línea 37)
6. Moore_Penrose_PInv.cpp (Línea 42)
7. Se actualiza los valores de velocidad, se integran y se publican en el dron
