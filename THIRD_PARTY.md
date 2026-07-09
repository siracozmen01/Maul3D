# Third-party licenses

The engine itself has zero runtime dependencies. Two categories of
outside work are acknowledged here:

## Adapted code

Several kernels are adapted from the Box2D / Box3D lineage by Erin
Catto, MIT licensed: the soft-step solver stage structure, joint
formulations (revolute, prismatic, spherical limits, the softness
model), collision and ray-cast kernels, and the trigonometric
approximations. Adaptations are noted in the source files where
they occur.

```
Copyright (c) Erin Catto

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Testbed-only dependencies

The interactive testbed (never the engine) fetches
[raylib](https://github.com/raysan5/raylib) (zlib/libpng license)
at configure time when built with `-DMAUL3D_BUILD_TESTBED=ON`. The
engine library links against nothing.
