![Build-github](https://github.com/buq2/camera_calibrator/actions/workflows/main.yml/badge.svg)

# Camera calibrator

Hope is to build faster and easier to use camera calibrator than what OpenCV has.

# Build

```
cmake -S . -B build -A x64 -DCMAKE_BUILD_TYPE=Release
# Or on MSVC command prompt 
# cmake -S . -B build -G Ninja
cmake --build build --parallel 12 --config Release
```

## Specific VS version 

```
cmake -S . -B build -A x64 -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017"
cmake -S . -B build_debug -A x64 -DCMAKE_BUILD_TYPE=Debug -G "Visual Studio 15 2017"
```

# Debug build

```
cmake -S . -B build_debug -A x64 -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug --parallel 12 --config Debug
```

# Build python bindings

```
CMAKE_BUILD_PARALLEL_LEVEL=8 CMAKE_GENERATOR="Visual Studio 17 2022" pip install -v -e .
```

# Tests

```
ctest --test-dir build
```

# cppcheck

Windows cmd
```
docker run --rm -v "%cd%":/data frankwolf/cppcheck --verbose --enable=all --inconclusive --language=c++ --suppress=missingIncludeSystem --suppress=unusedFunction --error-exitcode=1 src
```

Linux
```
docker run --rm -v "$PWD":/data frankwolf/cppcheck --verbose --enable=all --inconclusive --language=c++ --suppress=missingIncludeSystem --suppress=unusedFunction --error-exitcode=1 src
```

# clang-format

On git bash on Windows
```
docker run --rm -it -v `pwd -W`:/workdir unibeautify/clang-format -i -style=Google **/{*.cpp,*.hh}
``` 

# Docker

```
docker build -t calibrator .
docker run --rm -t calibrator /bin/bash -c 'cd build && ctest'
```
