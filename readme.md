# Camera calibrator

Hope is to build faster and easier to use camera calibrator than what OpenCV has.

# Build

```
cmake -S . -B build -A x64
# Or on MSVC command prompt 
# cmake -S . -B build -G Ninja
cmake --build build --parallel 12 --config Release
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
