# gtsam-project-python

Prior factor for a 2d vector, created by Vladimir Tchuiev. Prelude to a larger 4-way factor for infering posterior epistemic classifier uncertainty.

## PREREQUISITES

- Python 3.6+ is required, since we support these versions.
- Make the following changes to `GTSAM`:

  - Set the CMake flag `GTSAM_INSTALL_CYTHON_TOOLBOX` to `ON` to enable building the cython wrapper.
  - Set the CMake flag `GTSAM_PYTHON_VERSION` to `3`, otherwise the default interpreter will be used.
  - You can do this on the command line as follows:

    ```sh
    cmake -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON -DGTSAM_PYTHON_VERSION=3 ..
    ```

## INSTALL

- In the `example` directory, create the `build` directory and `cd` into it.
- Run `cmake ..`.
- Run `make`, and the wrapped module will be installed to a `cython` directory.
- Navigate to the `cython` directory and run `python setup.py install`.

## DOCUMENTATION

For more detailed information, please refer to the [tutorial](TUTORIAL.md). I Don't have the time currently to make a detailed ReadMe for my specific case, may do that later.
Note that the namespace in the CMakeLists.txt must be identical to the ones that appear in the .cpp and .h files, so I changed it manually to "gtsam". Note also that PyObject* type doesn't work well with
the wrapper, so had to change it to gtsam::Vector.
