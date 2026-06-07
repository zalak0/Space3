# GOOSE H743 ADS Real Sensor Backend Bring-up

This file tracks the bring-up path from the proven fake ADS backend to the real H743 ADS sensor backend.

## Current proven runtime states

### Fake backend selected

CMake:

```cmake
set(GOOSE_ADS_H743_REAL_BACKEND 0)