# cs_embedded_software — Project file structure

Top-level layout for the embedded software course project.

```
cs_embedded_software/
├── .github/                # CI workflows, issue templates
├── build/                  # Build artifacts (ignored in VCS)
├── docs/                   # Design docs, slides, lab instructions
├── examples/               # Example applications and usage
├── include/                # Public headers
├── src/                    # Source code (drivers, app, board)
│   ├── board/              # Board support package
│   ├── drivers/            # Peripheral drivers
│   └── app/                # Application code
├── tests/                  # Unit and integration tests
├── scripts/                # Helper scripts (flash, debug, ci)
├── tools/                  # Tooling, cross-compilers, utilities
├── third_party/            # External libraries and submodules
├── Makefile                # Top-level build file
├── CMakeLists.txt          # Optional CMake project file
├── README.md               # This file
└── LICENSE
```

Use this as a starting point; adapt directories to your board, toolchain, and workflow.
```
