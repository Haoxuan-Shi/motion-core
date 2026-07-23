# Contributing

Contributions should keep the library dependency-free at runtime and preserve
deterministic ordering and fixed iteration counts.

1. Create a focused branch and add a regression test under `tests/`.
2. Configure with `cmake -S . -B build`.
3. Build with `cmake --build build --config Release`.
4. Run `ctest --test-dir build -C Release --output-on-failure`.
5. Update the README, architecture document, and changelog when public behavior
   changes.

Use C++17, avoid undefined behavior and hidden global state, validate all public
numeric inputs, and keep generated files out of commits. New controller claims
must be matched by tests and stated with their numerical limitations.

