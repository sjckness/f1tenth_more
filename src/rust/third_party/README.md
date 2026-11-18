# Vendored third-party sources for the Rust packages

`COLCON_IGNORE` keeps colcon out of this directory (QDLDL has a `CMakeLists.txt`,
which colcon would otherwise build as a package of its own).

## qdldl/: QDLDL v0.1.8

| | |
|---|---|
| What | QDLDL, the LDL^T factorisation OSQP C uses for its KKT system |
| Upstream | https://github.com/osqp/qdldl, tag `v0.1.8` (a lightweight tag) |
| Commit | `138fdac58b9cd1c4137ff1b99152c8108a6cff5b` (2025-03-17, "Merge pull request #59 from osqp/im/v0.1.8") |
| Git tree of that commit | `7bbce655e64e74c476ca4694c06d7bfb08a5cf3d` |
| License | Apache-2.0, `qdldl/LICENSE` (unchanged) |
| Content | the tag's tree, every file, unmodified; only `.git` left out |
| Vendored | 2026-10-24 (R0b), `git clone --depth 1 --branch v0.1.8` |

**Why.** `f1tenth_mpc_core` links OSQP C 1.0.0 through `osqp-sys =1.0.1`, whose `build.rs`
builds OSQP with CMake. OSQP's `algebra/_common/lin_sys/qdldl/qdldl.cmake` declares QDLDL with
`FetchContent` (`GIT_REPOSITORY https://github.com/osqp/qdldl.git`, `GIT_TAG v0.1.8`), so every
clean build cloned it from GitHub: the Orin image build needed network for it, and a GitHub
outage or a moved tag would break or change the build. The online builds before R0b fetched
exactly this commit (their `_deps/qdldl-src` is identical to `qdldl/` here, checked with
`diff -r`).

**How it is used.** `../build_env.sh` sets `CMAKE_TOOLCHAIN_FILE_<host triple>` to
`osqp_offline.cmake`; the `cmake` crate passes that file to OSQP's CMake, which then takes
QDLDL from `FETCHCONTENT_SOURCE_DIR_QDLDL` (this `qdldl/`) with
`FETCHCONTENT_FULLY_DISCONNECTED` on. QDLDL compiles as before, only from here: the build tree
has `_deps/qdldl-build` and no `_deps/qdldl-subbuild` (no download step). Proof and the
unchanged solver results: `output/rust_main/r0b.md`.

**Check the copy** (after it is committed; the tree hash covers every file and its mode):

```bash
git rev-parse HEAD:src/rust/third_party/qdldl     # must print 7bbce655e64e74c476ca4694c06d7bfb08a5cf3d
```

or against upstream: `git clone --depth 1 --branch v0.1.8 https://github.com/osqp/qdldl.git
/tmp/qdldl && diff -r -x .git /tmp/qdldl src/rust/third_party/qdldl` (no output).

**Updating.** Only together with `osqp-sys` (its OSQP's `qdldl.cmake` names the tag): replace
the directory with the new tag's tree, update this table, and re-run
`tools/rust_parity/mpc/parity_solver.py`. Do not edit files in `qdldl/`.
