### Docker

* Go to the [wiki](https://github.com/introlab/rtabmap/wiki/Installation#docker) for usage examples and how to build locally the images.

#### Tags

All images are published to [introlab3it/rtabmap](https://hub.docker.com/r/introlab3it/rtabmap) as
multi-arch manifests (`linux/amd64` and `linux/arm64`).

| Tag | Alias | Base | ROS |
| --- | --- | --- | --- |
| `resolute` | `26.04`, `latest` | Ubuntu 26.04 | ROS2 Lyrical |
| `noble-kilted` | | Ubuntu 24.04 | ROS2 Kilted |
| `noble` | `24.04` | Ubuntu 24.04 | ROS2 Jazzy |
| `jammy` | `22.04` | Ubuntu 22.04 | ROS2 Humble |
| `focal` | `20.04` | Ubuntu 20.04 | ROS1 Noetic |

Each image is built on top of a matching `<tag>-deps` image holding the third-party
dependencies, so that a source change only rebuilds the top layer.

> [!IMPORTANT]
> **`latest` now points to the newest ROS2 image** (`resolute`), where it used to point
> to `focal` (ROS1 Noetic). Pulling `introlab3it/rtabmap` without a tag therefore gets you
> a different ROS version than before. Use `introlab3it/rtabmap:focal` to stay on ROS1, or
> pin an explicit tag in general rather than relying on `latest`.

`bionic` / `18.04` (ROS1 Melodic) is no longer built; the last published image stays on
Docker Hub but will not be updated. Its Dockerfile is kept in [bionic/](bionic) for reference.

The `<tag>-amd64` / `<tag>-arm64` tags are per-architecture build outputs that CI joins into
the manifests above (see [.github/workflows/docker-ros.yml](../.github/workflows/docker-ros.yml));
use the plain tags instead.

Android build environments (`android23`, `android24`, `android26`, `android30`, `tango`) are
`linux/amd64` only and are built by
[.github/workflows/android.yml](../.github/workflows/android.yml).
