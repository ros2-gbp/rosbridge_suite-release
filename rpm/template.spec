%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-rosbridge-library
Version:        1.3.1
Release:        3%{?dist}%{?release_suffix}
Summary:        ROS rosbridge_library package

License:        BSD
URL:            http://ros.org/wiki/rosbridge_library
Source0:        %{name}-%{version}.tar.gz

Requires:       python%{python3_pkgversion}-bson
Requires:       python3-pillow
Requires:       ros-iron-rclpy
Requires:       ros-iron-rosidl-default-runtime
Requires:       ros-iron-ros-workspace
BuildRequires:  python%{python3_pkgversion}-bson
BuildRequires:  python3-pillow
BuildRequires:  ros-iron-ament-cmake
BuildRequires:  ros-iron-ament-cmake-ros
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-iron-actionlib-msgs
BuildRequires:  ros-iron-ament-cmake-pytest
BuildRequires:  ros-iron-builtin-interfaces
BuildRequires:  ros-iron-diagnostic-msgs
BuildRequires:  ros-iron-example-interfaces
BuildRequires:  ros-iron-geometry-msgs
BuildRequires:  ros-iron-nav-msgs
BuildRequires:  ros-iron-rosbridge-test-msgs
BuildRequires:  ros-iron-sensor-msgs
BuildRequires:  ros-iron-std-msgs
BuildRequires:  ros-iron-std-srvs
BuildRequires:  ros-iron-stereo-msgs
BuildRequires:  ros-iron-tf2-msgs
BuildRequires:  ros-iron-trajectory-msgs
BuildRequires:  ros-iron-visualization-msgs
%endif

%description
The core rosbridge package, responsible for interpreting JSON andperforming the
appropriate ROS action, like subscribe, publish, call service, and interact with
params.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/iron" \
    -DAMENT_PREFIX_PATH="/opt/ros/iron" \
    -DCMAKE_PREFIX_PATH="/opt/ros/iron" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/iron

%changelog
* Thu Apr 20 2023 Jihoon Lee <jihoonlee.in@gmail.com> - 1.3.1-3
- Autogenerated by Bloom

* Tue Mar 21 2023 Jihoon Lee <jihoonlee.in@gmail.com> - 1.3.1-2
- Autogenerated by Bloom

