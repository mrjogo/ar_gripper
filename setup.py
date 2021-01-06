# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

import os
import re
from distutils.command.install_lib import install_lib
from distutils import log

# Replacement for setuptools.find_packages(); setuptools isn't
# recommended with catkin[1], and spews warnings
#
# [1]: http://docs.ros.org/melodic/api/catkin/html/user_guide/setup_dot_py.html
#
# Warnings   << robot_command:install [...]/logs/robot_command/build.install.001.log
# zip_safe flag not set; analyzing archive contents...
# robot_command.program_interpreter.interpreter: module MAY be using inspect.getframeinfo
# robot_command.program_interpreter.interpreter: module MAY be using inspect.stack
# robot_command.rpl.command: module MAY be using inspect.getframeinfo
# robot_command.rpl.command: module MAY be using inspect.stack
# cd [...]/build/robot_command; catkin build --get-env robot_command | catkin env -si  /usr/bin/make install; cd -
# ........................................................................................


def find_packages(path):
    return [
        re.sub('^[^A-z0-9_]+', '', root[len(path) + 1 :].replace('/', '.'))
        for root, dirs, files in os.walk(path)
        if '__init__.py' in files and not root.endswith('/testing')
    ]


# Catkin build runs something like this:
#
# python setup.py \
#     build --build-base ${CMAKE_CURRENT_SOURCE_DIR} \
#     install \
#     $DESTDIR_ARG \
#     --install-layout=deb --prefix=${CMAKE_INSTALL_PREFIX} \
#     --install-scripts=${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}

# In setuptools, convert this to a binary-only distribution by
# replacing that with something like this:
#
# python setup.py \
#     build --build-base ${CMAKE_CURRENT_BINARY_DIR} \
#     bdist_egg --exclude-source-files --dist-dir=@GEN_DIR@/egg
# easy_install \
#     --no-deps --prefix=${CMAKE_INSTALL_PREFIX} --always-unzip \
#     --script-dir="${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}" \
#     ${PROJECT_NAME}-*-py*.egg


class InstallLibSourceless(install_lib):
    def remove_uncompiled_python(self, package):
        path = os.path.join(self.install_dir, package.replace('.', '/'))
        log.info("Removing uncompiled python sources in " + path)
        for f in os.listdir(path):
            file_path = os.path.join(path, f)
            if os.path.isdir(file_path):
                continue
            if not f.endswith('.py'):
                continue
            if not os.path.exists(file_path + 'c'):
                continue  # Don't erase uncompiled sources
            os.unlink(file_path)

    def run(self):
        install_lib.run(self)
        for p in self.distribution.packages:
            self.remove_uncompiled_python(p)


# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'},
    # Patch install_lib command
    cmdclass=dict(install_lib=InstallLibSourceless),
)

setup(**setup_args)
